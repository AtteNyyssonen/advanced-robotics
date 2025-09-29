#include "arm_controllers/velocity_controller.hpp"

#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>

#include <Eigen/Dense>
#include <algorithm>
#include <memory>

namespace arm_controllers
{

VelocityController::VelocityController()
: ControllerInterface(),
  elapsed_time_(0.0),
  traj_duration_(5.0),
  goal_active_(false)
{
}

controller_interface::CallbackReturn VelocityController::on_init()
{
  // declare parameters (use auto_declare recommended by controller_interface)
  auto_declare<std::string>("urdf_param", "robot_description");
  auto_declare<std::string>("root_link", "panda_link0");
  auto_declare<std::string>("tip_link", "panda_link7");
  auto_declare<std::vector<std::string>>("joints", std::vector<std::string>{});
  auto_declare<std::vector<double>>("kd_gains", std::vector<double>{});
  auto_declare<double>("traj_duration", 5.0);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration VelocityController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & j : joints_) {
    cfg.names.push_back(j + "/velocity");
  }
  return cfg;
}

controller_interface::InterfaceConfiguration VelocityController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & j : joints_) {
    cfg.names.push_back(j + "/position");
    cfg.names.push_back(j + "/velocity");
  }
  return cfg;
}

controller_interface::CallbackReturn VelocityController::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_node()->get_logger(), "Configuring VelocityController");

  urdf_param_ = get_node()->get_parameter("urdf_param").as_string();
  root_link_ = get_node()->get_parameter("root_link").as_string();
  tip_link_ = get_node()->get_parameter("tip_link").as_string();
  joints_ = get_node()->get_parameter("joints").as_string_array();

  kd_gains_ = get_node()->get_parameter("kd_gains").as_double_array();
  if (kd_gains_.size() != joints_.size()) {
    kd_gains_.assign(joints_.size(), 10.0);
  }

  traj_duration_ = get_node()->get_parameter("traj_duration").as_double();

  if (!init_kdl_from_urdf(urdf_param_)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to init KDL from URDF");
    return controller_interface::CallbackReturn::ERROR;
  }

  pub_pos_ = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>("joint_positions", rclcpp::SystemDefaultsQoS());
  pub_vel_ = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>("joint_velocities", rclcpp::SystemDefaultsQoS());
  pub_desired_vel_ = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>("joint_velocities_desired", rclcpp::SystemDefaultsQoS());
  pub_error_ = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>("joint_velocity_error", rclcpp::SystemDefaultsQoS());
  pub_control_out_ = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>("control_output", rclcpp::SystemDefaultsQoS());

  ee_goal_subscriber_ = get_node()->create_subscription<geometry_msgs::msg::Point>(
    "cartesian_goal", rclcpp::SystemDefaultsQoS(),
    [this](const geometry_msgs::msg::Point::SharedPtr msg) {
      KDL::ChainFkSolverPos_recursive fk_solver(kdl_chain_);
      fk_solver.JntToCart(kdl_q_, ee_start_);
      ee_goal_ = KDL::Frame(KDL::Vector(msg->x, msg->y, msg->z));
      elapsed_time_ = 0.0;
      goal_active_ = true;
      RCLCPP_INFO(get_node()->get_logger(), "Received cartesian_goal: x=%.3f y=%.3f z=%.3f",
                  msg->x, msg->y, msg->z);
    });

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn VelocityController::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_node()->get_logger(), "Activating VelocityController");

  velocity_command_handles_.clear();
  position_state_handles_.clear();
  velocity_state_handles_.clear();

  for (const auto & joint : joints_) {
    auto cmd_iface = std::find_if(
      command_interfaces_.begin(), command_interfaces_.end(),
      [&](const auto & ci){ return ci.get_name() == (joint + "/velocity"); });
    if (cmd_iface == command_interfaces_.end()) {
      RCLCPP_ERROR(get_node()->get_logger(), "Missing velocity command interface for joint '%s'", joint.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
    velocity_command_handles_.emplace_back(std::move(*cmd_iface));

    auto pos_iface = std::find_if(
      state_interfaces_.begin(), state_interfaces_.end(),
      [&](const auto & si){ return si.get_name() == (joint + "/position"); });
    auto vel_iface = std::find_if(
      state_interfaces_.begin(), state_interfaces_.end(),
      [&](const auto & si){ return si.get_name() == (joint + "/velocity"); });

    if (pos_iface == state_interfaces_.end() || vel_iface == state_interfaces_.end()) {
      RCLCPP_ERROR(get_node()->get_logger(), "Missing state interfaces for %s", joint.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
    position_state_handles_.emplace_back(std::move(*pos_iface));
    velocity_state_handles_.emplace_back(std::move(*vel_iface));
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn VelocityController::on_deactivate(const rclcpp_lifecycle::State &)
{
  for (auto & h : velocity_command_handles_) {
    h.set_value(0.0);
  }
  goal_active_ = false;
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type VelocityController::update(const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  const size_t n = joints_.size();
  if (n == 0) return controller_interface::return_type::OK;

  if ((size_t)kdl_q_.rows() != n) {
    kdl_q_ = KDL::JntArray(n);
    kdl_qdot_ = KDL::JntArray(n);
    coriolis_ = KDL::JntArray(n);
    mass_matrix_ = KDL::JntSpaceInertiaMatrix(n);
  }
  for (size_t i = 0; i < n; ++i) {
    kdl_q_(i) = position_state_handles_[i].get_value();
    kdl_qdot_(i) = velocity_state_handles_[i].get_value();
  }

  std::vector<double> pos_vec(n), vel_vec(n), desired_vel_vec(n), err_vec(n), out_vec(n, 0.0);
  for (size_t i=0;i<n;++i){ pos_vec[i] = kdl_q_(i); vel_vec[i] = kdl_qdot_(i); }

  std::vector<double> qdot_d_vec(n, 0.0);

  if (goal_active_) {
    elapsed_time_ += period.seconds();
    double s = std::min(elapsed_time_ / traj_duration_, 1.0);

    KDL::Vector start = ee_start_.p;
    KDL::Vector goal  = ee_goal_.p;
    KDL::Vector xdot_d = (goal - start) / traj_duration_;

    KDL::Jacobian J(n);
    KDL::ChainJntToJacSolver jac_solver(kdl_chain_);
    jac_solver.JntToJac(kdl_q_, J);

    Eigen::MatrixXd J_eig(6, (int)n);
    for (unsigned r=0;r<6;++r)
      for (unsigned c=0;c<(int)n;++c)
        J_eig(r,c) = J(r,c);

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(J_eig, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::VectorXd S = svd.singularValues();
    Eigen::VectorXd Sinv = S;
    for (int i=0;i<S.size();++i) Sinv(i) = (S(i) > 1e-6) ? 1.0 / S(i) : 0.0;
    Eigen::MatrixXd J_pinv = svd.matrixV() * Sinv.asDiagonal() * svd.matrixU().transpose();

    Eigen::VectorXd dx(6);
    dx << xdot_d.x(), xdot_d.y(), xdot_d.z(), 0,0,0;

    Eigen::VectorXd qdot_d = J_pinv * dx;

    for (size_t i=0;i<n;++i) {
      qdot_d_vec[i] = qdot_d(i);
      desired_vel_vec[i] = qdot_d(i);
    }
    
    Eigen::VectorXd qdot_eig(n);
    for (size_t i=0;i<n;++i) qdot_eig(i) = kdl_qdot_(i);
    Eigen::VectorXd e = qdot_d - qdot_eig;

    kdl_dyn_->JntToMass(kdl_q_, mass_matrix_);
    KDL::JntArray cori(n), grav(n);
    kdl_dyn_->JntToCoriolis(kdl_q_, kdl_qdot_, cori);
    kdl_dyn_->JntToGravity(kdl_q_, grav);

    for (size_t i=0;i<n;++i) {
      double kd = (i < kd_gains_.size()) ? kd_gains_[i] : 1.0;
      double mii = 1.0;
      if (mass_matrix_.rows() == (int)n) {
        mii = mass_matrix_(i,i);
        if (!(mii > 0.0)) mii = 1.0;
      }
      double vcmd = kd * e(i) / (mii + 1e-6);
      out_vec[i] = vcmd;
      err_vec[i] = e(i);
    }

    if (s >= 1.0) {
      goal_active_ = false;
      RCLCPP_INFO(get_node()->get_logger(), "Cartesian goal reached");
    }
  } else {
    for (size_t i=0;i<n;++i) {
      desired_vel_vec[i] = 0.0;
      err_vec[i] = 0.0;
      out_vec[i] = 0.0;
    }
  }

  for (size_t i=0;i<velocity_command_handles_.size();++i) {
    velocity_command_handles_[i].set_value(out_vec[i]);
  }

  auto pub_arr = [&](rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr p, const std::vector<double> &d){
    std_msgs::msg::Float64MultiArray msg; msg.data = d; if (p) p->publish(msg);
  };
  pub_arr(pub_pos_, pos_vec);
  pub_arr(pub_vel_, vel_vec);
  pub_arr(pub_desired_vel_, desired_vel_vec);
  pub_arr(pub_error_, err_vec);
  pub_arr(pub_control_out_, out_vec);

  return controller_interface::return_type::OK;
}

bool VelocityController::init_kdl_from_urdf(const std::string & urdf_param)
{
  std::string urdf_xml;
  if (!get_node()->get_parameter(urdf_param, urdf_xml)) {
    rclcpp::Parameter p;
    if (!get_node()->get_parameter(urdf_param, p)) {
      RCLCPP_ERROR(get_node()->get_logger(), "URDF parameter '%s' not found", urdf_param.c_str());
      return false;
    } else {
      urdf_xml = p.as_string();
    }
  }

  if (!kdl_parser::treeFromString(urdf_xml, kdl_tree_)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to parse URDF to KDL tree");
    return false;
  }
  if (!kdl_tree_.getChain(root_link_, tip_link_, kdl_chain_)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to get KDL chain from %s to %s", root_link_.c_str(), tip_link_.c_str());
    return false;
  }

  KDL::Vector gravity(0.0, 0.0, -9.81);
  kdl_dyn_.reset(new KDL::ChainDynParam(kdl_chain_, gravity));
  kdl_q_ = KDL::JntArray(kdl_chain_.getNrOfJoints());
  kdl_qdot_ = KDL::JntArray(kdl_chain_.getNrOfJoints());
  coriolis_ = KDL::JntArray(kdl_chain_.getNrOfJoints());
  mass_matrix_ = KDL::JntSpaceInertiaMatrix(kdl_chain_.getNrOfJoints());
  return true;
}

}

PLUGINLIB_EXPORT_CLASS(arm_controllers::VelocityController, 
    controller_interface::ControllerInterface)
