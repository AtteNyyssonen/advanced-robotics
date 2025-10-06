#include "arm_controllers/gravity_pd_controller.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <urdf/model.h>

namespace arm_controllers
{

GravityPDController::GravityPDController()
: controller_interface::ControllerInterface(),
  gravity_vec_(0.0, 0.0, -9.81)
{
}

controller_interface::CallbackReturn GravityPDController::on_init()
{
  auto_declare<std::vector<std::string>>("joints", std::vector<std::string>{});
  auto_declare<std::string>("urdf_param", "robot_description");
  auto_declare<std::string>("root_link", "");
  auto_declare<std::string>("tip_link", "");
  auto_declare<std::vector<double>>("kp_cartesian_gains", std::vector<double>{});
  auto_declare<std::vector<double>>("kd_cartesian_gains", std::vector<double>{});
  auto_declare<double>("goal_tolerance", 0.01);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration GravityPDController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint_name : joint_names_) {
    config.names.push_back(joint_name + "/velocity");
  }
  return config;
}

controller_interface::InterfaceConfiguration GravityPDController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint_name : joint_names_) {
    config.names.push_back(joint_name + "/position");
    config.names.push_back(joint_name + "/velocity");
  }
  return config;
}

controller_interface::CallbackReturn GravityPDController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto node = get_node();
  joint_names_ = node->get_parameter("joints").as_string_array();
  urdf_param_ = node->get_parameter("urdf_param").as_string();
  root_link_  = node->get_parameter("root_link").as_string();
  tip_link_   = node->get_parameter("tip_link").as_string();
  
  std::string urdf_string;
  if (!node->get_parameter(urdf_param_, urdf_string)) {
    RCLCPP_ERROR(node->get_logger(), "Failed to get URDF from parameter [%s]", urdf_param_.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }
  KDL::Tree tree;
  if (!kdl_parser::treeFromString(urdf_string, tree)) {
    RCLCPP_ERROR(node->get_logger(), "Failed to extract KDL tree from URDF");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (!tree.getChain(root_link_, tip_link_, chain_)) {
    RCLCPP_ERROR(node->get_logger(), "Failed to extract KDL chain from %s to %s",
                 root_link_.c_str(), tip_link_.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }

  const auto kdl_n = chain_.getNrOfJoints();
  if (kdl_n == 0) {
      RCLCPP_ERROR(node->get_logger(), "KDL chain has zero joints.");
      return controller_interface::CallbackReturn::ERROR;
  }
  if (static_cast<size_t>(kdl_n) != joint_names_.size()) {
    RCLCPP_ERROR(node->get_logger(), "Mismatch between KDL chain joints (%d) and provided joint names (%zu).",
                 kdl_n, joint_names_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  q_kdl_.resize(kdl_n);
  qdot_kdl_.resize(kdl_n);
  J_kdl_.resize(kdl_n);
  G_kdl_.resize(kdl_n);

  fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain_);
  jac_solver_ = std::make_unique<KDL::ChainJntToJacSolver>(chain_);
  dyn_solver_ = std::make_unique<KDL::ChainDynParam>(chain_, gravity_vec_);

  auto kp_gains = node->get_parameter("kp_cartesian_gains").as_double_array();
  if (kp_gains.size() != 6) {
    RCLCPP_ERROR(node->get_logger(), "Parameter 'kp_cartesian_gains' must have 6 values.");
    return controller_interface::CallbackReturn::ERROR;
  }
  Kp_cartesian_.resize(6);
  for (size_t i = 0; i < 6; ++i) Kp_cartesian_(i) = kp_gains[i];

  auto kd_gains = node->get_parameter("kd_cartesian_gains").as_double_array();
  if (kd_gains.size() != 6) {
    RCLCPP_ERROR(node->get_logger(), "Parameter 'kd_cartesian_gains' must have 6 values.");
    return controller_interface::CallbackReturn::ERROR;
  }
  Kd_cartesian_.resize(6);
  for (size_t i = 0; i < 6; ++i) Kd_cartesian_(i) = kd_gains[i];
  
  goal_tolerance_ = node->get_parameter("goal_tolerance").as_double();

  goal_subscriber_ = node->create_subscription<geometry_msgs::msg::Point>(
    "/cartesian_goal", rclcpp::SystemDefaultsQoS(),
    [this](const geometry_msgs::msg::Point::SharedPtr msg) {
      ee_goal_ = KDL::Frame(KDL::Vector(msg->x, msg->y, msg->z));
      goal_active_ = true;
      RCLCPP_INFO(get_node()->get_logger(), "Received new Cartesian goal: [x: %.2f, y: %.2f, z: %.2f]",
                  msg->x, msg->y, msg->z);
    });

  RCLCPP_INFO(node->get_logger(), "Configured Cartesian PD controller with %d joints", kdl_n);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn GravityPDController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  goal_active_ = false;
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn GravityPDController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (auto & command_interface : command_interfaces_) {
    command_interface.set_value(0.0);
  }
  goal_active_ = false;
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type GravityPDController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  const size_t n_joints = joint_names_.size();

  for (size_t i = 0; i < n_joints; i++) {
    q_kdl_(i) = state_interfaces_[2 * i].get_value();       // position
    qdot_kdl_(i) = state_interfaces_[2 * i + 1].get_value(); // velocity
  }

  Eigen::VectorXd tau_cmd = Eigen::VectorXd::Zero(n_joints);

  if (goal_active_) {
    // Forward Kinematics to get current end-effector pose
    KDL::Frame x_current;
    fk_solver_->JntToCart(q_kdl_, x_current);

    KDL::Vector pos_error = ee_goal_.p - x_current.p;
    
    if (pos_error.Norm() < goal_tolerance_) {
      goal_active_ = false;
      RCLCPP_INFO(get_node()->get_logger(), "Goal reached. Holding position.");
    }
    
    Eigen::VectorXd x_error(6);
    x_error << pos_error.x(), pos_error.y(), pos_error.z(), 0, 0, 0;
    KDL::JntArrayVel q_kdl_vel(q_kdl_, qdot_kdl_);
    KDL::Jacobian J_kdl(chain_.getNrOfJoints());
    // Calculate Jacobian and current Cartesian velocity (x_dot = J * q_dot)
    jac_solver_->JntToJac(q_kdl_, J_kdl_);
    
    Eigen::MatrixXd J = J_kdl.data;
    Eigen::VectorXd qdot = qdot_kdl_.data;

    Eigen::VectorXd xdot_vec = J * qdot;

    KDL::Twist x_dot_current_kdl(
      KDL::Vector(xdot_vec(0), xdot_vec(1), xdot_vec(2)),
      KDL::Vector(xdot_vec(3), xdot_vec(4), xdot_vec(5))
);
    
    Eigen::VectorXd x_dot_current_eigen(6);
    x_dot_current_eigen << x_dot_current_kdl.vel.x(), x_dot_current_kdl.vel.y(), x_dot_current_kdl.vel.z(),
                           x_dot_current_kdl.rot.x(), x_dot_current_kdl.rot.y(), x_dot_current_kdl.rot.z();

    // PD Control Law in Cartesian Space: F = Kp*error - Kd*velocity
    Eigen::VectorXd F_cartesian = Kp_cartesian_.cwiseProduct(x_error) - Kd_cartesian_.cwiseProduct(x_dot_current_eigen);

    // Map Cartesian force to joint torques using Jacobian Transpose: tau = J^T * F
    Eigen::MatrixXd J_eigen = J_kdl_.data;
    Eigen::VectorXd tau_pd = J_eigen.transpose() * F_cartesian;

    // Add gravity compensation torque
    dyn_solver_->JntToGravity(q_kdl_, G_kdl_);
    tau_cmd = tau_pd + G_kdl_.data;

  } else {
    // No active goal: command only gravity compensation torque to hold position
    dyn_solver_->JntToGravity(q_kdl_, G_kdl_);
    tau_cmd = G_kdl_.data;
  }


  for (size_t i = 0; i < n_joints; i++) {
    command_interfaces_[i].set_value(tau_cmd(i));
  }

  return controller_interface::return_type::OK;
}

}

PLUGINLIB_EXPORT_CLASS(arm_controllers::GravityPDController,
  controller_interface::ControllerInterface)
