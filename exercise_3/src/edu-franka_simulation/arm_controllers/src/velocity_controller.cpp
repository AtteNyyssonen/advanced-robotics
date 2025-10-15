#include "arm_controllers/velocity_controller.hpp"

#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <Eigen/Dense>
#include <algorithm>
#include <memory>
#include <string>
#include <vector>

namespace arm_controllers
{

controller_interface::InterfaceConfiguration VelocityController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (int i = 1; i <= num_joints; ++i) {
      config.names.push_back("panda_joint" + std::to_string(i) + "/effort");
    }

    return config;
}

controller_interface::InterfaceConfiguration VelocityController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back("panda_joint" + std::to_string(i) + "/position");
    config.names.push_back("panda_joint" + std::to_string(i) + "/velocity");
  }

  return config;
}

controller_interface::CallbackReturn VelocityController::on_init()
{
  auto_declare<std::string>("urdf_param", "robot_description");
  auto_declare<std::string>("root_link", "panda_link0");
  auto_declare<std::string>("tip_link", "panda_link8");
  auto_declare<std::vector<std::string>>("joints", std::vector<std::string>{});
  auto_declare<std::vector<double>>("kp_cartesian_gains", std::vector<double>{});
  auto_declare<std::vector<double>>("kp_joint_gains", std::vector<double>{});
  auto_declare<std::vector<double>>("kd_joint_gains", std::vector<double>{});
  
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn VelocityController::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_node()->get_logger(), "Configuring VelocityController");
  auto node = get_node();

  urdf_param_ = node->get_parameter("urdf_param").as_string();
  root_link_ = node->get_parameter("root_link").as_string();
  tip_link_ = node->get_parameter("tip_link").as_string();
  joint_names_ = node->get_parameter("joints").as_string_array();

  std::string urdf_string;
  if (!node->get_parameter(urdf_param_, urdf_string)) {
      RCLCPP_ERROR(node->get_logger(), "Failed to get URDF from parameter '%s'", urdf_param_.c_str());
      return controller_interface::CallbackReturn::ERROR;
  }
  KDL::Tree tree;
  if (!kdl_parser::treeFromString(urdf_string, tree)) {
      RCLCPP_ERROR(node->get_logger(), "Failed to extract KDL tree from URDF");
      return controller_interface::CallbackReturn::ERROR;
  }
  if (!tree.getChain(root_link_, tip_link_, kdl_chain_)) {
      RCLCPP_ERROR(node->get_logger(), "Failed to extract KDL chain from '%s' to '%s'", root_link_.c_str(), tip_link_.c_str());
      return controller_interface::CallbackReturn::ERROR;
  }

  const auto n_joints = kdl_chain_.getNrOfJoints();
  if (n_joints == 0 || n_joints != joint_names_.size()) {
      RCLCPP_ERROR(node->get_logger(), "KDL chain joints mismatch with provided joint names.");
      return controller_interface::CallbackReturn::ERROR;
  }
    
  q_kdl_.resize(n_joints);
  qdot_kdl_.resize(n_joints);
  M_kdl_.resize(n_joints);
  C_kdl_.resize(n_joints);
  G_kdl_.resize(n_joints);

  fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(kdl_chain_);
  jac_solver_ = std::make_unique<KDL::ChainJntToJacSolver>(kdl_chain_);
  dyn_solver_ = std::make_unique<KDL::ChainDynParam>(kdl_chain_, KDL::Vector(0.0, 0.0, -9.81));

  auto kp_cartesian = node->get_parameter("kp_cartesian_gains").as_double_array();
  Kp_cartesian_.resize(6);
  for (size_t i = 0; i < 6; ++i) Kp_cartesian_(i) = kp_cartesian.at(i);

  auto kp_joint = node->get_parameter("kp_joint_gains").as_double_array();
  Kp_joint_.resize(n_joints);
  for (size_t i = 0; i < n_joints; ++i) Kp_joint_(i) = kp_joint.at(i);

  auto kd_joint = node->get_parameter("kd_joint_gains").as_double_array();
  Kd_joint_.resize(n_joints);
  for (size_t i = 0; i < n_joints; ++i) Kd_joint_(i) = kd_joint.at(i);

  goal_subscriber_ = node->create_subscription<geometry_msgs::msg::Point>(
      "/cartesian_goal", rclcpp::SystemDefaultsQoS(),
      [this](const geometry_msgs::msg::Point::SharedPtr msg) {
          KDL::Frame new_goal(KDL::Vector(msg->x, msg->y, msg->z));
          new_goal.M = ee_goal_.M; // Keep current orientation
          ee_goal_ = new_goal;
          goal_active_ = true;
          RCLCPP_INFO(get_node()->get_logger(), "Received new Cartesian goal: [x: %.2f, y: %.2f, z: %.2f]", msg->x, msg->y, msg->z);
      });

  pub_end_effector_ = node->create_publisher<geometry_msgs::msg::Point>("end_effector_position", 10);
    
  RCLCPP_INFO(node->get_logger(), "Successfully configured VelocityController with %u joints", n_joints);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn VelocityController::on_activate(const rclcpp_lifecycle::State &)
{
  for (size_t i = 0; i < joint_names_.size(); i++) {
    q_kdl_(i) = state_interfaces_[2 * i].get_value();
  }
  fk_solver_->JntToCart(q_kdl_, ee_goal_);
  goal_active_ = true;
  RCLCPP_INFO(get_node()->get_logger(), "Activated. Initial goal set to current EE position: [x: %.2f, y: %.2f, z: %.2f]",
              ee_goal_.p.x(), ee_goal_.p.y(), ee_goal_.p.z());
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn VelocityController::on_deactivate(const rclcpp_lifecycle::State &)
{
  for (auto & command_interface : command_interfaces_) {
    command_interface.set_value(0.0);
  }
  goal_active_ = false;
  RCLCPP_INFO(get_node()->get_logger(), "Deactivated.");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type VelocityController::update(const rclcpp::Time &, const rclcpp::Duration &)
{
  const size_t n_joints = joint_names_.size();
  for (size_t i = 0; i < n_joints; i++) {
    q_kdl_(i) = state_interfaces_[2 * i].get_value();
    qdot_kdl_(i) = state_interfaces_[2 * i + 1].get_value();
  }
  KDL::Frame x_current;
  fk_solver_->JntToCart(q_kdl_, x_current);

  Eigen::VectorXd tau_cmd = Eigen::VectorXd::Zero(n_joints);

  if (goal_active_) {
    KDL::Vector pos_error = ee_goal_.p - x_current.p;
    Eigen::VectorXd x_dot_desired(6);
    x_dot_desired << pos_error.x() * Kp_cartesian_(0), pos_error.y() * Kp_cartesian_(1), pos_error.z() * Kp_cartesian_(2), 0, 0, 0;

    KDL::Jacobian J_kdl(n_joints);
    jac_solver_->JntToJac(q_kdl_, J_kdl);
    Eigen::MatrixXd J_eigen = J_kdl.data;

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(J_eigen, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::VectorXd S = svd.singularValues();
    Eigen::VectorXd Sinv = S;
    for (int i = 0; i < S.size(); ++i) Sinv(i) = (S(i) > 1e-6) ? 1.0 / S(i) : 0.0;
    Eigen::MatrixXd J_pinv = svd.matrixV() * Sinv.asDiagonal() * svd.matrixU().transpose();
    Eigen::VectorXd q_dot_desired = J_pinv * x_dot_desired;

    // Joint Velocities -> Joint Torques

    dyn_solver_->JntToMass(q_kdl_, M_kdl_);
    dyn_solver_->JntToCoriolis(q_kdl_, qdot_kdl_, C_kdl_);
    dyn_solver_->JntToGravity(q_kdl_, G_kdl_);

    Eigen::VectorXd q_dot_error = q_dot_desired - qdot_kdl_.data;
    Eigen::VectorXd p_term = Kp_joint_.cwiseProduct(q_dot_error);
    Eigen::VectorXd d_term = Kd_joint_.cwiseProduct(qdot_kdl_.data);
    Eigen::VectorXd q_ddot_ref = p_term - d_term;
    tau_cmd = M_kdl_.data * q_ddot_ref + C_kdl_.data + G_kdl_.data;

    } else {
      dyn_solver_->JntToGravity(q_kdl_, G_kdl_);
      tau_cmd = G_kdl_.data;
    }
    
    for (size_t i = 0; i < n_joints; i++) {
        command_interfaces_[i].set_value(tau_cmd(i));
    }

    geometry_msgs::msg::Point p_msg;
    p_msg.x = x_current.p.x();
    p_msg.y = x_current.p.y();
    p_msg.z = x_current.p.z();
    pub_end_effector_->publish(p_msg);

    return controller_interface::return_type::OK;
}

}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(arm_controllers::VelocityController, controller_interface::ControllerInterface)