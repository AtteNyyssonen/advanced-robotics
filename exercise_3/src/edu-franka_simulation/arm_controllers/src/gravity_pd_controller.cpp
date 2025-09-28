#include "arm_controllers/gravity_pd_controller.hpp"

#include <pluginlib/class_list_macros.hpp>

namespace arm_controllers
{

GravityPDController::GravityPDController()
: gravity_vec_(0.0, 0.0, -9.81)
{
}

controller_interface::CallbackReturn GravityPDController::on_init()
{
  auto node = get_node();

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn GravityPDController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto node = get_node();

  joint_names_ = node->get_parameter("joints").as_string_array();
  urdf_param_ = node->get_parameter("urdf_param").as_string();
  root_link_  = node->get_parameter("root_link").as_string();
  tip_link_   = node->get_parameter("tip_link").as_string();

  // Build KDL chain from URDF
  urdf::Model urdf_model;
  std::string urdf_string;

  if (!node->get_parameter(urdf_param_, urdf_string)) {
    RCLCPP_ERROR(node->get_logger(),
                "Failed to get URDF from parameter [%s]", urdf_param_.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }

  if (!urdf_model.initString(urdf_string)) {
    RCLCPP_ERROR(node->get_logger(),
                "Failed to parse URDF from string in parameter [%s]", urdf_param_.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }

  KDL::Tree tree;
  if (!kdl_parser::treeFromUrdfModel(urdf_model, tree)) {
    RCLCPP_ERROR(node->get_logger(), "Failed to extract KDL tree from URDF");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (!tree.getChain(root_link_, tip_link_, chain_)) {
    RCLCPP_ERROR(node->get_logger(), "Failed to extract KDL chain from %s to %s",
                 root_link_.c_str(), tip_link_.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }

  const auto kdl_n = chain_.getNrOfJoints();
  // Validate that joint names were provided and match KDL chain size
  if (joint_names_.empty()) {
    RCLCPP_ERROR(node->get_logger(), "Parameter 'joints' is empty. Set joint names in controller parameters.");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (static_cast<size_t>(kdl_n) != joint_names_.size()) {
    RCLCPP_ERROR(node->get_logger(),
      "Mismatch between KDL chain joints (%d) and provided joint names (%zu).",
      kdl_n, joint_names_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  // KDL arrays already resized above:
  q_kdl_.resize(kdl_n);
  qdot_kdl_.resize(kdl_n);
  g_kdl_.resize(kdl_n);

  // Gains
  double kp = node->get_parameter("kp").as_double();
  double kd = node->get_parameter("kd").as_double();
  Kp_ = Eigen::VectorXd::Constant(kdl_n, kp);
  Kd_ = Eigen::VectorXd::Constant(kdl_n, kd);

  // Desired q (initialize to zero)
  q_desired_ = Eigen::VectorXd::Zero(kdl_n);

  dyn_solver_ = std::make_unique<KDL::ChainDynParam>(chain_, gravity_vec_);


  RCLCPP_INFO(node->get_logger(),
            "Configured PD+Gravity controller with %zu joints",
            static_cast<size_t>(chain_.getNrOfJoints()));

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn GravityPDController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn GravityPDController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration GravityPDController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint_name : joint_names_) {
    config.names.push_back(joint_name + "/effort");
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

controller_interface::return_type GravityPDController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  (void)time;  // quiet warning of unused variable
  (void)period; // quiet warning of unused variable

  // Read current joint states
  for (size_t i = 0; i < joint_names_.size(); i++) {
    q_kdl_(i) = state_interfaces_[2*i].get_value();       // position
    qdot_kdl_(i) = state_interfaces_[2*i + 1].get_value(); // velocity
  }

  // Initialize desired joint positions once
   if (!q_desired_initialized_) {
    const size_t nj = joint_names_.size();
    if (nj == 0) {
      RCLCPP_ERROR(get_node()->get_logger(), "No joint names available when initializing q_desired_. Aborting update.");
      return controller_interface::return_type::ERROR;
    }
    q_desired_.resize(nj);
    // assign element-wise (safe regardless of vector size)
    if (nj == 7) {
      q_desired_(0) = 0.1;
      q_desired_(1) = -0.5;
      q_desired_(2) = 0.3;
      q_desired_(3) = -1.2;
      q_desired_(4) = 0.8;
      q_desired_(5) = 1.0;
      q_desired_(6) = -0.6;
    } else {
      for (size_t i = 0; i < nj; ++i) q_desired_(i) = 0.0;
      RCLCPP_WARN(get_node()->get_logger(), "q_desired_ initialized to zeros because joint count != 7 (%zu)", nj);
    }
    q_desired_initialized_ = true;
    RCLCPP_INFO(get_node()->get_logger(), "q_desired initialized to hardcoded pose");
  }

  // Compute PD term: tau_PD = Kp*(q_des - q) - Kd*q_dot
  Eigen::VectorXd q_eigen = Eigen::Map<Eigen::VectorXd>(q_kdl_.data.data(), q_kdl_.rows());
  Eigen::VectorXd qdot_eigen = Eigen::Map<Eigen::VectorXd>(qdot_kdl_.data.data(), qdot_kdl_.rows());

  Eigen::VectorXd pd_term = Kp_.cwiseProduct(q_desired_ - q_eigen) - Kd_.cwiseProduct(qdot_eigen);

  // Compute gravity compensation
  g_kdl_.data.setZero();
  int ret = dyn_solver_->JntToGravity(q_kdl_, g_kdl_);
  if (ret < 0) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to compute gravity torques.");
    return controller_interface::return_type::ERROR;
  }
  Eigen::VectorXd gravity_torques = Eigen::Map<Eigen::VectorXd>(g_kdl_.data.data(), g_kdl_.rows());

  // Final torque command
  tau_ = pd_term + gravity_torques;

  // Send torque commands to each joint
  for (size_t i = 0; i < command_interfaces_.size(); i++) {
    command_interfaces_[i].set_value(tau_(i));
  }


  return controller_interface::return_type::OK;
}

}

PLUGINLIB_EXPORT_CLASS(
  arm_controllers::GravityPDController,
  controller_interface::ControllerInterface)
