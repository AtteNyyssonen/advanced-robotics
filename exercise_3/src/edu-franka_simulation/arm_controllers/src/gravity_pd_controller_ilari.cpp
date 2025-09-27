#include "arm_controllers/gravity_pd_controller_ilari.hpp"

#include <pluginlib/class_list_macros.hpp>

namespace pd_gravity_controller
{

PDGravityController::PDGravityController()
: gravity_vec_(0.0, 0.0, -9.81)
{
}

controller_interface::CallbackReturn PDGravityController::on_init()
{
  auto node = get_node();

  node->declare_parameter<std::vector<std::string>>("joints", std::vector<std::string>{});
  node->declare_parameter<std::string>("urdf_param", "robot_description");
  node->declare_parameter<std::string>("root_link", "panda_link0");
  node->declare_parameter<std::string>("tip_link", "panda_link7");
  node->declare_parameter<double>("kp", 50.0);
  node->declare_parameter<double>("kd", 5.0);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn PDGravityController::on_configure(
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

  dyn_solver_ = std::make_unique<KDL::ChainDynParam>(chain_, gravity_vec_);

  q_kdl_.resize(chain_.getNrOfJoints());
  qdot_kdl_.resize(chain_.getNrOfJoints());
  g_kdl_.resize(chain_.getNrOfJoints());

  // Gains
  double kp = node->get_parameter("kp").as_double();
  double kd = node->get_parameter("kd").as_double();
  Kp_ = Eigen::VectorXd::Constant(chain_.getNrOfJoints(), kp);
  Kd_ = Eigen::VectorXd::Constant(chain_.getNrOfJoints(), kd);

  // Desired q (initialize to zero)
  q_desired_ = Eigen::VectorXd::Zero(chain_.getNrOfJoints());

  RCLCPP_INFO(node->get_logger(),
            "Configured PD+Gravity controller with %zu joints",
            static_cast<size_t>(chain_.getNrOfJoints()));

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn PDGravityController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn PDGravityController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration PDGravityController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint_name : joint_names_) {
    config.names.push_back(joint_name + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration PDGravityController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint_name : joint_names_) {
    config.names.push_back(joint_name + "/position");
    config.names.push_back(joint_name + "/velocity");
  }
  return config;
}

controller_interface::return_type PDGravityController::update(
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
    q_desired_ = Eigen::VectorXd(joint_names_.size());
    q_desired_ << 0.1, -0.5, 0.3, -1.2, 0.8, 1.0, -0.6; // specific joint positions in radians
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

}  // namespace pd_gravity_controller

PLUGINLIB_EXPORT_CLASS(
  pd_gravity_controller::PDGravityController,
  controller_interface::ControllerInterface)
