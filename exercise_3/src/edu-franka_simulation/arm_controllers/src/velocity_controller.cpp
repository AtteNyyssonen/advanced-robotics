#include "arm_controllers/velocity_controller.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <urdf/model.h>

namespace arm_controllers
{

controller_interface::InterfaceConfiguration 
VelocityController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    // Add the effort control interfaces for the joints
    for (int i = 1; i <= num_joints; ++i) {
      config.names.push_back("panda_joint" + std::to_string(i) + "/effort");
    }

    return config;
  }

controller_interface::InterfaceConfiguration
VelocityController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  
  // Add the position and velocity state interfaces for the joints
  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back("panda_joint" + std::to_string(i) + "/position");
    config.names.push_back("panda_joint" + std::to_string(i) + "/velocity");
  }

  return config;
}

controller_interface::CallbackReturn VelocityController::on_init()
{
  auto_declare<std::vector<std::string>>("joints", std::vector<std::string>{});
  auto_declare<std::string>("urdf_param", "robot_description");
  auto_declare<std::string>("root_link", "");
  auto_declare<std::string>("tip_link", "");
  auto_declare<std::vector<double>>("kp_cartesian_gains", std::vector<double>{});
  auto_declare<std::vector<double>>("kd_cartesian_gains", std::vector<double>{});
  auto_declare<std::vector<double>>("kd_joint_gains", std::vector<double>{});

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn VelocityController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto node = get_node();
  urdf_param_ = node->get_parameter("urdf_param").as_string();
  root_link_  = node->get_parameter("root_link").as_string();
  tip_link_   = node->get_parameter("tip_link").as_string();
  joint_names_ = node->get_parameter("joints").as_string_array(); 
  
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
  KDL::Vector gravity_vec_ = KDL::Vector(0.0, 0.0, -9.81);
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


  auto kd_joint_gains = node->get_parameter("kd_joint_gains").as_double_array();
  if (kd_joint_gains.size() != 7) {
    RCLCPP_ERROR(node->get_logger(), "Parameter 'kd_joint_gains' must have 7 values.");
    return controller_interface::CallbackReturn::ERROR;
  }
  Kd_joint_.resize(7);
  for (size_t i = 0; i < 6; ++i) Kd_joint_(i) = kd_joint_gains[i];


  goal_subscriber_ = node->create_subscription<geometry_msgs::msg::Point>(
    "/cartesian_goal", rclcpp::SystemDefaultsQoS(),
    [this](const geometry_msgs::msg::Point::SharedPtr msg) {
      KDL::Frame new_goal(KDL::Vector(msg->x, msg->y, msg->z));
      new_goal.M = ee_goal_.M; 
      ee_goal_ = new_goal;
      goal_active_ = true;
      RCLCPP_INFO(get_node()->get_logger(), "Received new Cartesian goal: [x: %.2f, y: %.2f, z: %.2f]",
                  msg->x, msg->y, msg->z);
    });

  // plotjuggler
  pub_end_effector_ = get_node()->create_publisher<geometry_msgs::msg::Point>("end_effector_position", 10);


  RCLCPP_INFO(node->get_logger(), "Configured velocity controller with %d joints", kdl_n);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn VelocityController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  const size_t n_joints = joint_names_.size();
  for (size_t i = 0; i < n_joints; i++) {
    q_kdl_(i) = state_interfaces_[2 * i].get_value();
    qdot_kdl_(i) = 0.0; 
  }

  KDL::Frame x_current;
  fk_solver_->JntToCart(q_kdl_, x_current);

  ee_goal_ = x_current;
  goal_active_ = true;
  
  RCLCPP_INFO(get_node()->get_logger(), "Activated. Initial goal set to current EE position: [x: %.2f, y: %.2f, z: %.2f]",
      ee_goal_.p.x(), ee_goal_.p.y(), ee_goal_.p.z());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn VelocityController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (auto & command_interface : command_interfaces_) {
    command_interface.set_value(0.0);
  }
  goal_active_ = false;
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type VelocityController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  const size_t n_joints = joint_names_.size();

  for (size_t i = 0; i < n_joints; i++) {
    q_kdl_(i) = state_interfaces_[2 * i].get_value();       // position
    qdot_kdl_(i) = state_interfaces_[2 * i + 1].get_value(); // velocity
  }

  Eigen::VectorXd tau_cmd = Eigen::VectorXd::Zero(n_joints);
  KDL::Frame x_current;

  fk_solver_->JntToCart(q_kdl_, x_current);

  dyn_solver_->JntToGravity(q_kdl_, G_kdl_);
  Eigen::VectorXd tau_gravity = G_kdl_.data;

  if (goal_active_) {
    // Forward Kinematics to get current end-effector pose
    KDL::Vector pos_error = ee_goal_.p - x_current.p;
    KDL::Rotation R_err = ee_goal_.M * x_current.M.Inverse();
    KDL::Vector rot_error_kdl = R_err.GetRot(); 
    if (pos_error.Norm() <= 0.01 && rot_error_kdl.Norm() <= 0.01) {
      RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000, "Goal reached. Holding pose.");
      ee_goal_.M = x_current.M;
    }
    Eigen::VectorXd x_error(6);
    x_error << pos_error.x(), pos_error.y(), pos_error.z(), 
                   rot_error_kdl.x(), rot_error_kdl.y(), rot_error_kdl.z();
    KDL::JntArrayVel q_kdl_vel(q_kdl_, qdot_kdl_);
    KDL::Jacobian J_kdl(chain_.getNrOfJoints());

    jac_solver_->JntToJac(q_kdl_, J_kdl_);
    Eigen::MatrixXd J = J_kdl.data;
    Eigen::VectorXd qdot = qdot_kdl_.data;

                  
    // x_dot_cmd = Kp * x_error                   
    Eigen::VectorXd x_dot_cmd = Kp_cartesian_.cwiseProduct(x_error);

    Eigen::MatrixXd J_pseudo_inverse = J.completeOrthogonalDecomposition().pseudoInverse();

    // From task to joint
    Eigen::VectorXd q_dot_cmd = J_pseudo_inverse * x_dot_cmd;

    // Velocity control equation
    Eigen::VectorXd tau_v = Kd_joint_.cwiseProduct(q_dot_cmd - qdot);             

    tau_cmd = tau_gravity + tau_v;
  }

  for (size_t i = 0; i < n_joints; i++) {
    command_interfaces_[i].set_value(tau_cmd(i));
  }


  // Publish end-effector position to plotjuggler
  geometry_msgs::msg::Point p_msg;
  p_msg.x = x_current.p.x();
  p_msg.y = x_current.p.y();
  p_msg.z = x_current.p.z();
  pub_end_effector_->publish(p_msg);

  return controller_interface::return_type::OK;
}

}

PLUGINLIB_EXPORT_CLASS(arm_controllers::VelocityController,
  controller_interface::ControllerInterface)
