#include <arm_controllers/impedance_controller.hpp>

#include <cassert>
#include <cmath>
#include <exception>
#include <string>

#include <Eigen/Eigen>

namespace arm_controllers {

controller_interface::InterfaceConfiguration
ImpedanceController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back("panda_joint" + std::to_string(i) + "/effort");
  }

  // // Debug logging
  // RCLCPP_INFO(get_node()->get_logger(), "Configured JointImpedanceExampleController with %d joints", num_joints);
  // for (const auto& name : config.names) {
  //   RCLCPP_INFO(get_node()->get_logger(), "Interface name: %s", name.c_str());
  // }

  return config;
}

controller_interface::InterfaceConfiguration
ImpedanceController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back("panda_joint" + std::to_string(i) + "/position");
    config.names.push_back("panda_joint" + std::to_string(i) + "/velocity");
  }

  // // Debug logging
  // RCLCPP_INFO(get_node()->get_logger(), "Configured JointImpedanceExampleController with %d joints", num_joints);
  // for (const auto& name : config.names) {
  //   RCLCPP_INFO(get_node()->get_logger(), "Interface name: %s", name.c_str());
  // }

  return config;
}

controller_interface::return_type ImpedanceController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& period) {
  updateJointStates();
  Vector7d q_goal = initial_q_;
  elapsed_time_ = elapsed_time_ + period.seconds();

  // delta angle is calculated using a cosine function to create a smooth trajectory.
  // uncomment the three lines below to add a periodic movement to the goal position
  double delta_angle = M_PI / 8.0 * (1 - std::cos(M_PI / 5 * elapsed_time_));
  double delta_angle_r = M_PI / 4.0 * (1 - std::sin(M_PI / 10.0 * elapsed_time_));
  
  q_goal(0) += delta_angle_r;
  q_goal(1) += delta_angle;
  q_goal(3) += delta_angle;
  std::lock_guard<std::mutex> lock(ft_wrench_mutex_);
  Eigen::Vector3d F_S, T_S;

  F_S << ft_wrench_data_.force.x, 
         ft_wrench_data_.force.y, 
         ft_wrench_data_.force.z;

  T_S << ft_wrench_data_.torque.x,
         ft_wrench_data_.torque.y,
         ft_wrench_data_.torque.z;
  
  for (int i = 0; i < num_joints; ++i) {
    q_kdl_(i) = q_goal(i); // q_kdl_ now holds q_goal
  }
  if (fk_solver_->JntToCart(q_kdl_, x_d_) < 0) {
    RCLCPP_WARN(get_node()->get_logger(), "KDL FK failed on desired pose.");
  }

  Vector3d F_E = R_E_S_ * F_S;
  Vector3d T_E = R_E_S_ * T_S;
  Vector6d h_e_transformed;
  h_e_transformed.head<3>() = F_E;
  h_e_transformed.tail<3>() = T_E;

  dyn_param_->JntToMass(q_kdl_, M_kdl_);
  dyn_param_->JntToCoriolis(q_kdl_, q_dot_kdl_, C_kdl_);
  dyn_param_->JntToGravity(q_kdl_, G_kdl_);
  
  KDL::Frame x_current;
  for (int i = 0; i < num_joints; ++i) {
    q_kdl_(i) = q_(i);
  }

  fk_solver_->JntToCart(q_kdl_, x_current);
  jac_solver_->JntToJac(q_kdl_, J_kdl_);
  
  M_ = M_kdl_.data;
  h_ = C_kdl_.data + G_kdl_.data;
  J_ = J_kdl_.data;
  
  // task space error
  // pose error x_e = x_d - x
  KDL::Twist x_e_twist = KDL::diff(x_current, x_d_);
  Vector6d x_e;
  x_e << x_e_twist.vel.x(), x_e_twist.vel.y(), x_e_twist.vel.z(),
         x_e_twist.rot.x(), x_e_twist.rot.y(), x_e_twist.rot.z();
  
  // velocity error
  // assuming x_dot_d = 0
  Vector6d x_dot = J_ * dq_;
  Vector6d x_dot_e = -x_dot; 
  
  // reference acceleration, no dJ in KDL
  // assume x_ddot_d = 0 (holding pose)
  Vector6d x_ddot_ref = m_d_task_inv_ * (k_p_task_ * x_e + k_d_task_ * x_dot_e - h_e_transformed);
  
  // compute commanded joint acceleration
  // y = J_inv * (x_ddot_ref) 
  Vector6d b = x_ddot_ref; 
  Vector7d y = J_.completeOrthogonalDecomposition().solve(b);
  
  // compute u
  // u = M(q)*y + h(q, q_dot) + (J^T)(q)*h_e
  Vector7d tau_d_calculated = M_ * y + h_ + J_.transpose() * h_e_transformed;

  RCLCPP_INFO(
    get_node()->get_logger(),
    "Wrench h_e: Fx=%.3f, Fy=%.3f, Fz=%.3f | Tx=%.3f, Ty=%.3f, Tz=%.3f",
    h_e_transformed(0), h_e_transformed(1), h_e_transformed(2),
    h_e_transformed(3), h_e_transformed(4), h_e_transformed(5)
  );

  RCLCPP_INFO(
    get_node()->get_logger(),
    "Pose Error x_e: Pos_Mag=%.4f (m) | Rot_Mag=%.4f (rad)",
    x_e.head<3>().norm(), x_e.tail<3>().norm()
  );

  RCLCPP_INFO(
    get_node()->get_logger(),
    "Vel Error x_dot_e: Mag=%.4f (m/s)",
    x_dot_e.norm()
  );

  for (int i = 0; i < num_joints; ++i) {
    command_interfaces_[i].set_value(tau_d_calculated(i));
  }
  
  return controller_interface::return_type::OK;
}


CallbackReturn ImpedanceController::on_init() {
  try {
    auto_declare<std::vector<double>>("k_p_task", {});
    auto_declare<std::vector<double>>("k_d_task", {});
    auto_declare<std::vector<double>>("m_d_task", {});
    
    auto_declare<std::string>("root_link", "");
    auto_declare<std::string>("tip_link", "");

  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn ImpedanceController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  auto k_p = get_node()->get_parameter("k_p_task").as_double_array();
  auto k_d = get_node()->get_parameter("k_d_task").as_double_array();
  auto m_d = get_node()->get_parameter("m_d_task").as_double_array();
  k_p_task_ = Eigen::Map<Vector6d>(k_p.data()).asDiagonal();
  k_d_task_ = Eigen::Map<Vector6d>(k_d.data()).asDiagonal();
  m_d_task_inv_ = Eigen::Map<Vector6d>(m_d.data()).asDiagonal().inverse();

  root_name_ = get_node()->get_parameter("root_link").as_string();
  tip_name_ = get_node()->get_parameter("tip_link").as_string();

  if (k_p.empty()) {
    RCLCPP_FATAL(get_node()->get_logger(), "k_gains parameter not set");
    return CallbackReturn::FAILURE;
  }

  if (k_d.empty()) {
    RCLCPP_FATAL(get_node()->get_logger(), "d_gains parameter not set");
    return CallbackReturn::FAILURE;
  }

  auto parameters_client =
      std::make_shared<rclcpp::AsyncParametersClient>(get_node(), "/robot_state_publisher");
  parameters_client->wait_for_service();

  auto future = parameters_client->get_parameters({"robot_description"});
  auto result = future.get();
  if (!result.empty()) {
    robot_description_ = result[0].value_to_string();
  } else {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to get robot_description parameter.");
  }

  if (!kdl_parser::treeFromString(robot_description_, kdl_tree_)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to build KDL tree from URDF");
    return CallbackReturn::ERROR;
  }
  
  if (!kdl_tree_.getChain(root_name_, tip_name_, kdl_chain_)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to get KDL chain");
    return CallbackReturn::ERROR;
  }
  fk_solver_ = std::make_shared<KDL::ChainFkSolverPos_recursive>(kdl_chain_);
  jac_solver_ = std::make_shared<KDL::ChainJntToJacSolver>(kdl_chain_);
  
  KDL::Vector gravity_v(0.0, 0.0, -9.81); 
  dyn_param_ = std::make_shared<KDL::ChainDynParam>(kdl_chain_, gravity_v);

  q_kdl_.resize(num_joints);
  q_dot_kdl_.resize(num_joints);
  M_kdl_.resize(num_joints);
  C_kdl_.resize(num_joints);
  G_kdl_.resize(num_joints);
  J_kdl_.resize(num_joints);

  auto node = get_node(); 
    
  ft_sensor_sub_ = node->create_subscription<geometry_msgs::msg::Wrench>(
    "/sensor_joint/force_torque",
    rclcpp::SystemDefaultsQoS(),
    std::bind(&ImpedanceController::ft_sensor_callback, this, std::placeholders::_1)
  );

  return CallbackReturn::SUCCESS;
}

CallbackReturn ImpedanceController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  updateJointStates();
  initial_q_ = q_;
  elapsed_time_ = 0.0;
  for (int i = 0; i < num_joints; ++i) {
    q_kdl_(i) = q_(i);
  }
  if (fk_solver_->JntToCart(q_kdl_, x_d_) < 0) {
    RCLCPP_ERROR(get_node()->get_logger(), "KDL FK failed on activate");
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(get_node()->get_logger(), "Activated controller. Holding target pose.");
  return CallbackReturn::SUCCESS;
}

void ImpedanceController::updateJointStates() {
  for (auto i = 0; i < num_joints; ++i) {
    const auto& position_interface = state_interfaces_.at(2 * i);
    const auto& velocity_interface = state_interfaces_.at(2 * i + 1);

    assert(position_interface.get_interface_name() == "position");
    assert(velocity_interface.get_interface_name() == "velocity");

    q_(i) = position_interface.get_value();
    dq_(i) = velocity_interface.get_value();
  }
}
void ImpedanceController::ft_sensor_callback(const geometry_msgs::msg::Wrench::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(ft_wrench_mutex_);
    ft_wrench_data_ = *msg;
}

}  // namespace arm_controllers
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(arm_controllers::ImpedanceController,
                       controller_interface::ControllerInterface)
