#include "arm_controllers/admittance_controller.hpp"

#include <cassert>
#include <cmath>
#include <exception>
#include <string>

#include <kdl_parser/kdl_parser.hpp>
#include "pluginlib/class_list_macros.hpp"

namespace arm_controllers {

controller_interface::InterfaceConfiguration
AdmittanceController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back("panda_joint" + std::to_string(i) + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration
AdmittanceController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back("panda_joint" + std::to_string(i) + "/position");
    config.names.push_back("panda_joint" + std::to_string(i) + "/velocity");
  }
  return config;
}

CallbackReturn AdmittanceController::on_init() {
  try {
    auto_declare<std::vector<double>>("M_t", {});
    auto_declare<std::vector<double>>("K_Dt", {});
    auto_declare<std::vector<double>>("K_Pt", {});

    auto_declare<std::string>("root_link", "");
    auto_declare<std::string>("tip_link", "");
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn AdmittanceController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  
  // 1. Load Gains
  auto m_t = get_node()->get_parameter("M_t").as_double_array();
  auto k_dt = get_node()->get_parameter("K_Dt").as_double_array(); // should be 2sqrt(Mt*Kpt)
  auto k_pt = get_node()->get_parameter("K_Pt").as_double_array();

  if (m_t.empty() || k_dt.empty() || k_pt.empty()) {
    RCLCPP_FATAL(get_node()->get_logger(), "Admittance gains (M_t, K_Dt, K_Pt) not set!");
    return CallbackReturn::FAILURE;
  }

  M_t_ = Eigen::Map<Vector6d>(m_t.data()).asDiagonal();
  K_Dt_ = Eigen::Map<Vector6d>(k_dt.data()).asDiagonal();
  K_Pt_ = Eigen::Map<Vector6d>(k_pt.data()).asDiagonal();
  
  M_t_inv_ = M_t_.inverse();

  root_name_ = get_node()->get_parameter("root_link").as_string();
  tip_name_ = get_node()->get_parameter("tip_link").as_string();

  auto parameters_client = std::make_shared<rclcpp::AsyncParametersClient>(get_node(), "/robot_state_publisher");
  parameters_client->wait_for_service();
  auto future = parameters_client->get_parameters({"robot_description"});
  auto result = future.get();
  if (!result.empty()) {
    robot_description_ = result[0].value_to_string();
  } else {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to get robot_description parameter.");
    return CallbackReturn::ERROR;
  }

  if (!kdl_parser::treeFromString(robot_description_, kdl_tree_)) {
    return CallbackReturn::ERROR;
  }
  if (!kdl_tree_.getChain(root_name_, tip_name_, kdl_chain_)) {
    return CallbackReturn::ERROR;
  }
  fk_solver_ = std::make_shared<KDL::ChainFkSolverPos_recursive>(kdl_chain_);
  ik_vel_solver_ = std::make_shared<KDL::ChainIkSolverVel_pinv>(kdl_chain_);
  ik_pos_solver_ = std::make_shared<KDL::ChainIkSolverPos_LMA>(kdl_chain_, 1e-5, 500, 1e-15);

  q_.resize(num_joints);
  dq_.resize(num_joints);
  q_kdl_.resize(num_joints);
  
  z_.setZero();
  z_dot_.setZero();
  z_ddot_.setZero();

  auto node = get_node();
  ft_sensor_sub_ = node->create_subscription<geometry_msgs::msg::Wrench>(
    "/sensor_joint/force_torque",
    rclcpp::SystemDefaultsQoS(),
    std::bind(&AdmittanceController::ft_sensor_callback, this, std::placeholders::_1)
  );

  return CallbackReturn::SUCCESS;
}

CallbackReturn AdmittanceController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  updateJointStates();
  initial_q_ = q_;
  elapsed_time_ = 0.0;
  first_update_ = true;
  for (int i = 0; i < num_joints; ++i) {
    q_kdl_(i) = q_(i);
    q_cmd_(i) = q_(i);
  }
  fk_solver_->JntToCart(q_kdl_, x_d_);
  RCLCPP_INFO(get_node()->get_logger(), "Admittance Controller Activated.");
  return CallbackReturn::SUCCESS;
}

void AdmittanceController::updateJointStates() {
  for (auto i = 0; i < num_joints; ++i) {
    const auto& position_interface = state_interfaces_.at(2 * i);
    const auto& velocity_interface = state_interfaces_.at(2 * i + 1);

    assert(position_interface.get_interface_name() == "position");
    assert(velocity_interface.get_interface_name() == "velocity");

    q_(i) = position_interface.get_value();
    dq_(i) = velocity_interface.get_value();
  }
}

void AdmittanceController::ft_sensor_callback(const geometry_msgs::msg::Wrench::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(ft_wrench_mutex_);
  ft_wrench_data_ = *msg;
}

controller_interface::return_type AdmittanceController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& period) {
  
  updateJointStates();
  if (first_update_) {
    initial_q_ = q_;

    z_.setZero();
    z_dot_.setZero();

    first_update_ = false;
  }
  double dt = period.seconds();
  elapsed_time_ = elapsed_time_ + period.seconds();

  Vector7d q_traj = initial_q_;
  double delta_angle = M_PI / 8.0 * (1 - std::cos(M_PI / 5 * elapsed_time_));
  double delta_angle_r = M_PI / 4.0 * (1 - std::sin(M_PI / 10.0 * elapsed_time_));
  q_traj(0) += delta_angle_r;
  q_traj(1) += delta_angle;
  q_traj(3) += delta_angle;
  
  // convert joint yrajectory to cartesian frame {d}
  KDL::JntArray q_traj_kdl(num_joints);
  for(int i=0; i<num_joints; ++i) {
    q_traj_kdl(i) = q_traj(i);
  }
  fk_solver_->JntToCart(q_traj_kdl, x_d_);

  std::lock_guard<std::mutex> lock(ft_wrench_mutex_);
  Eigen::Vector3d F_S, T_S;
  F_S << ft_wrench_data_.force.x, ft_wrench_data_.force.y, ft_wrench_data_.force.z;
  T_S << ft_wrench_data_.torque.x, ft_wrench_data_.torque.y, ft_wrench_data_.torque.z;
  
  // transform to end-effector frame
  Vector6d h_ed;
  h_ed.head<3>() = R_E_S_ * F_S;
  h_ed.tail<3>() = R_E_S_ * T_S;


  // equation: M_t * z_ddot + K_Dt * z_dot + K_Pt * z = h_ed
  // solve for z_ddot:
  // z_ddot = M_t_inv * (h_ed - K_Dt * z_dot - K_Pt * z)
  
  z_ddot_ = M_t_inv_ * (h_ed - K_Dt_ * z_dot_ - K_Pt_ * z_);
  
  // euler integration approximation small timestep
  // velocity z_dot is the old velocity plus acceleration times time, 
  // and the new position z is the old position plus velocity times time.
  z_dot_ += z_ddot_ * dt;
  z_ += z_dot_ * dt;
  
  x_t_ = x_d_;

  // o_t = o_d - z
  x_t_.p.x(x_d_.p.x() - z_(0));
  x_t_.p.y(x_d_.p.y() - z_(1));
  x_t_.p.z(x_d_.p.z() - z_(2));
  
  // compliant rotation matrix x_t.M is the original desired rotation 
  // x_d.M multiplied by the inverse of the rotation defined by the admittance offset z_rot
  KDL::Rotation rot_z = KDL::Rotation::RPY(z_(3), z_(4), z_(5));
  x_t_.M = x_d_.M * rot_z.Inverse();

  // find joint angles q_cmd that achieve pose x_t_
  // seed with current pose
  KDL::JntArray q_seed(num_joints);
  for (int i=0; i<num_joints; ++i)
  {
    q_seed(i) = q_(i);
  }
  
  KDL::JntArray q_out(num_joints);
  int ik_ret = ik_pos_solver_->CartToJnt(q_seed, x_t_, q_out); // solves q_output
  
  if (ik_ret < 0) {
    RCLCPP_WARN(get_node()->get_logger(), "IK Failed! Holding last position.");
  } else {
    for(int i=0; i<num_joints; ++i) {
      q_cmd_(i) = q_out(i);
    }
  }
  
  RCLCPP_INFO(get_node()->get_logger(), 
      "Z: %.4f, %.4f, %.4f | H_ed: %.2f, %.2f, %.2f", 
      z_(0), z_(1), z_(2), h_ed(0), h_ed(1), h_ed(2));

  RCLCPP_INFO(get_node()->get_logger(),
    "Force Input h_ed: Fx=%.2f, Fy=%.2f, Fz=%.2f | Tx=%.2f, Ty=%.2f, Tz=%.2f",
    h_ed(0), h_ed(1), h_ed(2), h_ed(3), h_ed(4), h_ed(5)
  );

  RCLCPP_INFO(get_node()->get_logger(),
    "Admittance Offset z: Px=%.4f, Py=%.4f, Pz=%.4f | Rx=%.4f, Ry=%.4f, Rz=%.4f",
    z_(0), z_(1), z_(2), z_(3), z_(4), z_(5)
  );

  KDL::Frame x_current;
  for (int i = 0; i < num_joints; ++i) {
    q_kdl_(i) = q_(i);
  }
  fk_solver_->JntToCart(q_kdl_, x_current);

  KDL::Twist tracking_error_twist = KDL::diff(x_current, x_t_);
  Vector6d tracking_error;
  tracking_error << tracking_error_twist.vel.x(), tracking_error_twist.vel.y(), tracking_error_twist.vel.z(),
                    tracking_error_twist.rot.x(), tracking_error_twist.rot.y(), tracking_error_twist.rot.z();

  RCLCPP_INFO(get_node()->get_logger(),
    "Tracking Error (x_t - x_e): Pos_Mag=%.4f (m) | Rot_Mag=%.4f (rad)",
    tracking_error.head<3>().norm(), tracking_error.tail<3>().norm()
  );

  for (int i = 0; i < num_joints; ++i) {
    command_interfaces_[i].set_value(q_cmd_(i));
  }

  return controller_interface::return_type::OK;
}

} // namespace arm_controllers

PLUGINLIB_EXPORT_CLASS(arm_controllers::AdmittanceController, 
    controller_interface::ControllerInterface)
