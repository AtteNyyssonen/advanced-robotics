#pragma once

#include <rclcpp/rclcpp.hpp>
#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/frames.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <vector>
#include <string>
#include <memory>

namespace arm_controllers
{

class KinematicController : public controller_interface::ControllerInterface
{
public:
  KinematicController();
  controller_interface::CallbackReturn on_init() override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::vector<std::string> joints_;
  std::vector<double> kp_gains_;
  std::string urdf_param_;
  std::string root_link_;
  std::string tip_link_;

  KDL::Tree kdl_tree_;
  KDL::Chain kdl_chain_;
  KDL::JntArray kdl_q_;
  KDL::JntArray kdl_qdot_;

  std::vector<hardware_interface::LoanedCommandInterface> velocity_command_handles_;
  std::vector<hardware_interface::LoanedStateInterface> position_state_handles_;
  std::vector<hardware_interface::LoanedStateInterface> velocity_state_handles_;

  double elapsed_time_;
  double traj_duration_;
  bool goal_active_;
  KDL::Frame ee_start_;
  KDL::Frame ee_goal_;

  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr ee_goal_subscriber_;

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_pos_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_vel_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_desired_vel_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_error_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_control_out_;

  bool init_kdl_from_urdf(const std::string & urdf_param);
};

}
