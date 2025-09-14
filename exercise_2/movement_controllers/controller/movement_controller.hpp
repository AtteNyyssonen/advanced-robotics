#pragma once

#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <std_msgs/msg/string.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>

#include <memory>
#include <string>
#include <vector>

namespace arm_controllers
{

class MovementController : public controller_interface::ControllerInterface
{
public:
  MovementController();

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State &previous_state) override;
  controller_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State &previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State &previous_state) override;

  // Required pure virtual overrides
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update(
      const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  void command_callback(const std_msgs::msg::String::SharedPtr msg);

  std::string active_controller_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_client_;
};

}