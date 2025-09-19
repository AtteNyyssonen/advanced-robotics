// Copyright (c) 2025 Tampere University, Autonomous Mobile Machines
//
// Licensed under the MIT License.
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <arm_controllers/first_movement_controller.hpp>

#include <cassert>
#include <cmath>
#include <exception>
#include <string>

#include <Eigen/Eigen>

namespace arm_controllers {

controller_interface::InterfaceConfiguration
FirstMovementController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back("panda_joint" + std::to_string(i) + "/position");
  }
  return config;
}

controller_interface::InterfaceConfiguration
FirstMovementController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  if (!is_gazebo_) {
    for (int i = 1; i <= num_joints; ++i) {
      config.names.push_back("panda_joint" + std::to_string(i) + "/position");
    }
  } else {
    for (int i = 1; i <= num_joints; ++i) {
      config.names.push_back("panda_joint" + std::to_string(i) + "/position");
    }
  }
  return config;
}


controller_interface::return_type FirstMovementController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/) 
{
  if (initialization_flag_) {
    for (int i = 0; i < num_joints; ++i) {
      initial_q_.at(i) = state_interfaces_[i].get_value();
    }
    initialization_flag_ = false;

    angle_ = 0.0;           // start at zero
    direction_ = true; // start moving toward +2.8
  }

  const double step = 0.001;      // radians per update
  const double upper_limit = 2.8;
  const double lower_limit = -2.8;

  // Move angle in the current direction
  if (direction_) {
    angle_ += step;
    if (angle_ >= upper_limit) {
      angle_ = upper_limit;
      direction_ = false;
    }
  } else {
    angle_ -= step;
    if (angle_ <= lower_limit) {
      angle_ = lower_limit;
      direction_ = true;
    }
  }

  //std::cout << "Current : " << angle_ << std::endl;

  // Desired position relative to initial
  double q_desired = initial_q_[0] + angle_;
  command_interfaces_[0].set_value(q_desired);

  return controller_interface::return_type::OK;
}

CallbackReturn FirstMovementController::on_init() {
  try {
    auto_declare<bool>("gazebo", false);
    auto_declare<std::string>("robot_description", "");
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn FirstMovementController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  is_gazebo_ = get_node()->get_parameter("gazebo").as_bool();

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

  return CallbackReturn::SUCCESS;
}

CallbackReturn FirstMovementController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  initialization_flag_ = true;
  elapsed_time_ = 0.0;
  return CallbackReturn::SUCCESS;
}

}  // namespace arm_controllers
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(arm_controllers::FirstMovementController,
                       controller_interface::ControllerInterface)
