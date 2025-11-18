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



#pragma once

#include <string>

#include <Eigen/Eigen>
#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>

#include "geometry_msgs/msg/wrench.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include <rclcpp/subscription.hpp>
#include <rclcpp/time.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace arm_controllers {

/**
 * The joint impedance example controller moves joint 4 and 5 in a very compliant periodic movement.
 */
class ImpedanceController : public controller_interface::ControllerInterface {
 public:
  using Vector7d = Eigen::Matrix<double, 7, 1>;
  [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration()
      const override;
  [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration()
      const override;
  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

 private:
  std::string robot_description_;
  const int num_joints = 7;
  Vector7d q_;
  Vector7d initial_q_;
  Vector7d dq_;
  Vector7d dq_filtered_;
  Vector7d k_gains_;
  Vector7d d_gains_;
  double elapsed_time_{0.0};
  void updateJointStates();
  KDL::Tree kdl_tree_;
  KDL::Chain kdl_chain_;
  std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
  std::shared_ptr<KDL::ChainJntToJacSolver> jac_solver_;
  std::shared_ptr<KDL::ChainDynParam> dyn_param_;

  std::string root_name_, tip_name_;

  // KDL data holders
  KDL::JntArray q_kdl_, q_dot_kdl_;
  KDL::JntArray G_kdl_, C_kdl_; // Gravity, Coriolis
  KDL::JntSpaceInertiaMatrix M_kdl_; // Mass Matrix
  KDL::Jacobian J_kdl_;

  using Matrix7d = Eigen::Matrix<double, 7, 7>;
  using Vector6d = Eigen::Matrix<double, 6, 1>;
  using Matrix6d = Eigen::Matrix<double, 6, 6>;
  using Matrix6x7d = Eigen::Matrix<double, 6, 7>;
  using Vector3d = Eigen::Matrix<double, 3, 1>;
  using Matrix3d = Eigen::Matrix<double, 3, 3>;
  Matrix6d k_p_task_, k_d_task_, m_d_task_inv_;
  Matrix6x7d J_;
  Matrix7d M_;
  Vector7d h_;
  Vector6d h_e_ = Vector6d::Zero();
  const Matrix3d R_E_S_ = (Matrix3d() << 
    1.0, 0.0, 0.0,
    0.0, 0.0, 1.0,
    0.0, -1.0, 0.0).finished();
  KDL::Frame x_d_;
  
  rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr ft_sensor_sub_;
  std::mutex ft_wrench_mutex_;
  geometry_msgs::msg::Wrench ft_wrench_data_;
  void ft_sensor_callback(const geometry_msgs::msg::Wrench::SharedPtr msg);
};

}  // namespace arm_controllers
