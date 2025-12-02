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

// Basic ROS2 headers
#include <Eigen/Eigen>
#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

// Real-time buffer (real-time utilities)
#include <realtime_tools/realtime_buffer.hpp>

// Plugin system
#include <pluginlib/class_list_macros.hpp>

// Messages
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <tf2_ros/transform_broadcaster.h>



// URDF model parsing
#include <urdf/model.h>

// KDL libraries
#include <kdl/kdl.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_lma.hpp> 

// Memory management (use standard C++ smart pointers)
#include <memory>


#define PI 3.141592
#define D2R PI / 180.0
#define R2D 180.0 / PI
#define num_joints 7
#define SaveDataMax 57

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace arm_controllers {

class ComputedTorqueController : public controller_interface::ControllerInterface {
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
  Vector7d position_interface_values_;
  Vector7d velocity_interface_values_;
  Vector7d p_gains_;
  Vector7d i_gains_;
  Vector7d d_gains_;
  void updateJointStates();
  void computeJointLimitRepulsion(Eigen::VectorXd);
  void computeEEWallRepulsion(Eigen::VectorXd);
  // joint handles
  std::vector<std::string> joint_names_;  // joint names
  // std::vector<hardware_interface::JointHandle> joints_;  // joint handles
  std::vector<urdf::JointConstSharedPtr> joint_urdfs_;  // joint urdfs

  // KDL
  KDL::Tree kdl_tree_;
  KDL::Chain kdl_chain_;

  // kdl variables
  KDL::JntSpaceInertiaMatrix M_; // inertia matrix
  KDL::JntArray C_; // coriolis and centrifugal forces
  KDL::JntArray G_; // gravity forces
  KDL::Vector gravity_;

  // kdl solver (solver to compute the inverse dynamics)
  std::unique_ptr<KDL::ChainDynParam> id_solver_;
  std::unique_ptr<KDL::ChainJntToJacSolver> jac_solver_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
  std::shared_ptr<KDL::ChainIkSolverPos_LMA> ik_pos_solver_;

  //Joint space state
  KDL::JntArray qd_, qd_dot_, qd_ddot_;
  KDL::JntArray q_, qdot_;
  KDL::JntArray e_, e_dot_, e_int_;

  // input
  KDL::JntArray aux_d_;
  KDL::JntArray comp_d_;
  KDL::JntArray tau_d_;
  std::vector<double> joint_Qstar_; // influence distances for each joint (radians)
  double joint_k_;                  // single gain for joint limits (could be per-joint)

  KDL::Jacobian J_kdl_;
  Eigen::MatrixXd J_eigen_;
  Eigen::VectorXd tau_rep_joint_limits_; // repulsive torques from joint limits
  Eigen::VectorXd tau_rep_ee_;           // repulsive torques from ee obstacle
  
  // gains
  KDL::JntArray Kp_, Ki_, Kd_;

  Eigen::Vector3d wall_point_;  // p0 from sdf e.g. (1.3, 0, 0)
  Eigen::Vector3d wall_normal_; // e.g. (1,0,0)
  double wall_Qstar_;           // influence distance for wall (meters)
  double wall_k_;               // gain for wall repulsive potential


  // Other member variables...
  double t;
  double SaveData_[SaveDataMax];

  // goal
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr goal_subscriber_;
  KDL::Frame ee_goal_;

  // Publishers
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_qd_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_q_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_e_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_SaveData_;
  

  // messages
  std_msgs::msg::Float64MultiArray msg_qd_;
  std_msgs::msg::Float64MultiArray msg_q_;  
  std_msgs::msg::Float64MultiArray msg_e_;
  std_msgs::msg::Float64MultiArray msg_SaveData_;
  std::string root_link_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  bool goal_switch_ = false;

};

}  // namespace arm_controllers
