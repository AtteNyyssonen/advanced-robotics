#pragma once

#include <string>
#include <Eigen/Eigen>
#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <pluginlib/class_list_macros.hpp>
#include "std_msgs/msg/float64_multi_array.hpp"
#include <urdf/model.h>
#include <kdl/kdl.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <memory>


#define PI 3.141592
#define D2R PI / 180.0
#define R2D 180.0 / PI
#define num_joints 7
#define SaveDataMax 57

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace arm_controllers {

class GravityPDController : public controller_interface::ControllerInterface {
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
  Vector7d d_gains_;

  std::vector<std::string> joint_names_;
  std::vector<urdf::JointConstSharedPtr> joint_urdfs_; 

  KDL::Tree kdl_tree_;
  KDL::Chain kdl_chain_;

  KDL::JntArray G_;
  KDL::Vector gravity_;


  std::unique_ptr<KDL::ChainDynParam> id_solver_;
  Eigen::VectorXd tau_;
  KDL::JntArray qd_;
  KDL::JntArray q_, qdot_;
  KDL::JntArray e_, e_dot_, e_int_;

  KDL::JntArray Kp_, Kd_;

  double t;

  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_qd_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_q_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_e_;

  std_msgs::msg::Float64MultiArray msg_qd_;
  std_msgs::msg::Float64MultiArray msg_q_;  
  std_msgs::msg::Float64MultiArray msg_e_;
};

}
