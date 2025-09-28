#pragma once

#include <memory>
#include <string>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>

#include <Eigen/Dense>
#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace arm_controllers {

class GravityPDController : public controller_interface::ControllerInterface {
public:
  GravityPDController();
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

private:
  // Core torque computation
  Eigen::VectorXd computeTorque(
    const Eigen::VectorXd & q_des,
    const Eigen::VectorXd & qdot_des,
    const Eigen::VectorXd & q_meas,
    const Eigen::VectorXd & qdot_meas);

  // URDF / KDL structures
  KDL::Chain chain_;
  std::unique_ptr<KDL::ChainDynParam> dyn_solver_;
  KDL::JntArray q_kdl_, qdot_kdl_, g_kdl_;
  KDL::Vector gravity_vec_;

  // Joint names
  std::vector<std::string> joint_names_;

  // Gains
  Eigen::VectorXd Kp_;
  Eigen::VectorXd Kd_;

  // Desired joint positions
  Eigen::VectorXd q_desired_;

  // Parameters
  std::string urdf_param_;
  std::string root_link_;
  std::string tip_link_;

  Eigen::VectorXd tau_;

  bool q_desired_initialized_ = false;



  //testing
  double angle_{0.0};
  bool direction_{true};
  rclcpp::Time start_time_;

  const int num_joints = 7;
  std::array<double, 7> initial_q_{0, 0, 0, 0, 0, 0, 0};
};

}