#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <Eigen/Dense>

#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jacobian.hpp>
#include <kdl_parser/kdl_parser.hpp>

#define num_joints 7

namespace arm_controllers
{
class VelocityController : public controller_interface::ControllerInterface
{
public:
  [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration()
      const override;
  [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration()
      const override;
  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  std::vector<std::string> joint_names_;
  std::string urdf_param_;
  std::string root_link_;
  std::string tip_link_;

  KDL::Chain chain_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
  std::unique_ptr<KDL::ChainJntToJacSolver> jac_solver_;
  std::unique_ptr<KDL::ChainDynParam> dyn_solver_;
  KDL::JntArray q_kdl_;
  KDL::JntArray qdot_kdl_;
  KDL::Jacobian J_kdl_;
  KDL::JntSpaceInertiaMatrix M_kdl_;
  KDL::JntArray C_kdl_; // Coriolis & Centrifugal
  KDL::JntArray G_kdl_; // Gravity

  Eigen::VectorXd Kd_cartesian_;
  Eigen::VectorXd Kp_cartesian_;
  Eigen::VectorXd Kd_joint_;
  
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr goal_subscriber_;
  KDL::Frame ee_goal_;
  bool goal_active_{false};
  double goal_tolerance_;
  double max_cartesian_velocity_;

  // plotjuggler
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pub_end_effector_;

};

}