#include <memory>
#include <string>
#include <vector>
#include <mutex>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "geometry_msgs/msg/wrench.hpp"

#include <Eigen/Eigen>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_lma.hpp>

namespace arm_controllers {

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class AdmittanceController : public controller_interface::ControllerInterface {
public:
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  
  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
  void updateJointStates();
  void ft_sensor_callback(const geometry_msgs::msg::Wrench::SharedPtr msg);

  int num_joints = 7;
  std::vector<std::string> joint_names_;
  std::string robot_description_;
  std::string root_name_;
  std::string tip_name_;

  KDL::Tree kdl_tree_;
  KDL::Chain kdl_chain_;
  std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
  std::shared_ptr<KDL::ChainIkSolverVel_pinv> ik_vel_solver_;
  std::shared_ptr<KDL::ChainIkSolverPos_LMA> ik_pos_solver_;

  using Vector7d = Eigen::Matrix<double, 7, 1>;
  using Vector6d = Eigen::Matrix<double, 6, 1>;
  using Matrix6d = Eigen::Matrix<double, 6, 6>;

  Vector7d q_, dq_;
  Vector7d q_cmd_;
  Vector7d initial_q_;

  Matrix6d M_t_;
  Matrix6d K_Dt_;
  Matrix6d K_Pt_;
  
  Matrix6d M_t_inv_;

  Vector6d z_;
  Vector6d z_dot_;
  Vector6d z_ddot_;

  KDL::Frame x_d_;
  KDL::Frame x_t_;
  
  std::mutex ft_wrench_mutex_;
  geometry_msgs::msg::Wrench ft_wrench_data_;
  rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr ft_sensor_sub_;
  
  const Eigen::Matrix3d R_E_S_ = (Eigen::Matrix3d() << 
    1.0, 0.0, 0.0,
    0.0, 0.0, 1.0,
    0.0, -1.0, 0.0).finished();

  KDL::JntArray q_kdl_;
  bool first_update_ = true;
  double elapsed_time_ = 0.0;
};

} // namespace arm_controllers