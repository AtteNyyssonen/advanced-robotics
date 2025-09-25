#include <arm_controllers/gravity_pd_controller.hpp>

#include <cassert>
#include <cmath>
#include <exception>
#include <string>

#include "rclcpp/qos.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include <Eigen/Eigen>

namespace arm_controllers {

controller_interface::InterfaceConfiguration
GravityPDController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // Add the effort control interfaces for the joints
  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back("panda_joint" + std::to_string(i) + "/effort");
  }

  return config;
}

controller_interface::InterfaceConfiguration
GravityPDController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  
  // Add the position and velocity state interfaces for the joints
  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back("panda_joint" + std::to_string(i) + "/position");
    config.names.push_back("panda_joint" + std::to_string(i) + "/velocity");
  }

  return config;
}

controller_interface::return_type GravityPDController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/) {

  return controller_interface::return_type::OK;
}

CallbackReturn GravityPDController::on_init() {

  RCLCPP_INFO(get_node()->get_logger(), "Robot Initialization (on_init) started");

  // Initialize the joint names, gains, and the number of joints
  try {
    auto_declare<std::vector<std::string>>("joints", {});
    auto_declare<std::vector<double>>("p_gains", {});
    auto_declare<std::vector<double>>("d_gains", {});
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage (on_init) with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  G_.resize(num_joints);
  
  // Set the gravity vector
  gravity_ = KDL::Vector::Zero(); 
  gravity_(2) = -9.81;            

  // Initialize the KDL variables
  qd_.data = Eigen::VectorXd::Zero(num_joints);
  q_.data = Eigen::VectorXd::Zero(num_joints);
  qdot_.data = Eigen::VectorXd::Zero(num_joints);
  e_.data = Eigen::VectorXd::Zero(num_joints);
  e_dot_.data = Eigen::VectorXd::Zero(num_joints);

  // Create publishers for the desired and current joint positions, velocities, and accelerations
  pub_qd_ = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>("qd", 1000);
  pub_q_ = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>("q", 1000);
  pub_e_ = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>("e", 1000);

  RCLCPP_INFO(get_node()->get_logger(), "Robot Initialization (on_init) done");

  return CallbackReturn::SUCCESS;
}

CallbackReturn ComputedTorqueController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {

  RCLCPP_INFO(get_node()->get_logger(), "Robot Configuration (on_configure) started");

  // Get the robot description parameter
  auto parameters_client =
      std::make_shared<rclcpp::AsyncParametersClient>(get_node(), "/robot_state_publisher");
  parameters_client->wait_for_service();
  auto future = parameters_client->get_parameters({"robot_description"});
  auto result = future.get();

  // Check if the robot description was retrieved successfully
  if (!result.empty()) {
    robot_description_ = result[0].value_to_string();
  } else {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to get robot_description parameter.");
    return CallbackReturn::FAILURE;
  }

  // Get the joint names from the parameter server
  joint_names_ = auto_declare<std::vector<std::string>>("joints", joint_names_);
  if (joint_names_.empty()) {
    RCLCPP_FATAL(get_node()->get_logger(), "joint_names_ not set");
    return CallbackReturn::FAILURE;
  }
  // Check if there are the correct number of joint names
  if (joint_names_.size() != static_cast<uint>(num_joints)) {
    RCLCPP_FATAL(get_node()->get_logger(), "joint_names_ should be of size %d but is of size %ld",
                 num_joints, joint_names_.size());
    return CallbackReturn::FAILURE;
  }

  // Get the node's namespace
  Kp_.resize(num_joints);
  Kd_.resize(num_joints);

  // Get the gains from the parameter server
  std::vector<double> Kp(num_joints), Kd(num_joints);
  auto p_gains = get_node()->get_parameter("p_gains").as_double_array();
  auto d_gains = get_node()->get_parameter("d_gains").as_double_array();

  // Check if the gains were retrieved successfully
  if (p_gains.empty()) {
    RCLCPP_FATAL(get_node()->get_logger(), "p_gains parameter not set");
    return CallbackReturn::FAILURE;
  }
  if (p_gains.size() != static_cast<uint>(num_joints)) {
    RCLCPP_FATAL(get_node()->get_logger(), "p_gains should be of size %d but is of size %ld",
                 num_joints, p_gains.size());
    return CallbackReturn::FAILURE;
  }
  if (d_gains.empty()) {
    RCLCPP_FATAL(get_node()->get_logger(), "d_gains parameter not set");
    return CallbackReturn::FAILURE;
  }
  if (d_gains.size() != static_cast<uint>(num_joints)) {
    RCLCPP_FATAL(get_node()->get_logger(), "d_gains should be of size %d but is of size %ld",
                 num_joints, d_gains.size());
    return CallbackReturn::FAILURE;
  }

  // Set the gains for the controller
  for (int i = 0; i < num_joints; ++i) {
    Kp_(i) = p_gains.at(i);
    Kp[i] = p_gains.at(i);
    Kd_(i) = d_gains.at(i);
    Kd[i] = d_gains.at(i);
  }

  // Get the URDF model and the joint URDF objects
  urdf::Model urdf;
  if (!urdf.initString(robot_description_))
  {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to parse urdf file");
      return CallbackReturn::ERROR;
  }
  else
  {
      RCLCPP_INFO(get_node()->get_logger(), "Found robot_description");
  }

  // Get the joint URDF objects
  for (int i = 0; i < num_joints; i++)
  {
    urdf::JointConstSharedPtr joint_urdf = urdf.getJoint(joint_names_[i]);
    if (!joint_urdf)
    {
        RCLCPP_ERROR(get_node()->get_logger(), "Could not find joint '%s' in urdf", joint_names_[i].c_str());
        return CallbackReturn::ERROR;
    }
    joint_urdfs_.push_back(joint_urdf);
  }

  // Get the KDL tree from the robot description
  if (!kdl_parser::treeFromUrdfModel(urdf, kdl_tree_))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to construct kdl tree");
    return CallbackReturn::ERROR;
  }
  else
  {
    RCLCPP_INFO(get_node()->get_logger(), "Constructed kdl tree");
  }

  // Get the root and tip link names from the parameter server
  // If the parameter is not found, return an error
  std::string root_name, tip_name;
  if (get_node()->has_parameter("root_link"))
  {
    root_name = get_node()->get_parameter("root_link").as_string();
    RCLCPP_INFO(get_node()->get_logger(), "Found root link name form yaml: %s", root_name.c_str());
  }
  else
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Could not find root link name");
    return CallbackReturn::ERROR;
  }
  if (get_node()->has_parameter("tip_link"))
  {
    tip_name = get_node()->get_parameter("tip_link").as_string();
    RCLCPP_INFO(get_node()->get_logger(), "Found tip link name form yaml: %s", tip_name.c_str());
  }
  else
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Could not find tip link name");
    return CallbackReturn::ERROR;
  }

  // Get the KDL chain from the KDL tree
  // if kdl tree has no chain from root to tip, return error
  if (!kdl_tree_.getChain(root_name, tip_name, kdl_chain_))
  {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), "Failed to get KDL chain from tree: ");
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), "  " << root_name << " --> " << tip_name);
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), "  Tree has " << kdl_tree_.getNrOfJoints() << " joints");
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), "  Tree has " << kdl_tree_.getNrOfSegments() << " segments");
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), "  The segments are:");

    KDL::SegmentMap segment_map = kdl_tree_.getSegments();
    KDL::SegmentMap::iterator it;

    for (it = segment_map.begin(); it != segment_map.end(); it++)
    {
      RCLCPP_ERROR(get_node()->get_logger(), "    %s", std::string((*it).first).c_str());
    }

    return CallbackReturn::ERROR;
  }
  else
  {
    RCLCPP_INFO(get_node()->get_logger(), "Got kdl chain");

    // debug: print kdl tree and kdl chain
    RCLCPP_INFO(get_node()->get_logger(), "  %s --> %s", root_name.c_str(), tip_name.c_str());
    RCLCPP_INFO(get_node()->get_logger(), "  Tree has %d joints", kdl_tree_.getNrOfJoints());
    RCLCPP_INFO(get_node()->get_logger(), "  Tree has %d segments", kdl_tree_.getNrOfSegments());
    RCLCPP_INFO(get_node()->get_logger(), "  The kdl_tree_ segments are:");

    // Print the segments of the KDL tree
    KDL::SegmentMap segment_map = kdl_tree_.getSegments();
    KDL::SegmentMap::iterator it;
    for (it = segment_map.begin(); it != segment_map.end(); it++)
    {
      RCLCPP_INFO(get_node()->get_logger(), "    %s", std::string((*it).first).c_str());
    }
    RCLCPP_INFO(get_node()->get_logger(), "  Chain has %d joints", kdl_chain_.getNrOfJoints());
    RCLCPP_INFO(get_node()->get_logger(), "  Chain has %d segments", kdl_chain_.getNrOfSegments());
    RCLCPP_INFO(get_node()->get_logger(), "  The kdl_chain_ segments are:");
    for (unsigned int i = 0; i < kdl_chain_.getNrOfSegments(); i++) {
        const KDL::Segment& segment = kdl_chain_.getSegment(i);
        RCLCPP_INFO(get_node()->get_logger(), "    %s", segment.getName().c_str());
    }
  }

  // Create the KDL chain dyn param solver
  id_solver_.reset(new KDL::ChainDynParam(kdl_chain_, gravity_));
  G_.resize(kdl_chain_.getNrOfJoints());

  // print kdltree, kdlchain, jointnames, jointurdfs for learning purposes
  fprintf(stderr, "Number of segments in kdl_tree_: %d\n", kdl_tree_.getNrOfSegments());
  fprintf(stderr, "Number of joints in kdl_chain_: %d\n", kdl_chain_.getNrOfJoints());
  fprintf(stderr, "Joint names in joint_names_: ");
  for (int i = 0; i < num_joints; i++)
  {
    fprintf(stderr, "%s ", joint_names_[i].c_str());
  }
  fprintf(stderr, "\n");

  RCLCPP_INFO(get_node()->get_logger(), "Robot Configuration (on_configure) done");

  return CallbackReturn::SUCCESS;
}

CallbackReturn GravityPDController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {

  G_.data.setZero();

  t = 0.0;  // Initialize the simulation time variable

  // Initialize the variables
  qd_.resize(num_joints);
  q_.resize(num_joints);
  qdot_.resize(num_joints);
  e_.resize(num_joints);
  e_dot_.resize(num_joints);

  Kp_.resize(num_joints);
  Kd_.resize(num_joints);

  // Activate the publishers
  pub_qd_->on_activate();
  pub_q_->on_activate();
  pub_e_->on_activate();

  return CallbackReturn::SUCCESS;
}

}
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(arm_controllers::GravityPDController,
                       controller_interface::ControllerInterface)