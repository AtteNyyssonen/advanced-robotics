#include "arm_controllers/movement_controller.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace arm_controllers
{

MovementController::MovementController()
: controller_interface::ControllerInterface() {}

controller_interface::CallbackReturn MovementController::on_init()
{
  active_controller_.clear();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MovementController::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto node = get_node();

  command_sub_ = node->create_subscription<std_msgs::msg::String>(
      "~/select_controller", 10,
      std::bind(&MovementController::command_callback, this, std::placeholders::_1));

  switch_client_ = node->create_client<controller_manager_msgs::srv::SwitchController>(
      "/controller_manager/switch_controller");

  RCLCPP_INFO(node->get_logger(), "MovementController configured");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MovementController::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_node()->get_logger(), "MovementController activated");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MovementController::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_node()->get_logger(), "MovementController deactivated");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
MovementController::command_interface_configuration() const
{
  return controller_interface::InterfaceConfiguration{
      controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration
MovementController::state_interface_configuration() const
{
  return controller_interface::InterfaceConfiguration{
      controller_interface::interface_configuration_type::NONE};
}

controller_interface::return_type
MovementController::update(const rclcpp::Time & /*time*/,
                           const rclcpp::Duration & /*period*/)
{
  return controller_interface::return_type::OK;
}

void MovementController::command_callback(const std_msgs::msg::String::SharedPtr msg)
{
  auto node = get_node();
  const std::string &requested = msg->data;

  if (requested != "first" && requested != "second" && requested != "third")
  {
    RCLCPP_WARN(node->get_logger(), "Invalid controller request: %s", requested.c_str());
    return;
  }

  std::string requested_controller = requested + "_movement_controller";

  auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
  request->strictness = 2;
  request->start_controllers = {requested_controller};

  if (!active_controller_.empty() && active_controller_ != requested)
  {
    request->stop_controllers = {active_controller_ + "_movement_controller"};
  }

  if (!switch_client_->wait_for_service(std::chrono::seconds(1)))
  {
    RCLCPP_ERROR(node->get_logger(), "Switch service not available");
    return;
  }

  auto future = switch_client_->async_send_request(request);

  try
  {
    auto response = future.get();
    if (response->ok)
    {
      active_controller_ = requested;
      RCLCPP_INFO(node->get_logger(),
                  "Switched to %s controller", requested_controller.c_str());
    }
    else
    {
      RCLCPP_ERROR(node->get_logger(),
                   "Failed to switch to %s", requested_controller.c_str());
    }
  }
  catch (const std::exception &e)
  {
    RCLCPP_ERROR(node->get_logger(), "Exception while calling switch service: %s", e.what());
  }
}

}

PLUGINLIB_EXPORT_CLASS(arm_controllers::MovementController,
                       controller_interface::ControllerInterface)