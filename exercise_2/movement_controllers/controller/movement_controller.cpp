#include "arm_controllers/movement_controller.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace arm_controllers
{

MovementController::MovementController(): controller_interface::ControllerInterface() {}

controller_interface::CallbackReturn MovementController::on_init()
{
  active_controller_.clear();
  implemented_controllers_ = {"first", "second"};
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
  auto node = get_node();
  RCLCPP_INFO(node->get_logger(), "MovementController activated");

  if (!switch_client_->wait_for_service(std::chrono::seconds(5))) {
    RCLCPP_ERROR(node->get_logger(), "Switch service not available in on_activate()");
    return controller_interface::CallbackReturn::FAILURE;
  }

  auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
  request->strictness = 2;
  request->start_controllers = {"first_movement_controller"};

  auto future = switch_client_->async_send_request(
    request,
    [this, node](rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedFuture response)
    {
      if (response.get()->ok) {
        active_controller_ = "first";
        RCLCPP_INFO(node->get_logger(), "Default controller activated: first_movement_controller");
      } else {
        RCLCPP_ERROR(node->get_logger(), "Failed to activate first_movement_controller as default controller");
      }
    });

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MovementController::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto node = get_node();
  RCLCPP_INFO(node->get_logger(), "MovementController deactivated");
  if (!active_controller_.empty())
  {
    auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
    request->strictness = 2;
    request->stop_controllers = {active_controller_ + "_movement_controller"};

    switch_client_->async_send_request(
      request,
      [this, node](rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedFuture response)
      {
        if (response.get()->ok) {
          RCLCPP_INFO(node->get_logger(),
                      "Stopped %s controller on deactivation",
                      (active_controller_ + "_movement_controller").c_str());
          active_controller_.clear();
        } else {
          RCLCPP_WARN(node->get_logger(),
                      "Failed to stop %s on deactivation",
                      (active_controller_ + "_movement_controller").c_str());
        }
      }
    );
  }
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

  if (std::find(implemented_controllers_.begin(), implemented_controllers_.end(), requested) == implemented_controllers_.end())
  {
    std::stringstream ss;
    for (auto &c : implemented_controllers_) {
      ss << c << " ";
    }
    RCLCPP_WARN(node->get_logger(),
                "Invalid controller request: %s. Implemented controllers are: [%s]",
                requested.c_str(), ss.str().c_str());
    return;
  }

  if (requested == active_controller_)
  {
    RCLCPP_INFO(node->get_logger(),
                "%s_movement_controller is already active. No switch performed.",
                requested.c_str());
    return;
  }

  std::string requested_controller = requested + "_movement_controller";

  auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
  request->strictness = 2;
  request->start_controllers = {requested_controller};

  if (!active_controller_.empty())
  {
    request->stop_controllers = {active_controller_ + "_movement_controller"};
  }

  if (!switch_client_->wait_for_service(std::chrono::seconds(1)))
  {
    RCLCPP_ERROR(node->get_logger(), "Switch service not available");
    return;
  }

  switch_client_->async_send_request(
    request,
    [this, node, requested, requested_controller](
      rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedFuture response)
    {
      if (response.get()->ok)
      {
        active_controller_ = requested;
        RCLCPP_INFO(node->get_logger(),
                    "Switched to %s", requested_controller.c_str());
      }
      else
      {
        RCLCPP_ERROR(node->get_logger(),
                     "Failed to switch to %s", requested_controller.c_str());
      }
    }
  );
}

}

PLUGINLIB_EXPORT_CLASS(arm_controllers::MovementController,
                       controller_interface::ControllerInterface)
