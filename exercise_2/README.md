Implementation -> Simple Reactivity, preprogrammed simple continous movement behaviour.

To launch the simulation:
ros2 launch franka_gazebo switching_example.launch.py

First movement controller should be activated by default.

To change the movement controller in use send a following message to topic

ros2 topic pub /movement_controller/select_controller std_msgs/msg/String "{data: 'first'}"

Implemented controllers are 'first' and 'second'. Already active controller is checked and input validity is checked.

If "main" movement controller is deactived -> every controller gets deactivated. On reactivation movement controller
defaults to the first controller.

Separated code for movement controllers and the "main" controller.
