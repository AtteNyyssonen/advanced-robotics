Answer: 1. Controller initialized with gain vectors, gains from elfin joints, urdf model of the robot's joints is
gathered and KDL used to parse the Urdf model to create a tree and chain from the tree. Also variables, inverse dynamics
solver and publishers are initialized. Then based on given commands these values are updated based on time.
Every cycle new values are calculated state is printed and the data saved.


3. Start with 
ros2 launch franka_gazebo velocity_controller.launch.py

Send goal, this will be interpolated into a linear trajectory
ros2 topic pub /cartesian_goal geometry_msgs/Point "{x: x.x, y: x.x
, z: x.x}"

Plotjuggler use is a bit hard.

6. Behaviour discussion on kinematic controllers joint space vs task space.

