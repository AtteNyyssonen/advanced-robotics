### These exercises serve as introduction to Eigen for linear algebra and KDL for kinematic and dynamics solvers

## Exercise 1: forward kinematics

![](assets/advanced-robotics-exercise-1.drawio.svg)  2 DoF robot manipulator with joints $q_{1}$ and $q_{2}$ is given. Write position of end effector (FK), $x_{tip}$ using both KDL and Eigen:
1) both joints, $q_{1}$ and $q_{2}$ are revolute
2) $q_{1}$ is revolute but $q_{2}$ is prismatic


Templates for both KDL and Eigen are provided in src/.

To run these examples. First create ros2 package and then:

`colcon build` 


`ros2 run {your_package_name} kdl`


`ros2 run {your_package_name} eigen`


## Exercise 2: library 

- What is the matrix order in Eigen ?
- Matrixes are stored to memory either column by column (default) or row by row if defined.

- What else KDL and Eigen library can do that you might use during the course ?
- Calculations for forward and inverse kinematics. Jacobian calculations.

## Exercise 3: controllers
You will be writing your own controllers during the courses. There are example controller available in sim environment repo ![edu_franka_simulation](https://github.com/tau-alma/edu-franka_simulation)

- What controllers are available ?
- Controllers based on computed torque, joint impedance, joint position, joint velocity and time delay

- Where is the main controller loop and how does it work ?
- The ros launch command launches all the necessary nodes, controller, publishers, simulation. Then the nodes go to work
- based on the desired state of the simulated robot arm.

- How kinematic chain of manipulator is parsed and utilized ?
- The joint states from the kinematic chain are published and those are then read. Also the kinematic chain is the basis
- for the gazebo simulator rendering.

- How states are read from simulator ?
- Controller node reads data published from nodes like the robot description node and joint state publishers.

- How control commands are sent to simulator ?
- The desired/future calculated position get published to the node robot_description that gazebo uses to simulate the 
- robots pose.

- How these controller files works together with ros2 control
- The created controllers use/extend the controller_interface package from ros2. This means that they can be used
- by any application that also uses the ros2 control packages.
