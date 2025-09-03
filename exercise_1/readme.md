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
- What else KDL and Eigen library can do that you might use during the course ?

## Exercise 3: controllers
You will be writing your own controllers during the courses. There are example controller available in sim environment repo ![edu_franka_simulation](https://github.com/tau-alma/edu-franka_simulation)

- What controllers are available ?
- Where is the main controller loop and how does it work ?
- How kinematic chain of manipulator is parsed and utilized ?
- How states are read from simulator ?
- How control commands are sent to simulator ?
- How these controller files works together with ros2 control
