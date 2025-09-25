Answer: 1. Controller initialized with gain vectors, gains from elfin joints, urdf model of the robot's joints is
gathered and KDL used to parse the Urdf model to create a tree and chain from the tree. Also variables, inverse dynamics
solver and publishers are initialized. Then based on given commands these values are updated based on time.
Every cycle new values are calculated state is printed and the data saved.


6. Behaviour discussion on kinematic controllers joint space vs task space.

