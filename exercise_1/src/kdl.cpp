/*
Simple example showing how to use solvers in the KDL and creating kinematic model.

Task 1: Your task is to create kinematic chain in KDL using KDL::Chain and calculate the end effector rotation
        and translation matrices using KDL::ChainFkSolverPos_recursive to calculate the forward kinematics

    HELP: For the Chain creation http://docs.ros.org/en/indigo/api/orocos_kdl/html/classKDL_1_1Chain.html
    and for the solver http://docs.ros.org/en/indigo/api/orocos_kdl/html/classKDL_1_1TreeFkSolverPos__recursive.html

Task 2: Access the position and rotation data from the KDL::Frame variable
    HELP: http://docs.ros.org/en/indigo/api/orocos_kdl/html/classKDL_1_1Frame.html

*/
#include <kdl/tree.hpp>
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <iostream>

Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");

int main(int argc, char **argv)
{

    
    KDL::Chain kdlChain = KDL::Chain();
    
    // your code here to form chain

    // your code here to form chain


    KDL::JntArray jointAngles = KDL::JntArray(3);
    
    // your code here to set joint angles
    
    // your code here to set joint angles
    
    
    KDL::ChainFkSolverPos_recursive FKSolver = KDL::ChainFkSolverPos_recursive(kdlChain);


    // Your code here to solve Forward Kinematics

    // Your code here to solve Forward Kinematics

    return 0;
}