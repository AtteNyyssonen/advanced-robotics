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
#include <cmath>

Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");

int main()
{

    
    KDL::Chain kdlChain = KDL::Chain();

    double dx = 0.0;
    double dz = 0.0;
    double l1 = 1.0;
    double l2 = 1.0;

    kdlChain.addSegment(KDL::Segment(
        KDL::Joint(KDL::Joint::None), 
        KDL::Frame(KDL::Vector(dx, 0.0, dz))));

    kdlChain.addSegment(KDL::Segment(
        KDL::Joint(KDL::Joint::RotY), 
        KDL::Frame(KDL::Vector(l1, 0.0, 0.0))));
    
    //End effector
    kdlChain.addSegment(KDL::Segment(
        KDL::Joint(KDL::Joint::RotY), 
        KDL::Frame(KDL::Vector(l2, 0.0, 0.0))));



    KDL::JntArray jointAngles = KDL::JntArray(2);

    jointAngles(0) = M_PI / 2; // Joint 1
    jointAngles(1) = -M_PI / 2; // Joint 2
    //jointAngles(2) = 1.0; // Joint 3 (not used, since it's fixed)
    
    
    KDL::ChainFkSolverPos_recursive FKSolver = KDL::ChainFkSolverPos_recursive(kdlChain);

    
    // Create a frame to hold the result
    KDL::Frame endEffectorPose;

    // Compute FK for the last segment (index = number of segments)
    int fk_status = FKSolver.JntToCart(jointAngles, endEffectorPose, -1);

    std::cout << fk_status << std::endl;

    if (fk_status >= 0) {
        std::cout << "End effector position: " 
                  << endEffectorPose.p.x() << ", "
                  << endEffectorPose.p.y() << ", "
                  << endEffectorPose.p.z() << std::endl;
    } else {
        std::cerr << "FK solver failed!" << std::endl;
    }

    return 0;
}
