/*
Simple example showing how to use solvers in the KDL and creating kinematic model.

Task 1: Exercise task is to create chain forward kinematic solver and read end effector position given 
the kinematic chain.

*/
#include <kdl/tree.hpp>
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <iostream>
#include <Eigen/Geometry>
#include <cmath>


Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");



int main()
{

    double dx = 0.0;
    double dz = 0.0;
    double l1 = 1.0;
    double l2 = 1.0;

    float theta1 = 0.0;//M_PI / 2; // Joint 1
    float theta2 = 0.0;//M_PI / 2; // Joint 2

    // create matrices needed for calculation

    Eigen::Matrix4f A1 = Eigen::Matrix4f::Identity();
    A1(0,3) = dx;
    A1(2,3) = dz;

    Eigen::Matrix4f A2 = Eigen::Matrix4f::Identity();
    A2(0,3) = l1;
    A2(0,0) = cos(theta1);
    A2(0,2) = sin(theta1);
    A2(2,0) = -sin(theta1);
    A2(2,2) = cos(theta1);

    Eigen::Matrix4f A3 = Eigen::Matrix4f::Identity();
    A3(0,3) = l2;
    A3(0,0) = cos(theta2);
    A3(0,2) = sin(theta2);
    A3(2,0) = -sin(theta2);
    A3(2,2) = cos(theta2);
    // Calculate the end effector position and save the type
    // to eeFrame
    Eigen::Matrix4f eeFrame; // final transform

    Eigen::Vector4f vec = Eigen::Vector4f(1.0, 0.0, 1.0, 0.0);
    eeFrame = A1 * A2 * A3;

    auto eeFrame_final = eeFrame * vec;


    std::cout << eeFrame_final.format(CleanFmt) << std::endl; // print formated matrix to console
    return 0;
}