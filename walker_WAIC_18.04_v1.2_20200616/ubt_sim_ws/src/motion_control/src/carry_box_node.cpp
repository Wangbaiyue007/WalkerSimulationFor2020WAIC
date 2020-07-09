// Copyright 2012-2020 Ubtech Robotics Corp. Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <ros/ros.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int64.h>
#include <std_msgs/String.h>

#include <string.h>

#include <ubt_core_msgs/JointCommand.h>
//Forward & Inverse kinematics lib
#include "walker_arm_kin.h"

#include <sstream>

MatrixXd fetch();
int main(int argc, char **argv)
{
    ros::init(argc, argv, "carry_box_node", ros::init_options::AnonymousName);

    ros::NodeHandle n;

    //pub arm joints' data
    ros::Publisher arm_l_pub = n.advertise<ubt_core_msgs::JointCommand>(
        "/walker/leftLimb/controller", 10);
    ros::Publisher arm_r_pub = n.advertise<ubt_core_msgs::JointCommand>(
        "/walker/rightLimb/controller", 10);

    //left arm data test
    ubt_core_msgs::JointCommand left_arm_data;
    left_arm_data.names.resize(7);
    left_arm_data.command.resize(7);
    ubt_core_msgs::JointCommand right_arm_data;
    right_arm_data.names.resize(7);
    right_arm_data.command.resize(7);

    //pub finger joints' data
    ros::Publisher finger_l_pub = n.advertise<ubt_core_msgs::JointCommand>(
        "/walker/leftHand/controller", 10);
    ros::Publisher finger_r_pub = n.advertise<ubt_core_msgs::JointCommand>(
        "/walker/rightHand/controller", 10);

    //finger data test
    ubt_core_msgs::JointCommand left_hand_data;
    left_hand_data.names.resize(10);
    left_hand_data.command.resize(10);
    ubt_core_msgs::JointCommand right_hand_data;
    right_hand_data.names.resize(10);
    right_hand_data.command.resize(10);

    ros::Rate loop_rate(1000);

    MatrixXd result(8, 7);
    result = fetch();

    double time = 0.0;
    bool is_first = false;
    const size_t limb_joint_count = 7;
    const size_t hand_joint_count = 10;
    while (ros::ok())
    {
        time += 0.001;
        // control arm joints (position)
        left_arm_data.mode = 5;
        if (time < 0.3)
        {
            for (size_t ll = 0; ll < limb_joint_count; ll++)
            {
                left_arm_data.command[ll] = result(0, ll);
                right_arm_data.command[ll] = result(4, ll);
            }
        }
        else if (time < 0.6)
        {
            for (size_t ll = 0; ll < limb_joint_count; ll++)
            {
                left_arm_data.command[ll] = result(1, ll);
                right_arm_data.command[ll] = result(5, ll);
            }
        }/*
        else if (time < 0.9)
        {
            for (size_t ll = 0; ll < limb_joint_count; ll++)
            {
                left_arm_data.command[ll] = result(2, ll);
                right_arm_data.command[ll] = result(6, ll);
            }
        }*/
        arm_l_pub.publish(left_arm_data);
        arm_r_pub.publish(right_arm_data);

        // control hand joints (torque & position)
        /*
        if (time > 0.72)
        {
            if (time < 1.0)
            {
                left_hand_data.mode = 5;
                left_hand_data.command[0] = 1.2;
                left_hand_data.command[1] = 0.5;
                for (size_t ll = 1; ll < 3; ll++)
                {
                    left_hand_data.command[ll * 2] = 0.5;
                    left_hand_data.command[ll * 2 + 1] = 0.6;
                }
                for (size_t ll = 3; ll < 5; ll++)
                {
                    left_hand_data.command[ll * 2] = 0.4;
                    left_hand_data.command[ll * 2 + 1] = 0.4;
                }
            }
            else
            {
                left_hand_data.mode = 7;
                left_hand_data.command[0] = 0.3;
                left_hand_data.command[1] = 0.06;
                for (size_t ll = 1; ll < 5; ll++)
                {
                    left_hand_data.command[ll * 2] = 0.06;
                    left_hand_data.command[ll * 2 + 1] = 0.012;
                }
            }
            finger_l_pub.publish(left_hand_data);
        }*/

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

MatrixXd fetch()
{
    MatrixXd result(8, 7);
    VectorXd base_l_Position(3);
    base_l_Position << 0.055, -0.027, 0.43;
    VectorXd base_r_Position(3);
    base_r_Position << 0.055, 0.027, 0.43;
    VectorXd box_origin(3);
    box_origin << 0.303, 0.0128, -0.0479;

    //path of the urdf
    std::string ModelPath = "/home/baiyue/walker_team/walker_WAIC_18.04_v1.2_20200616/ubt_sim_ws/src/example/config/walker.urdf";
    //estimate the first middle point
    VectorXd l_midpoint1(7), r_midpoint1(7);
    l_midpoint1 << -1, -0.2, -PI * 0.4, -1.8, PI * 0.4, 0, 0;
    r_midpoint1 << 1, -0.2, PI * 0.4, -1.8, -PI * 0.4, 0, 0;
    //calculate forward kinematics to get an end orientation
    Tree walker_tree;
    kdl_parser::treeFromFile(ModelPath, walker_tree);
    VectorXd l_fkresult(7), r_fkresult(7);
    forward_kinematics_left(l_midpoint1, walker_tree, l_fkresult);
    cout << "mid 1 left: " << endl;
    cout << l_fkresult << endl;
    forward_kinematics_right(r_midpoint1, walker_tree, r_fkresult);
    cout << "mid 1 right: " << endl;
    cout << r_fkresult << endl;

    VectorXd l_MidPosition2(3), l_GoalPosition(3), l_LiftPosition(3), r_MidPosition2(3), r_GoalPosition(3), r_LiftPosition(3);
    Quaterniond lq, lq2, lq3, rq, rq2, rq3;

    l_MidPosition2 = box_origin - base_l_Position;
    l_MidPosition2(0) = l_MidPosition2(0) + 0.1;
    l_MidPosition2(1) = l_MidPosition2(1) + 0.24;
    l_MidPosition2(2) = l_MidPosition2(2) + 0.22;
    r_MidPosition2 = box_origin - base_r_Position;
    r_MidPosition2(0) = r_MidPosition2(0) + 0.1;
    r_MidPosition2(1) = r_MidPosition2(1) - 0.24;
    r_MidPosition2(2) = r_MidPosition2(2) + 0.22;

    l_GoalPosition = box_origin - base_l_Position;
    l_GoalPosition(0) = l_GoalPosition(0);
    l_GoalPosition(1) = l_GoalPosition(1) + 0.2 - 0.018;
    l_GoalPosition(2) = l_GoalPosition(2) + 0.2 - 0.018;
    r_GoalPosition = box_origin - base_r_Position;
    r_GoalPosition(0) = r_GoalPosition(0);
    r_GoalPosition(1) = r_GoalPosition(1) - 0.2 + 0.018;
    r_GoalPosition(2) = r_GoalPosition(2) + 0.2 - 0.018;

    l_LiftPosition = l_GoalPosition;
    l_LiftPosition(2) = l_LiftPosition(2) + 0.02;
    r_LiftPosition = r_GoalPosition;
    r_LiftPosition(2) = r_LiftPosition(2) + 0.02;

    Eigen::AngleAxisd yawAngle_l(-PI, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd yawAngle_r(PI, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd rollAngle_l(PI*0.8, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd rollAngle_r(PI*0.8, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle_l(0.1, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd pitchAngle_r(-0.1, Eigen::Vector3d::UnitY());
    lq = yawAngle_l * pitchAngle_l * rollAngle_l;
    rq = yawAngle_r * pitchAngle_r * rollAngle_r;

    Eigen::AngleAxisd yawAngle2_l(-PI, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd yawAngle2_r(PI, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd rollAngle2_l(PI*0.85, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd rollAngle2_r(PI*0.85, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle2_l(0.05, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd pitchAngle2_r(-0.05, Eigen::Vector3d::UnitY());
    lq2 = yawAngle2_l * pitchAngle2_l * rollAngle2_l;
    rq2 = yawAngle2_r * pitchAngle2_r * rollAngle2_r;

    lq3 = lq2;
    rq3 = rq2;

    //cout << GoalPosition << endl;
    //set the second middle position and orientation of the end point
    VectorXd l_MidOrientation2(4), r_MidOrientation2(4);
    l_MidOrientation2(0) = lq2.w(); r_MidOrientation2(0) = rq2.w();
    l_MidOrientation2(1) = lq2.x(); r_MidOrientation2(1) = rq2.x();
    l_MidOrientation2(2) = lq2.y(); r_MidOrientation2(2) = rq2.y();
    l_MidOrientation2(3) = lq2.z(); r_MidOrientation2(3) = rq2.z();
    VectorXd l_midpoint2(7), r_midpoint2(7);
    cout << "mid 2 left: " << endl;
    get_inverse_left(ModelPath, l_MidPosition2, l_MidOrientation2, l_midpoint2);
    cout << "mid 2 right: " << endl;
    get_inverse_right(ModelPath, r_MidPosition2, r_MidOrientation2, r_midpoint2);

    //set the goal position and orientation of the end point
    VectorXd l_GoalOrientation(4), r_GoalOrientation(4);
    l_GoalOrientation(0) = lq.w(); r_GoalOrientation(0) = rq.w();
    l_GoalOrientation(1) = lq.x(); r_GoalOrientation(1) = rq.x();
    l_GoalOrientation(2) = lq.y(); r_GoalOrientation(2) = rq.y();
    l_GoalOrientation(3) = lq.z(); r_GoalOrientation(3) = rq.z();
    VectorXd l_goal(7), r_goal(7);
    cout << "goal left: " << endl;
    get_inverse_left(ModelPath, l_GoalPosition, l_GoalOrientation, l_goal);
    cout << "goal right: " << endl;
    get_inverse_right(ModelPath, r_GoalPosition, r_GoalOrientation, r_goal);

    VectorXd l_LiftOrientation(4), r_LiftOrientation(4);
    l_LiftOrientation(0) = lq3.w(); r_LiftOrientation(0) = rq3.w();
    l_LiftOrientation(1) = lq3.x(); r_LiftOrientation(1) = rq3.x();
    l_LiftOrientation(2) = lq3.y(); r_LiftOrientation(2) = rq3.y();
    l_LiftOrientation(3) = lq3.z(); r_LiftOrientation(3) = rq3.z();
    VectorXd l_lift(7), r_lift(7);
    cout << "lift left: " << endl;
    get_inverse_left(ModelPath, l_LiftPosition, l_LiftOrientation, l_lift);
    cout << "lift right: " << endl;
    get_inverse_right(ModelPath, r_LiftPosition, r_LiftOrientation, r_lift);
    for (int i = 0; i < 7; i++)
    {
        result(0, i) = l_midpoint1(i);
        result(1, i) = l_midpoint2(i);
        result(2, i) = l_goal(i);
        result(3, i) = l_lift(i);
        result(4, i) = r_midpoint1(i);
        result(5, i) = r_midpoint2(i);
        result(6, i) = r_goal(i);
        result(7, i) = r_lift(i);
    }

    return result;
}
