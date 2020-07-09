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

MatrixXd fetch(int cupnum)
{
    MatrixXd result(4, 7);
    VectorXd base_l_Position(3);
    base_l_Position << 0.055, -0.027, 0.43;
    VectorXd base_r_Position(3);
    base_r_Position << 0.055, 0.027, 0.43;
    VectorXd Cup1_Position(3);
    Cup1_Position << 0.41, 0.43, 0.21;
    VectorXd Cup2_Position(3);
    Cup2_Position << 0.41, 0.18, 0.214;
    VectorXd Cup3_Position(3);
    Cup3_Position << 0.41, -0.07, 0.214;
    VectorXd Cup4_Position(3);
    Cup4_Position << 0.41, -0.32, 0.214;
    VectorXd Cup5_Position(3);
    Cup5_Position << 0.41, -0.57, 0.214;

    if (cupnum == 1 || 2)
    {
        //path of the urdf
        std::string ModelPath = "/home/baiyue/walker_team/walker_WAIC_18.04_v1.2_20200616/ubt_sim_ws/src/example/config/walker.urdf";
        //estimate the first middle point
        VectorXd midpoint1(7);
        midpoint1 << -1, -0.1, -PI * 0.4, -1.8, PI * 0.4, 0, 0;
        //calculate forward kinematics to get an end orientation
        Tree walker_tree;
        kdl_parser::treeFromFile(ModelPath, walker_tree);
        VectorXd fkresult(7);
        forward_kinematics_left(midpoint1, walker_tree, fkresult);
        cout << "mid 1: " << endl;
        cout << fkresult << endl;

        VectorXd MidPosition2(3), GoalPosition(3), LiftPosition(3);
        Quaterniond q, q2, q3;
        if (cupnum == 1)
        {
            MidPosition2 = Cup1_Position - base_l_Position;
            MidPosition2(0) = MidPosition2(0) - 0.1;
            MidPosition2(1) = MidPosition2(1) + 0.02;
            MidPosition2(2) = MidPosition2(2) + 0.06;

            GoalPosition = Cup1_Position - base_l_Position;
            GoalPosition(0) = GoalPosition(0) + 0.008;
            GoalPosition(1) = GoalPosition(1) - 0.018;

            LiftPosition = GoalPosition;
            LiftPosition(2) = LiftPosition(2) + 0.02;

            Eigen::AngleAxisd yawAngle(3, Eigen::Vector3d::UnitZ());
            Eigen::AngleAxisd rollAngle(PI * 0.5, Eigen::Vector3d::UnitX());
            Eigen::AngleAxisd pitchAngle(-0.1, Eigen::Vector3d::UnitY());
            q = yawAngle * pitchAngle * rollAngle;

            Eigen::AngleAxisd yawAngle2(3, Eigen::Vector3d::UnitZ());
            Eigen::AngleAxisd rollAngle2(PI * 0.5, Eigen::Vector3d::UnitX());
            Eigen::AngleAxisd pitchAngle2(-0.1, Eigen::Vector3d::UnitY());
            q2 = yawAngle2 * pitchAngle2 * rollAngle2;

            Eigen::AngleAxisd yawAngle3(PI * 1.1, Eigen::Vector3d::UnitZ());
            q3 = yawAngle3 * pitchAngle2 * rollAngle2;
        }
        //cout << GoalPosition << endl;
        //set the second middle position and orientation of the end point
        VectorXd MidOrientation2(4);
        MidOrientation2(0) = q2.w();
        MidOrientation2(1) = q2.x();
        MidOrientation2(2) = q2.y();
        MidOrientation2(3) = q2.z();
        VectorXd midpoint2(7);
        cout << "mid 2: " << endl;
        get_inverse_left(ModelPath, MidPosition2, MidOrientation2, midpoint2);

        //set the goal position and orientation of the end point
        VectorXd GoalOrientation(4);
        GoalOrientation(0) = q.w();
        GoalOrientation(1) = q.x();
        GoalOrientation(2) = q.y();
        GoalOrientation(3) = q.z();
        VectorXd goal(7);
        cout << "goal: " << endl;
        get_inverse_left(ModelPath, GoalPosition, GoalOrientation, goal);

        VectorXd LiftOrientation(4);
        LiftOrientation(0) = q3.w();
        LiftOrientation(1) = q3.x();
        LiftOrientation(2) = q3.y();
        LiftOrientation(3) = q3.z();
        VectorXd lift(7);
        cout << "lift: " << endl;
        get_inverse_left(ModelPath, LiftPosition, LiftOrientation, lift);
        for (int i = 0; i < 7; i++)
        {
            result(0, i) = midpoint1(i);
            result(1, i) = midpoint2(i);
            result(2, i) = goal(i);
            result(3, i) = lift(i);
        }
    }

    return result;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "grasp_cup_node", ros::init_options::AnonymousName);

    ros::NodeHandle n;

    //pub left arm joints' data
    ros::Publisher arm_l_pub = n.advertise<ubt_core_msgs::JointCommand>(
        "/walker/leftLimb/controller", 10);

    //left arm data test
    ubt_core_msgs::JointCommand left_arm_data;
    left_arm_data.names.resize(7);
    left_arm_data.command.resize(7);

    //pub left finger joints' data
    ros::Publisher finger_l_pub = n.advertise<ubt_core_msgs::JointCommand>(
        "/walker/leftHand/controller", 10);

    //left finger data test
    ubt_core_msgs::JointCommand left_hand_data;
    left_hand_data.names.resize(10);
    left_hand_data.command.resize(10);

    ros::Rate loop_rate(1000);

    MatrixXd result(3, 7);
    cout << "Which cup to grasp?" << endl;
    int x;
    cin >> x;
    result = fetch(x);

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
            }
        }
        else if (time < 0.6)
        {
            for (size_t ll = 0; ll < limb_joint_count; ll++)
            {
                left_arm_data.command[ll] = result(1, ll);
            }
        }
        else if (time < 0.9)
        {
            for (size_t ll = 0; ll < limb_joint_count; ll++)
            {
                left_arm_data.command[ll] = result(2, ll);
            }
        }

        if (time > 1.5)
        {
            left_arm_data.mode = 6;
            for (size_t ll = 0; ll < limb_joint_count; ll++)
            {
                left_arm_data.command[ll] = 0.0;
            }
            if (time < 2)
                left_arm_data.command[0] = 1;
        }
        arm_l_pub.publish(left_arm_data);

        // control hand joints (torque & position)

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
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
