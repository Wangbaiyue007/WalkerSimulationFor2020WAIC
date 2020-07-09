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
//gait srv msgs
#include "walker_srvs/leg_motion_MetaFuncCtrl.h"
//Forward & Inverse kinematics lib
#include "walker_arm_kin.h"

#include <sstream>

std::string leg_status;
// subscribe the leg status topic
void subWalkerStatus(const std_msgs::String &msgs)
{
    leg_status = msgs.data;
    //std::cout << "status  " << leg_status << std::endl;
}

int64_t step_num = 0;
// subscribe the step number topic
void subWalkerStepNum(const std_msgs::Int64 &msgs)
{
    step_num = msgs.data;
    //std::cout << "step_num  " << step_num << std::endl;
}

MatrixXd fetch();

int main(int argc, char **argv)
{
    ros::init(argc, argv, "puch_cart_node", ros::init_options::AnonymousName);

    ros::NodeHandle n;

    //pub walking velocity
    ros::Publisher walker_vel =
        n.advertise<geometry_msgs::Twist>("/nav/cmd_vel_nav", 10);

    //sub gait status
    ros::Subscriber walker_status_sub =
        n.subscribe("/Leg/leg_status", 10, &subWalkerStatus);
    //sub gait step num
    ros::Subscriber walker_step_num_sub =
        n.subscribe("/Leg/StepNum", 10, &subWalkerStepNum);

    //gait start and stop service
    ros::ServiceClient client =
        n.serviceClient<walker_srvs::leg_motion_MetaFuncCtrl>(
            "/Leg/TaskScheduler");

    //gait velocity data test
    geometry_msgs::Twist vel_ctrl;

    //gait service data test
    walker_srvs::leg_motion_MetaFuncCtrl walker_walking;
    walker_walking.request.func_name = "dynamic";
    walker_walking.request.cmd = "start";

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
        if (time > 1.2 && time < 1.7)
        {
            for (size_t ll = 0; ll < limb_joint_count; ll++)
            {
                left_arm_data.command[ll] = result(0, ll);
                right_arm_data.command[ll] = result(4, ll);
            }
        }
        else if (time > 1.7)
        {
            for (size_t ll = 0; ll < limb_joint_count; ll++)
            {
                left_arm_data.command[ll] = result(1, ll);
                right_arm_data.command[ll] = result(5, ll);
            }
        } /*
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

        if (time > 2)
        {
            left_hand_data.mode = 7;
            right_hand_data.mode = 7;
            left_hand_data.command[0] = 0.32;
            left_hand_data.command[1] = 0.0;
            right_hand_data.command[0] = 0.32;
            right_hand_data.command[1] = 0.0;
            for (size_t ll = 1; ll < 5; ll++)
            {
                left_hand_data.command[ll * 2] = 0.06;
                left_hand_data.command[ll * 2 + 1] = 0.012;
                right_hand_data.command[ll * 2] = 0.06;
                right_hand_data.command[ll * 2 + 1] = 0.012;
            }
            finger_l_pub.publish(left_hand_data);
            finger_r_pub.publish(right_hand_data);
        }

        if (time > 2.2)
        {
            // call walker walking
            if (leg_status == "standing" && step_num == 0 && !is_first)
            {
                walker_walking.request.func_name = "dynamic";
                walker_walking.request.cmd = "start";
                if (client.call(walker_walking))
                {
                    ROS_INFO("call success!!!");
                }
            }

            // control walking velocity and direction
            if (leg_status == "dynamic")
            {
                vel_ctrl.linear.x = 0.3;
                //vel_ctrl.angular.z = 0.0;
                if (step_num > 4)
                    vel_ctrl.linear.x = 0.25;
                walker_vel.publish(vel_ctrl);
            }

            // call walker stop
            if (leg_status == "dynamic" && step_num > 9 && !is_first)
            {
                walker_walking.request.func_name = "dynamic";
                walker_walking.request.cmd = "stop";
                is_first = true;
                if (client.call(walker_walking))
                {
                    ROS_INFO("call success!!!");
                }
            }
        }

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
    VectorXd cart_Position(3);
    cart_Position << 0.4, 0.0, 0.0;

    //path of the urdf
    std::string ModelPath = "/home/baiyue/walker_team/walker_WAIC_18.04_v1.2_20200616/ubt_sim_ws/src/example/config/walker.urdf";
    //estimate the first middle point
    VectorXd l_midpoint1(7), r_midpoint1(7);
    l_midpoint1 << 0.2 * PI, -0.2 * PI, -PI / 8, -0.26 * PI, 0, 0.1 * PI, 0.1 * PI;
    r_midpoint1 << -0.2 * PI, -0.2 * PI, PI / 8, -0.26 * PI, 0, -0.1 * PI, 0.1 * PI;
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

    VectorXd l_midpoint2(7), r_midpoint2(7);
    l_midpoint2 << 0.24 * PI, -0.09 * PI, -PI / 8, -0.24 * PI, 0.2 * PI, 0.35, 0.2;
    r_midpoint2 << -0.24 * PI, -0.09 * PI, PI / 8, -0.24 * PI, -0.2 * PI, -0.35, 0.2;
    for (int i = 0; i < 7; i++)
    {
        result(0, i) = l_midpoint1(i);
        result(1, i) = l_midpoint2(i);
        result(4, i) = r_midpoint1(i);
        result(5, i) = r_midpoint2(i);
    }

    return result;
}
