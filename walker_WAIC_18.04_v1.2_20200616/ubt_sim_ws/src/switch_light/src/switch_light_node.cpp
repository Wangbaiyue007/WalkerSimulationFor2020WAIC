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

std::string leg_status;

int main(int argc, char **argv) {
  ros::init(argc, argv, "switch_light_node", ros::init_options::AnonymousName);

  ros::NodeHandle n;

  //pub left arm joints' data
  ros::Publisher test_pub = n.advertise<ubt_core_msgs::JointCommand>(
      "/walker/leftLimb/controller", 10);

  //left arm data test
  ubt_core_msgs::JointCommand left_arm_data;
  left_arm_data.names.resize(7);
  left_arm_data.command.resize(7);

  ros::Rate loop_rate(1000);

  std::string ModelPath = "/home/baiyue/walker_team/walker_WAIC_18.04_v1.2_20200616/ubt_sim_ws/src/example/config/walker.urdf";
  //estimate joint angles
  VectorXd JointValues(7);
  JointValues << PI/5, -PI/5, -PI*0.6, -PI*0.5, PI/2, 0, 0;
  //calculate forward kinematics to get an end orientation
  Tree walker_tree;
  kdl_parser::treeFromFile(ModelPath, walker_tree);
  VectorXd fkresult(7);
  forward_kinematics_left(JointValues, walker_tree, fkresult);
  //set the goal position of the end point
  VectorXd GoalPosition(3);
  GoalPosition << 0.4, 0.4, 0.1;
  //obtain orientation from forward kinmatics result
  VectorXd GoalOrientation(4);
  for(int i = 0; i < 4; i++)
  {
    GoalOrientation(i) = fkresult(i+3);
  }
  //calculate the actual joint angle
  VectorXd result(7);
  get_inverse_left(ModelPath, GoalPosition, GoalOrientation, result);

  double time = 0.0;
  
  bool is_first = false;
  const size_t limb_joint_count = 7;
  while (ros::ok()) {
    time += 0.001;
    // control arm joints (position)
    left_arm_data.mode = 5;
    
    for(size_t ll = 0; ll < limb_joint_count; ll++)
    {
      left_arm_data.command[ll] = result(ll);
    }
    test_pub.publish(left_arm_data);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
