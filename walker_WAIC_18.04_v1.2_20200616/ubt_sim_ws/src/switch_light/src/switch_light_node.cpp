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
//leg kinematic lib
#include "kinematic/kinematics.h"

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

  double time = 0.0;
  double LSPposition = PI/2;
  int iteration = 300;
  int count = 0;
  double d_LSPposition = LSPposition/iteration;
  double desired_LSPposition = 0;
  
  bool is_first = false;
  const size_t limb_joint_count = 7;
  while (ros::ok()) {
    time += 0.001;
    count ++;

    // control arm joints
    left_arm_data.mode = 5;
    if(count <= 300)
    {
      desired_LSPposition += d_LSPposition;
    }
    left_arm_data.command[0] = desired_LSPposition;
    left_arm_data.command[1] = 0;
    left_arm_data.command[2] = 0;
    left_arm_data.command[3] = 0;
    left_arm_data.command[4] = 0;
    left_arm_data.command[5] = 0;
    left_arm_data.command[6] = 0;
    test_pub.publish(left_arm_data);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
