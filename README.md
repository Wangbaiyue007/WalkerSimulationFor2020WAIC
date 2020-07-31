# 妮可机器康复队

## Instruction

### Set Up

1. 编辑在主文件夹下的隐藏文件.*bashrc*，在最后加上：

    source $(walker_install)/walker_install/setup.bash  
    source $(ubt_sim_ws)/ubt_sim_ws/devel/setup.bash  
    export ROS_PACKAGE_PATH=$(ubt_sim_ws)/ubt_sim_ws/src:$ROS_PACKAGE_PATH
2. cd to */ubt_sim_ws*, and try run *catkin_make*
3. If not successful, check *CMakeLists.txt*

### Launch Example

Open webots

    $ webots
Open world file *walker_model/world/WAIC.wbt*

Launch the json file that contains the key, and the robot will bend his knee:

    $ roslaunch leg_motion walker2_leg.launch account_file:=${walker_WAIC}/walker_WAIC/user_account.json
Launch example:

    $ roslaunch example example.launch

### Missions

Before launching the program, change the **ModelPath** in *switch_light_node.cpp* to your urdf path.

1.  

    $ roslaunch motion_control switch_light.launch

2. 

    $ roslaunch motion_control gras_pcup.launch

3.  

    $ roslaunch motion_control push_cart.launch

4. 

    $ roslaunch motion_control open_fridge.launch

## 赛事概述

WAIC黑客马拉松作为世界人工智能大会期间的重磅赛事和特色环节，将于2020年7月8日-11日在线上举办。ROBO GENIUS是优必选科技重磅打造的机器人及AI教育创新成长平台，聚集各领域专家、老师、学生及生态伙伴，提供贯穿K12到高校的各类人工智能及机器人赛事。优必选科技ROBO GENIUS作为本次大赛的联合承办方，与机器之心联合举办了走进未来·Walker大型仿人服务机器人仿真挑战赛（以下简称“Walker仿真挑战赛”）。  
  
赛事主题

让人形机器人进入家庭，成为家庭重要的一员，是优必选科技从未改变过的目标。Walker是优必选科技研发的中国第一款可商业化落地的大型仿人服务机器人，与波士顿动力的Atlas、本田的Asimo等共同入选“全球5大人形机器人”。

Walker具备36个高性能伺服关节以及力觉、视觉、听觉等全方位的感知系统，集合了优必选科技多年来在人工智能及机器人领域的尖端科技，可以实现平稳快速的行走和灵活精准的操作，具备了在常用家庭场景和办公场景的自由活动和服务的能力，开始真正走入人们的生活。

本届比赛基于仿真平台开放Walker模型及相关数据，邀请更多高校、科研机构以及顶级开发者们参加Walker仿真挑战赛，研发完成15项不同难度的任务，共同推动大型仿人服务机器人的落地应用和研发。
