

###
 # @Author       : GUO ZhengLong, LI Jinjie
 # @Date         : 2020-03-06 15:26:32
 # @LastEditors  : LI Jinjie
 # @LastEditTime : 2020-05-18 11:22:03
 # @Units        : None
 # @Description  : file content
 # @Dependencies : None
 # @NOTICE       : None
 ###

#!/bin/bash

cd ./catkin_ws/
catkin build
source $(pwd)/devel/setup.bash
echo "loading gazebo world..."

# add gazebo model path
export CATKIN_PATH=$(pwd)
cd ./src/innok_heros_gazebo/worlds/Apriltags_world/models
source add_path.sh
cd $CATKIN_PATH

roslaunch innok_heros_gazebo load_world_apriltag_map.launch &
sleep 10
echo "loading uav and car..."
roslaunch hector_quadrotor_gazebo spawn_quadrotor_for_RL_training.launch &

sleep 10
sh scripts/uav_arm.sh 1 /dev/null 2>&1 &
echo "all uav are ready to takeoff..."
sleep 10

echo "simulation platform ready..."
sleep 1
echo "simulation platform ready..."
sleep 1
echo "simulation platform ready..."
sleep 1
echo "simulation platform ready..."
sleep 1
echo "simulation platform ready..."
sleep 1
echo "simulation platform ready..."
sleep 1
echo "simulation platform ready..."
sleep 1
echo "simulation platform ready..."
sleep 1
echo "simulation platform ready..."
sleep 1
echo "simulation platform ready..."
wait

exit


