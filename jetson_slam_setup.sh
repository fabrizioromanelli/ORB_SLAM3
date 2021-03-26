#!/bin/bash
NUM_CPU=$(nproc)

mkdir ~/workspace
cd ~/workspace

echo "Cloning realsense and ORB_SLAM3 projects"
git clone git@github.com:fabrizioromanelli/installRealSenseSDK.git
git clone git@github.com:fabrizioromanelli/ORB_SLAM3.git -b development
mkdir -p ~/workspace/ros2_ws/src
cd ~/workspace/ros2_ws/src
echo "Cloning realsense and ORB_SLAM3 projects for ROS2"
git clone git@github.com:fabrizioromanelli/realsense-ros.git -b foxy
git clone git@github.com:fabrizioromanelli/orbslam3-ros2.git

echo "Compiling and installing realsense2 SDK"
sudo apt install g++-8
cd ~/workspace/installRealSenseSDK
./buildLibrealsense.sh

echo "Compiling and installing ORB_SLAM3"
cd ~/workspace/ORB_SLAM3
./build.sh -j$(($NUM_CPU - 1))

echo "Building ROS2 modules"
cd ~/workspace/ros2_ws
source /opt/ros/foxy/setup.bash
colcon build
