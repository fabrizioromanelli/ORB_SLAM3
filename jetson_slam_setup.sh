#!/bin/bash
NUM_CPU=$(nproc)

mkdir ~/workspace
cd ~/workspace

echo "Cloning and building opencv project"
git clone git@github.com:opencv/opencv.git -b 3.4.13
mkdir -p opencv/build
cd opencv/build
cmake ..
sudo make install -j$(($NUM_CPU - 1))

echo "Cloning realsense and ORB_SLAM3 projects"
cd ~/workspace
git clone git@github.com:fabrizioromanelli/installRealSenseSDK.git
git clone git@github.com:fabrizioromanelli/ORB_SLAM3.git -b development
git clone git@github.com:stevenlovegrove/Pangolin.git
mkdir -p ~/workspace/ros2_ws/src
cd ~/workspace/ros2_ws/src
echo "Cloning realsense and ORB_SLAM3 projects for ROS2"
git clone git@github.com:fabrizioromanelli/realsense-ros.git -b foxy
git clone git@github.com:fabrizioromanelli/orbslam3-ros2.git

echo "Compiling and installing realsense2 SDK. Please use gcc-8 and g++-8"
sudo apt install g++-8 -y
cd ~/workspace/installRealSenseSDK
./buildLibrealsense.sh

echo "Compiling and installing Pangolin"
cd ~/workspace/Pangolin
mkdir -p ~/workspace/Pangolin/build
cd ~/workspace/Pangolin/build
cmake ..
sudo make install -j$(($NUM_CPU - 1))

echo "Compiling and installing ORB_SLAM3"
cd ~/workspace/ORB_SLAM3
./build.sh -j$(($NUM_CPU - 1))

echo "Building ROS2 modules"
cd ~/workspace/ros2_ws
source /opt/ros/foxy/setup.bash
colcon build
