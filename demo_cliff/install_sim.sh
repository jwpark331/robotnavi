#!/usr/bin/env bash
# 계단 추락 데모용 패키지: Gazebo Classic 11 + gazebo_ros, Nav2, slam_toolbox (ROS 2 Humble, Ubuntu 22.04)
set -eo pipefail

if uname -r | grep -q cip; then
  echo "EVL 실시간 커널($(uname -r))입니다. Gazebo는 generic 커널에서 돌리세요: sudo grub-reboot \"1>2\" 후 재부팅" >&2
  exit 1
fi

sudo apt update
sudo apt install -y \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-nav2-simple-commander \
  ros-humble-slam-toolbox \
  ros-humble-rmw-cyclonedds-cpp \
  ros-humble-xacro ros-humble-teleop-twist-keyboard \
  ffmpeg

echo
echo "설치 완료. 확인:"
source /opt/ros/humble/setup.bash
ros2 pkg list | grep -E '^(gazebo_ros|nav2_bringup|slam_toolbox|nav2_simple_commander)$'
