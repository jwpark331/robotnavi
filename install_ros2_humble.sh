#!/usr/bin/env bash
# ROS 2 Humble 설치 (Ubuntu 22.04, 공식 문서 기준 deb 패키지)
set -euo pipefail

# 1. Universe 저장소 활성화
sudo apt update
sudo apt install -y software-properties-common curl
sudo add-apt-repository -y universe

# 2. ROS 2 apt 소스 설정 (ros2-apt-source 패키지)
ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

# 3. 시스템 업그레이드 후 ROS 2 Humble Desktop + 개발 도구 설치
sudo apt update
sudo apt upgrade -y
sudo apt install -y ros-humble-desktop ros-dev-tools

# 4. 환경 설정을 ~/.bashrc에 추가 (중복 방지)
grep -qxF 'source /opt/ros/humble/setup.bash' ~/.bashrc || echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc

# 5. rosdep 초기화
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init
fi
rosdep update

echo "ROS 2 Humble 설치 완료. 새 터미널을 열거나 'source ~/.bashrc' 실행하세요."
