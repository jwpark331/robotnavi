#!/usr/bin/env bash
# 새 노트북 준비: Ubuntu 22.04 + ROS 2 Humble 위에 필요한 패키지를 깔고 이 워크스페이스를 빌드한다.
#   bash setup_new_machine.sh      (sudo 암호를 물어본다)
# ROS 2 Humble 자체가 없으면 먼저: bash ../install_ros2_humble.sh
set -eo pipefail
cd "$(dirname "$0")"

. /etc/os-release
if [ "$VERSION_ID" != "22.04" ]; then
  echo "Ubuntu 22.04가 필요합니다 (지금: $PRETTY_NAME). ROS 2 Humble + Gazebo Classic 11 조합은 22.04 기준입니다." >&2
  exit 1
fi
if uname -r | grep -q cip; then
  echo "주의: EVL 실시간 커널($(uname -r))입니다. Gazebo는 generic 커널에서 돌리세요 (run_all.sh가 실행을 막습니다)." >&2
fi
if [ ! -f /opt/ros/humble/setup.bash ]; then
  echo "ROS 2 Humble이 없습니다. 먼저 'bash ../install_ros2_humble.sh'를 실행한 뒤 다시 실행하세요." >&2
  exit 1
fi

sudo apt update
sudo apt install -y \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-nav2-simple-commander \
  ros-humble-slam-toolbox \
  ros-humble-rmw-cyclonedds-cpp \
  ros-humble-xacro ros-humble-robot-state-publisher ros-humble-rviz2 \
  python3-colcon-common-extensions python3-numpy python3-pil python3-yaml python3-matplotlib \
  ffmpeg fonts-noto-cjk mesa-utils pciutils

# 빌드 (build/install/log는 저장소에 없다 — 기계마다 새로 만든다)
set +u
source /opt/ros/humble/setup.bash
(cd ws && colcon build --symlink-install)
source env.sh
ros2 pkg prefix cliff_demo >/dev/null && echo "cliff_demo 빌드 확인"

mkdir -p results
tools/record_env.sh "results/MACHINE_$(hostname).txt"
echo
cat "results/MACHINE_$(hostname).txt"
echo
echo "준비 완료. 다음 순서:"
echo "  1) 화면으로 한 번 보기:   GUI=true OUT=results/smoke_$(hostname) SCENS=S2 CONDS=P0 NOREPORT=1 ./run_all.sh"
echo "  2) 기존 결과 재현 확인:   ./run_calibration.sh   (18회, 약 30분)"
