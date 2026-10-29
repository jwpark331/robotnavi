# source demo_cliff/env.sh — ROS 2 Humble + 이 워크스페이스. ROS의 numpy 1.x를 쓰도록 사용자 site-packages(numpy 2)는 끈다.
source /opt/ros/humble/setup.bash
source "$(dirname "${BASH_SOURCE[0]}")/ws/install/setup.bash"
export PYTHONNOUSERSITE=1
export RCUTILS_COLORIZED_OUTPUT=0
# 통신 미들웨어: Cyclone DDS. Humble 기본(Fast DDS)에서 Nav2 lifecycle 서비스 응답 유실·한 노드만 TF를 못 받는 문제가
# 회차의 5–10 %에서 났다 (Nav2도 Humble에서 Cyclone을 권장). 설치돼 있을 때만 쓴다.
if [ -f /opt/ros/humble/lib/librmw_cyclonedds_cpp.so ]; then
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
fi
# 통신은 이 컴퓨터 안(루프백)으로만 — Cyclone 설정 파일로 지정한다 (ROS_LOCALHOST_ONLY는 참가자 상한 9에 걸려 실패).
export CYCLONEDDS_URI="file://$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/cyclonedds.xml"
unset ROS_LOCALHOST_ONLY
