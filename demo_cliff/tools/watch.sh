#!/usr/bin/env bash
# 돌고 있는 배치를 실시간으로 보기 — Gazebo 화면(gzclient) + RViz(로봇이 믿는 것: 지도·keepout·낭떠러지 마스크·경로·AMCL 파티클).
#   tools/watch.sh          # 둘 다
#   tools/watch.sh gazebo   # Gazebo만
#   tools/watch.sh rviz     # RViz만
# 회차가 바뀌면 배치가 Gazebo를 다시 띄우므로 gzclient는 자동으로 다시 붙는다. 끌 때는 RViz 창을 닫거나 이 탭에서 Ctrl-C.
# 화면을 띄우면 그래픽 부하가 늘어 시뮬 속도가 조금 느려질 수 있다 (결과는 시뮬 시간 기준이라 영향은 작다).
cd "$(dirname "$0")/.."
source env.sh
MODE=${1:-both}
trap 'kill $(jobs -p) 2>/dev/null; exit 0' INT TERM
if [ "$MODE" != rviz ]; then
  ( while true; do gzclient >/dev/null 2>&1; sleep 3; done ) &
fi
if [ "$MODE" != gazebo ]; then
  ros2 run rviz2 rviz2 -d tools/cliff_view.rviz --ros-args -p use_sim_time:=true >/dev/null 2>&1   # RViz 창을 닫으면 끝
  kill $(jobs -p) 2>/dev/null; pkill -x gzclient
else
  wait
fi
