#!/usr/bin/env bash
# 남은 Gazebo / Nav2 / 데모 노드 정리.
# 리눅스 프로세스 이름(comm)은 15글자에서 잘린다 (controller_server → controller_serv). 그래서 이름(-x)이 아니라
# 실행 파일 경로(-f)로 끈다. 끊긴 회차의 노드가 남으면 다음 회차에 같은 이름으로 섞여 lifecycle 설정이 멈추거나
# 옛 시각의 TF가 섞인다 ('Transform data too old').
# 주의: 이 파일의 패턴을 같은 명령줄에 쓰면 pkill -f가 그 셸까지 끈다 — 항상 이 스크립트로만 호출한다.
for p in gzserver gzclient; do pkill -x "$p"; done
PATS=(
  "/opt/ros/humble/bin/ros2 launch cliff_demo"
  "/opt/ros/humble/lib/nav2_"
  "/opt/ros/humble/lib/robot_state_publisher/"
  "/opt/ros/humble/lib/slam_toolbox/"
  "/opt/ros/humble/lib/gazebo_ros/"
  "cliff_demo/lib/cliff_demo/"
)
for pat in "${PATS[@]}"; do pkill -f -- "$pat"; done
sleep 1
for p in gzserver gzclient; do pkill -9 -x "$p"; done
for pat in "${PATS[@]}"; do pkill -9 -f -- "$pat"; done
exit 0
