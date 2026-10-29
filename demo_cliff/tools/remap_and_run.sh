#!/usr/bin/env bash
# 월드가 바뀌었을 때: 월드 SDF 재생성 → slam_toolbox 매핑 → keepout 마스크 → 빌드 → 전체 회차
cd "$(dirname "$0")/.."
source env.sh
set -e
tools/kill_sim.sh
(cd ws/src/cliff_demo && python3 -m cliff_demo.make_world worlds/city_hall.world && rm -f maps/*)
(cd ws && colcon build --symlink-install 2>&1 | tail -1)
echo "▶ 매핑 (slam_toolbox)"
timeout 600 ros2 launch cliff_demo mapping.launch.py map_path:="$PWD/ws/src/cliff_demo/maps/city_hall" > results/mapping.log 2>&1 || true
tools/kill_sim.sh
(cd ws/src/cliff_demo && python3 -m cliff_demo.make_masks maps)
(cd ws && colcon build --symlink-install 2>&1 | tail -1)
set +e
rm -rf results/runs
./run_all.sh
