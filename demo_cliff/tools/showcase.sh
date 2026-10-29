#!/usr/bin/env bash
# 시나리오 결과를 Gazebo 창으로 다시 보여 주는 쇼케이스 (본 결과 results/runs는 건드리지 않는다 → results/showcase).
#   tools/showcase.sh            # 7장면 전부
#   tools/showcase.sh 3          # 3번 장면부터
# 같은 시드끼리 짝지어 보여 준다 (사람 위치·밀림 거리가 똑같고 방어 방식만 다름).
# 시뮬은 통신 타이밍에 따라 조금씩 달라서, 본 실험과 같은 결과가 안 나올 수도 있다 (본 실험은 10회 중 비율로 본다).
cd "$(dirname "$0")/.."
source env.sh
START=${1:-1}
SCENES=(
  "S2 P0 2|S2 계단 앞 사람들 · P0 SLAM 지도만|사람 4명이 모퉁이를 막고 있다. 지도에는 계단 A가 빈 바닥이라 그쪽으로 돌아가다 앞바퀴가 빠진다 (약 17초)."
  "S2 P3 2|S2 계단 앞 사람들 · P3 깊이카메라 베이즈 (제안)|같은 상황. 깊이카메라가 계단을 금지구역으로 넣어 돌아가지 않고, 사람들이 흩어진 뒤(45초) 지나간다."
  "S3 P1 8|S3 위치 착각 · P1 keepout|복도 중간에서 로봇이 앞으로 밀린다(바퀴는 모름). 로봇이 믿는 위치가 틀려 keepout도 어긋나고, 계단 A로 들어간다 (약 15초)."
  "S3 P3m 8|S3 위치 착각 · P3m 베이즈 (map 프레임 기억)|착각한 위치에서 본 가장자리가 지도에 틀린 자리로 저장된다. 위치가 바로잡힌 뒤 그 기억이 바닥 위 '유령 낭떠러지'가 되어 갇힌다 (처음 40초만 보면 충분)."
  "S3 P3 8|S3 위치 착각 · P3 베이즈 (odom 프레임 기억, 제안)|같은 밀림. 기억이 로봇과 함께 옮겨져 유령 낭떠러지 없이 지나간다."
  "S4 P1 3|S4 keepout 안 그린 계단 · P1 keepout|문 앞을 사람들이 막아 북쪽(계단 B 쪽)으로 돌아 들어가야 한다. 계단 B에는 keepout이 없어 떨어진다 (약 28초)."
  "S4 P3 3|S4 keepout 안 그린 계단 · P3 베이즈 (제안)|같은 상황. 깊이카메라가 계단 B를 보고 피해서 사무실에 도착한다."
)
i=0
for item in "${SCENES[@]}"; do
  i=$((i+1)); [ $i -lt $START ] && continue
  IFS='|' read -r run title note <<< "$item"
  read -r s c k <<< "$run"
  echo
  echo "════════════════════════════════════════════════════════"
  echo " 장면 $i/7  $title  (시드 $k)"
  echo " $note"
  echo "════════════════════════════════════════════════════════"
  rm -rf "results/showcase/${s}_${c}_${k}"
  OUT=results/showcase SCENS=$s CONDS=$c SEEDS=$k GUI=true NOREPORT=1 ./run_all.sh | grep "→"
  sleep 3
done
echo
echo "끝. 녹화 영상과 리플레이: results/showcase/*/video.mp4"
