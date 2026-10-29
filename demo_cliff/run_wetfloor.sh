#!/usr/bin/env bash
# 젖은 바닥 실험 (반박 ⑥⑫ "순간이동 밀림만 시험했다 / 누적 슬립은?" 확인)
#   계단 A 바로 앞 모퉁이(G.WET_PATCH)의 마찰계수(MU)를 낮춘다. 로봇이 감속·회전하는 곳이라 바퀴가 미끄러지고,
#   바퀴 인코더 odom이 조금씩 어긋난다(누적 슬립). 동시에 제동 거리도 길어진다.
#   Gazebo 접촉은 마찰 한계를 넘기 전엔 전혀 미끄러지지 않아서, 이 로봇의 가속 한계(1.0 m/s²·3.2 rad/s²)에서는
#   μ ≈ 0.07 아래부터 미끄러진다 (μ 0.1에서 방향 어긋남이 보통 바닥과 같았다). 그래서 0.05(비눗물·기름)와 0.02(얼음)를 쓴다.
#   ① S2(모퉁이를 사람이 막아 젖은 구간에서 기다리며 제자리 회전) × 6조건 × μ {0.02, 0.05} × 시드 1–10
#   ② 대조: P3i는 본 실험에 없으므로 μ 1.0(보통 바닥)에서 S2 × 시드 1–10
#   ③ S1(사람 없이 젖은 모퉁이를 돌아 지나감) × 6조건 × μ 0.02 × 시드 1–3
#   ④ μ 0.1 확인: S2 × P3 × 시드 1–3 (이 정도로는 안 미끄러진다는 근거)
#   μ 1.0의 P1·P1i·P2·P3m·P3는 본 실험(results/runs)과 같은 코드·월드라 다시 돌리지 않는다.
#   결과: results/wetfloor/mu_<값>/  → tools/analyze_wetfloor.py
cd "$(dirname "$0")"
# 결과 폴더: 기본 results. 다른 컴퓨터 결과를 원래 결과와 섞지 않으려면 RES=results_<이름> 으로 따로 쌓는다
RES=${RES:-results}
export RESULTS_DIR="$PWD/$RES"     # 분석 스크립트(tools/analyze*.py)도 같은 폴더를 본다
source env.sh
SEEDS=${SEEDS:-"1 2 3 4 5 6 7 8 9 10"}
SWEEP_SEEDS=${SWEEP_SEEDS:-"1 2 3"}
ALL="P1 P1i P2 P3m P3 P3i"
batch() {
  for m in 0.02 0.05; do
    OUT=$RES/wetfloor/mu_$m SCENS=S2 CONDS="$ALL" MU=$m SEEDS="$SEEDS" NOREPORT=1 ./run_all.sh
  done
  OUT=$RES/wetfloor/mu_1.0 SCENS=S2 CONDS="P3i" MU=1.0 SEEDS="$SEEDS" NOREPORT=1 ./run_all.sh
  OUT=$RES/wetfloor/mu_0.02 SCENS=S1 CONDS="$ALL" MU=0.02 SEEDS="$SWEEP_SEEDS" NOREPORT=1 ./run_all.sh
  OUT=$RES/wetfloor/mu_0.1 SCENS=S2 CONDS="P3" MU=0.1 SEEDS="$SWEEP_SEEDS" NOREPORT=1 ./run_all.sh
}
batch
# 실험 장치 결함은 같은 시드로 다시 돌리고, 원본은 results/wetfloor_faults/에 옮겨 보관한다 (최대 3번)
for pass in 1 2 3; do
  n=$(python3 tools/find_faults.py $RES/wetfloor | grep -c $'\t')
  echo "== 정리 $pass회차: 결함 $n개"
  [ "$n" -eq 0 ] && break
  python3 tools/find_faults.py $RES/wetfloor | grep $'\t' | while IFS=$'\t' read -r d why; do
    dst="$RES/wetfloor_faults/pass$pass/$(echo "${d#$RES/wetfloor/}" | tr / _)"
    mkdir -p "$(dirname "$dst")" && mv "$d" "$dst" && echo "$why" > "$dst/fault_reason.txt"
  done
  batch
done
python3 tools/find_faults.py $RES/wetfloor
python3 tools/analyze_wetfloor.py > $RES/wetfloor/analysis.md && echo "→ $RES/wetfloor/analysis.md"
