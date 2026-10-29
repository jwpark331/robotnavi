#!/usr/bin/env bash
# 움직이는 사람 실험 (반박 ⑫ "사람이 서 있는 원기둥뿐이다" 대응)
#   서 있는 무리(S2·S4) 대신 6명이 시드마다 무작위 위치·시각에 나타나 복도·모퉁이·사무실 문 사이를 걸어 다닌다
#   (cliff_demo/walkers.py: 계단·벽은 피해 걷고, 로봇 앞에선 멈춰 비켜 주고, 모퉁이 등에선 잠깐 서 있다가 사라졌다 다시 나타남).
#   S5: 계단 A·B 모두 keepout,  S6: 계단 B keepout 누락 (S4와 같은 운영 실수)
#   S5·S6 × 7조건(P0 P1 P1i P2 P3m P3 P3i) × 시드 1–10 = 140회
#   결과: results/walkers/  → tools/analyze_walkers.py
cd "$(dirname "$0")"
# 결과 폴더: 기본 results. 다른 컴퓨터 결과를 원래 결과와 섞지 않으려면 RES=results_<이름> 으로 따로 쌓는다
RES=${RES:-results}
export RESULTS_DIR="$PWD/$RES"     # 분석 스크립트(tools/analyze*.py)도 같은 폴더를 본다
source env.sh
SEEDS=${SEEDS:-"1 2 3 4 5 6 7 8 9 10"}
batch() {
  OUT=$RES/walkers SCENS="S5 S6" CONDS="P0 P1 P1i P2 P3m P3 P3i" SEEDS="$SEEDS" NOREPORT=1 ./run_all.sh
}
batch
# 실험 장치 결함은 같은 시드로 다시 돌리고, 원본은 results/walkers_faults/에 옮겨 보관한다 (최대 3번)
for pass in 1 2 3; do
  n=$(python3 tools/find_faults.py $RES/walkers | grep -c $'\t')
  echo "== 정리 $pass회차: 결함 $n개"
  [ "$n" -eq 0 ] && break
  python3 tools/find_faults.py $RES/walkers | grep $'\t' | while IFS=$'\t' read -r d why; do
    dst="$RES/walkers_faults/pass$pass/$(basename "$d")"
    mkdir -p "$(dirname "$dst")" && mv "$d" "$dst" && echo "$why" > "$dst/fault_reason.txt"
  done
  batch
done
python3 tools/find_faults.py $RES/walkers
python3 tools/analyze_walkers.py > $RES/walkers/analysis.md && echo "→ $RES/walkers/analysis.md"
