#!/usr/bin/env bash
# 피드백 1(팀원 리뷰, 2026-10-19)의 치명적 약점 3개에 대한 방어 실험 — 판정 기준은 '판정 기준 (2026-10-20)'과 같다
#   §2-1 값 안 나오는 낭떠러지 → P3+ (3상태·속도 제한·IR 기억·후진)
#        검은 계단 S3·S4 × 10 = 20, 보통 S1–S4 × 10 = 40, 뒤로 비켜 주기 S8 × 10 = 10
#   §2-3 필터와 기억이 같이 바뀜 → 2×2: P2o(단일 프레임 + 기억), P3n(필터 + 기억 없음)
#        S1·S2 × 10 = 40 (+ S8은 위 §2-1 묶음에서 같이 돌린다 = 20)
#   §2-2 상관된 오차(가속할 때 차체 기울기) → 가감속 연동 기울기 3°/(m/s²) 주입
#        (피드백의 1.5°는 최대 가속 0.9 m/s²에서 3 m 앞 오차 7.1 cm로 판정 기준 8 cm 아래 → 시험에서 효과 없음. 서스펜션이 무른 경우의 스트레스 값)
#        S1·S2 × {P2, P3, P3+} × 10 = 60
#   결과: results/fb1/{dark,normal,s8,factorial,pitch}/  → tools/analyze_fb1.py
cd "$(dirname "$0")"
# 결과 폴더: 기본 results. 다른 컴퓨터 결과를 원래 결과와 섞지 않으려면 RES=results_<이름> 으로 따로 쌓는다
RES=${RES:-results}
export RESULTS_DIR="$PWD/$RES"     # 분석 스크립트(tools/analyze*.py)도 같은 폴더를 본다
source env.sh
SEEDS=${SEEDS:-"1 2 3 4 5 6 7 8 9 10"}
batch() {
  OUT=$RES/fb1/dark SCENS="S3 S4" CONDS="P3p" VOID=1.0 SEEDS="$SEEDS" NOREPORT=1 ./run_all.sh
  OUT=$RES/fb1/s8 SCENS="S8" CONDS="P3p P2o P3n" SEEDS="$SEEDS" NOREPORT=1 ./run_all.sh
  OUT=$RES/fb1/factorial SCENS="S1 S2" CONDS="P2o P3n" SEEDS="$SEEDS" NOREPORT=1 ./run_all.sh
  OUT=$RES/fb1/pitch SCENS="S1 S2" CONDS="P2 P3 P3p" PITCH=3.0 SEEDS="$SEEDS" NOREPORT=1 ./run_all.sh
  OUT=$RES/fb1/normal SCENS="S1 S2 S3 S4" CONDS="P3p" SEEDS="$SEEDS" NOREPORT=1 ./run_all.sh
}
batch
# 실험 장치 결함은 같은 시드로 다시 돌리고, 원본은 results/fb1_faults/에 옮겨 보관한다 (최대 3번)
for pass in 1 2 3; do
  n=$(python3 tools/find_faults.py $RES/fb1 | grep -c $'\t')
  echo "== 정리 $pass회차: 결함 $n개"
  [ "$n" -eq 0 ] && break
  python3 tools/find_faults.py $RES/fb1 | grep $'\t' | while IFS=$'\t' read -r d why; do
    dst="$RES/fb1_faults/pass$pass/$(echo "${d#$RES/fb1/}" | tr / _)"
    mkdir -p "$(dirname "$dst")" && mv "$d" "$dst" && echo "$why" > "$dst/fault_reason.txt"
  done
  batch
done
python3 tools/find_faults.py $RES/fb1
python3 tools/analyze_fb1.py > $RES/fb1/analysis.md && echo "→ $RES/fb1/analysis.md"
