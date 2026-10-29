#!/usr/bin/env bash
# 검은 계단 실험 (반박 ① "계단이 검은색이라 깊이값이 안 나오면 P3는 계단을 못 본다" 확인)
#   계단 구멍 안으로 들어간 깊이카메라 광선이 반사를 못 받는다(결측)고 보고, 그 비율(VOID)을 바꾼다.
#   ① 결측 100 % × S3·S4 × {P2 단일 프레임, P3 제안, P3i 제안+하향 IR} × 시드 1–10
#      S3(keepout 어긋남)·S4(keepout 없음)는 깊이카메라만이 계단을 막을 수 있는 상황이다.
#   ② sweep: 결측 50·70·90 % × S3·S4 × P3 × 시드 1–3 → 몇 %부터 무너지나
#   ③ 대조: 결측 0 % × S3·S4 × P3i × 시드 1–10 → IR을 더한 대가 (보통 계단에서 손해가 있나)
#   P0·P1·P1i는 깊이카메라를 안 쓰고, P2·P3의 결측 0 %는 본 실험(results/runs)과 같은 코드·설정이라 다시 돌리지 않는다.
#   결과: results/darkstair/void_<값>/  → tools/analyze_darkstair.py
cd "$(dirname "$0")"
# 결과 폴더: 기본 results. 다른 컴퓨터 결과를 원래 결과와 섞지 않으려면 RES=results_<이름> 으로 따로 쌓는다
RES=${RES:-results}
export RESULTS_DIR="$PWD/$RES"     # 분석 스크립트(tools/analyze*.py)도 같은 폴더를 본다
source env.sh
SEEDS=${SEEDS:-"1 2 3 4 5 6 7 8 9 10"}
SWEEP_SEEDS=${SWEEP_SEEDS:-"1 2 3"}
batch() {
  OUT=$RES/darkstair/void_1.0 SCENS="S3 S4" CONDS="P2 P3 P3i" VOID=1.0 SEEDS="$SEEDS" NOREPORT=1 ./run_all.sh
  for v in 0.5 0.7 0.9; do
    OUT=$RES/darkstair/void_$v SCENS="S3 S4" CONDS="P3" VOID=$v SEEDS="$SWEEP_SEEDS" NOREPORT=1 ./run_all.sh
  done
  OUT=$RES/darkstair/void_0.0 SCENS="S3 S4" CONDS="P3i" VOID=0.0 SEEDS="$SEEDS" NOREPORT=1 ./run_all.sh
}
batch
# 실험 장치 결함(컨트롤러 TF 멈춤·가짜 도착·결과 없음)은 같은 시드로 다시 돌린다 (본 실험과 같은 기준, 최대 3번)
# 결함 회차는 지우지 않고 results/darkstair_faults/로 옮겨 둔다 (나중에 "골라낸 것 아니냐"에 원본으로 답하기 위해)
for pass in 1 2 3; do
  n=$(python3 tools/find_faults.py $RES/darkstair | grep -c $'\t')
  echo "== 정리 $pass회차: 결함 $n개"
  [ "$n" -eq 0 ] && break
  python3 tools/find_faults.py $RES/darkstair | grep $'\t' | while IFS=$'\t' read -r d why; do
    dst="$RES/darkstair_faults/pass$pass/$(echo "${d#$RES/darkstair/}" | tr / _)"
    mkdir -p "$(dirname "$dst")" && mv "$d" "$dst" && echo "$why" > "$dst/fault_reason.txt"
  done
  batch
done
python3 tools/find_faults.py $RES/darkstair
python3 tools/analyze_darkstair.py > $RES/darkstair/analysis.md && echo "→ $RES/darkstair/analysis.md"
