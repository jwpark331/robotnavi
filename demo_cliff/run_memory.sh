#!/usr/bin/env bash
# 기억 시나리오(H1)와 개선안 P3h(H2) — 판정 기준: ~/robotnavi/보이지 않는 계단 — 판정 기준 (2026-10-20).md
#   ① H1: S7(계단 옆 목적지)·S8(계단 옆에서 뒤로 비켜 주기) × {P1 P1i P2 P3 P3i P3h} × 시드 1–10 = 120회
#        (계단 A keepout 누락이라 P0 = P1 → P0는 뺀다)
#   ② H2: P3h × 검은 계단(S3·S4, 결측 100 %) × 10 = 20회,  P3h × 보통(S1–S4) × 10 = 40회
#   결과: results/memory/, results/darkstair/void_1.0/ (P3h), results/p3h_normal/  → tools/analyze_memory.py
cd "$(dirname "$0")"
# 결과 폴더: 기본 results. 다른 컴퓨터 결과를 원래 결과와 섞지 않으려면 RES=results_<이름> 으로 따로 쌓는다
RES=${RES:-results}
export RESULTS_DIR="$PWD/$RES"     # 분석 스크립트(tools/analyze*.py)도 같은 폴더를 본다
source env.sh
SEEDS=${SEEDS:-"1 2 3 4 5 6 7 8 9 10"}
ROOTS="$RES/memory $RES/darkstair $RES/p3h_normal"
batch() {
  OUT=$RES/memory SCENS="S7 S8" CONDS="P1 P1i P2 P3 P3i P3h" SEEDS="$SEEDS" NOREPORT=1 ./run_all.sh
  OUT=$RES/darkstair/void_1.0 SCENS="S3 S4" CONDS="P3h" VOID=1.0 SEEDS="$SEEDS" NOREPORT=1 ./run_all.sh
  OUT=$RES/p3h_normal SCENS="S1 S2 S3 S4" CONDS="P3h" SEEDS="$SEEDS" NOREPORT=1 ./run_all.sh
}
batch
# 실험 장치 결함은 같은 시드로 다시 돌리고, 원본은 results/memory_faults/에 옮겨 보관한다 (최대 3번)
for pass in 1 2 3; do
  n=0; for r in $ROOTS; do n=$((n + $(python3 tools/find_faults.py $r | grep -c $'\t'))); done
  echo "== 정리 $pass회차: 결함 $n개"
  [ "$n" -eq 0 ] && break
  for r in $ROOTS; do
    python3 tools/find_faults.py $r | grep $'\t' | while IFS=$'\t' read -r d why; do
      dst="$RES/memory_faults/pass$pass/$(echo "${d#$RES/}" | tr / _)"
      mkdir -p "$(dirname "$dst")" && mv "$d" "$dst" && echo "$why" > "$dst/fault_reason.txt"
    done
  done
  batch
done
for r in $ROOTS; do python3 tools/find_faults.py $r; done
python3 tools/analyze_memory.py > $RES/memory/analysis.md && echo "→ $RES/memory/analysis.md"
