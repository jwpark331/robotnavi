#!/usr/bin/env bash
# 견고성 sweep (반론 2 대응: "값을 맞춘 것 아니냐")
#   ① 밀림 거리: S3 × {P1 keepout, P1i IR, P3 제안} × 0.0–1.2 m → keepout은 위치 오차 몇 cm까지 버티나
#   ② 매트 결측률: S1 × {P2 단일 프레임, P3 베이즈} × 0–90 % → 결측이 늘어도 베이즈는 버티나
#   결과: results/sweep/push_<값>/, results/sweep/dropout_<값>/  → 리포트(results/report.html)의 '견고성' 절
cd "$(dirname "$0")"
# 결과 폴더: 기본 results. 다른 컴퓨터 결과를 원래 결과와 섞지 않으려면 RES=results_<이름> 으로 따로 쌓는다
RES=${RES:-results}
export RESULTS_DIR="$PWD/$RES"     # 분석 스크립트(tools/analyze*.py)도 같은 폴더를 본다
SEEDS=${SEEDS:-"1 2 3"}
for v in 0.0 0.2 0.4 0.6 0.8 1.0 1.2; do
  OUT=$RES/sweep/push_$v SCENS=S3 CONDS="P1 P1i P3" PUSH=$v SEEDS="$SEEDS" NOREPORT=1 ./run_all.sh
done
for v in 0.0 0.5 0.75 0.9; do
  OUT=$RES/sweep/dropout_$v SCENS=S1 CONDS="P2 P3" DROPOUT=$v SEEDS="$SEEDS" NOREPORT=1 ./run_all.sh
done
python3 tools/make_report.py $RES/runs $RES/report.html
