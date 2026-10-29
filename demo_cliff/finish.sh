#!/usr/bin/env bash
# 배치가 끝난 뒤: 실험 장치 결함 회차(컨트롤러 TF 멈춤·가짜 도착·결과 없음)를 지우고 같은 시드로 다시 돌린다.
# 결함이 없어질 때까지 최대 5번. 같은 시드 = 같은 사람 위치·밀림 거리라서 대체해도 추정이 치우치지 않는다.
cd "$(dirname "$0")"
# 결과 폴더: 기본 results. 다른 컴퓨터 결과를 원래 결과와 섞지 않으려면 RES=results_<이름> 으로 따로 쌓는다
RES=${RES:-results}
export RESULTS_DIR="$PWD/$RES"     # 분석 스크립트(tools/analyze*.py)도 같은 폴더를 본다
source env.sh
SEEDS=${SEEDS:-"1 2 3 4 5 6 7 8 9 10"}
SWEEP_SEEDS=${SWEEP_SEEDS:-"1 2 3"}
for pass in 1 2 3 4 5; do
  n=$(( $(python3 tools/find_faults.py $RES/runs | grep -c $'\t') + $(python3 tools/find_faults.py $RES/sweep | grep -c $'\t') ))
  echo "== 정리 $pass회차: 결함 $n개"
  [ "$n" -eq 0 ] && break
  python3 tools/find_faults.py $RES/runs --delete >/dev/null
  python3 tools/find_faults.py $RES/sweep --delete >/dev/null
  SEEDS="$SEEDS" NOREPORT=1 ./run_all.sh
  SEEDS="$SWEEP_SEEDS" ./run_sweep.sh
done
python3 tools/find_faults.py $RES/runs
python3 tools/find_faults.py $RES/sweep
python3 tools/make_report.py $RES/runs $RES/report.html
python3 tools/analyze.py > $RES/analysis.md
python3 tools/figures.py
