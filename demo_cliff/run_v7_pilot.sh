#!/usr/bin/env bash
# 판정 기준 v7 사전 시험 (고정 전, 6회): 월드 1 · s = 0.90 · 1.00 · 1.10 · 시드 991–992 (기준 v7 8장).
# 비교 지표를 보고 설계를 바꾸지 않는다. 확인하는 것은 성립 조건(odom 배율이 실제로 들어갔나)과 장치 결함뿐.
#   ./run_v7_pilot.sh     → $RES/v7_pilot/  (analyze_v7.py로 점검)
cd "$(dirname "$0")"
RES=${RES:-results_$(hostname)}
P=$RES/v7_pilot
source env.sh
if [ -z "$DISPLAY" ] && [ -z "$WAYLAND_DISPLAY" ]; then
  echo "화면 변수(DISPLAY)가 없습니다. 없으면 Gazebo의 깊이카메라가 뜨지 않습니다 (11/05 스모크 시험)." >&2
  exit 1
fi
for sc in 0.90 1.00 1.10; do
  g="O-w1-s$(python3 -c "print(f'{int(round($sc * 100)):03d}')")"
  for k in 991 992; do
    [ -f "$P/$g/S8_P3y_$k/result.json" ] && continue
    CLIFF_WORLD=1 CLIFF_YIELD_D=0.55 CLIFF_ODOM_SCALE=$sc RING=true OUT=$P/$g SCENS=S8 \
      CONDS=P3y SEEDS=$k NOREPORT=1 ./run_all.sh </dev/null
  done
done
python3 tools/analyze_v7.py "$P" > "$P/check.md" && echo "→ $P/check.md"
