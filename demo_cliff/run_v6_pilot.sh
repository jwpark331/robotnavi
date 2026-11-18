#!/usr/bin/env bash
# 판정 기준 v6 사전 시험 (8장). 고정 전에 돈다. 판정에 쓰지 않는다. 시드 591 (확인 실험 501–520과 겹치지 않는다).
#   ./run_v6_pilot.sh                 # 전부: 단위 시험 → 월드 1 · 2 후진 스모크 → 평상시 스모크
#   PARTS="unit" ./run_v6_pilot.sh    # 단위 시험만 (시뮬레이션 없음)
# 결과: $RES/v6_pilot/  (회차 폴더 + unit.md + analysis.md)
cd "$(dirname "$0")"
RES=${RES:-results_$(hostname)}
P=${PILOT_DIR:-$RES/v6_pilot}
PARTS=${PARTS:-"unit backup normal"}
SEED=${SEED:-591}
source env.sh
mkdir -p "$P"
if grep -qw unit <<<"$PARTS"; then
  python3 tools/test_v6_backup.py | tee "$P/unit.md"
fi
if [ -z "$DISPLAY" ] && [ -z "$WAYLAND_DISPLAY" ] && grep -qE "backup|normal" <<<"$PARTS"; then
  echo "화면 변수(DISPLAY)가 없습니다. 없으면 Gazebo의 깊이카메라가 뜨지 않습니다 (11/05 스모크 시험)." >&2
  exit 1
fi
W2_MAT=$(python3 -c "import sys; sys.path.insert(0, 'tools'); import v6_plan; print(v6_plan.W2_MAT)")
if grep -qw backup <<<"$PARTS"; then
  for world in 1 2; do for yd in 0.55 1.10; do
    g="Y-w$world-d$(python3 -c "print(f'{int(round($yd*100)):03d}')")"
    mat=""; [ "$world" = 2 ] && mat=$W2_MAT
    CLIFF_WORLD=$world CLIFF_W2_MAT=$mat CLIFF_YIELD_D=$yd RING=true OUT=$P/$g SCENS=S8 \
      CONDS="${PILOT_CONDS:-P3y P3 P3n P1irb P2nav}" SEEDS=$SEED NOREPORT=1 ./run_all.sh </dev/null
  done; done
fi
if grep -qw normal <<<"$PARTS"; then
  CLIFF_WORLD=1 RING=true OUT=$P/N SCENS="S1 S2" CONDS="P3 P2nav" SEEDS=$SEED NOREPORT=1 ./run_all.sh </dev/null
fi
python3 tools/analyze_v6.py "$P" > "$P/analysis.md" 2>&1 && echo "→ $P/analysis.md (사전 시험: 판정에 쓰지 않는다)"
