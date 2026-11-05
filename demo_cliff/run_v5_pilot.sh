#!/usr/bin/env bash
# 판정 기준 v5 사전 시험 (10장). 비교 방식(P3 · P3m · P3n · P3i · P2i · P1ir)은 돌리지 않는다 — P1r 회차, 제동 시험, 지연 주입만.
# 고정 전에 돌린다. 시드 951–953 (확인 실험과 겹치지 않는다).
#   ./run_v5_pilot.sh                                   # 전부. 이미 있는 회차 · 설정값은 건너뛴다
#   PARTS="brake" BRAKE_ACCS="3.0 2.0 1.4 0.9" ./run_v5_pilot.sh   # 제동 시험만, 바퀴 설정값을 골라서
# 결과: $RES/v5_pilot/  → 채점 $RES/v5_pilot/check.md
cd "$(dirname "$0")"
RES=${RES:-results_$(hostname)}
P=$RES/v5_pilot
PARTS=${PARTS:-"delay brake runs"}
source env.sh
if [ -z "$DISPLAY" ] && [ -z "$WAYLAND_DISPLAY" ]; then
  echo "화면 변수(DISPLAY)가 없습니다. 없으면 Gazebo의 깊이카메라가 뜨지 않습니다 (11/05 스모크 시험)." >&2
  exit 1
fi
mkdir -p "$P"
plan() { python3 -c "import sys; sys.path.insert(0, 'tools'); import v5_plan as V; print($1)"; }

if grep -qw delay <<<"$PARTS"; then
  python3 tools/test_v5_delay.py 0.1 2>/dev/null | grep "지연" > "$P/delay.txt"
  cat "$P/delay.txt"
fi

if grep -qw brake <<<"$PARTS"; then
  ACCS=${BRAKE_ACCS:-$(plan "' '.join(str(a) for _, a in V.DECEL.values())")}
  WORLD2="$(ros2 pkg prefix cliff_demo)/share/cliff_demo/worlds/world2.world"
  mkdir -p "$P/brake"
  for acc in $ACCS; do
    out="$P/brake/acc_$acc.jsonl"
    [ -s "$out" ] && continue
    echo "▶ 제동 시험: 바퀴 설정 $acc"
    tools/kill_sim.sh
    CLIFF_WORLD=2 CLIFF_WHEEL_ACC=$acc ros2 launch cliff_demo sim.launch.py gui:=false world:="$WORLD2" > "$P/brake/launch_$acc.log" 2>&1 &
    CLIFF_WORLD=2 CLIFF_WHEEL_ACC=$acc timeout 180 python3 tools/brake_test.py --out "$out" --reps 3 > "$P/brake/test_$acc.log" 2>&1 \
      || echo "  → 제동 시험 실패 ($P/brake/test_$acc.log 확인)"
    tools/kill_sim.sh
    wait 2>/dev/null
  done
fi

if grep -qw runs <<<"$PARTS"; then
  export RING=true NOREPORT=1 CLIFF_WORLD=2 CLIFF_W2_MAT=$(plan "V.W2_MAT")
  run() { env "${@:2}" OUT="$P/$1" ./run_all.sh </dev/null; }
  run 1_s3 SEEDS="951 952" SCENS="S3 S3j" CONDS=P1r
  run 2_s8 SEEDS="951 952 953" SCENS=S8 CONDS=P1r
  run 3_s9 SEEDS="951 952" SCENS=S9 CONDS=P1r VMAX=$(plan "V.V_B")
fi

python3 tools/check_v5_pilot.py "$P" | tee "$P/check.md"
