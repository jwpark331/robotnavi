#!/usr/bin/env bash
# 판정 기준 v4 사전 시험 (8장, 30회). 시드 901–903. 가설 칸은 돌리지 않는다. 고정 전에 돌린다.
#   ./run_v4_pilot.sh           # 이미 result.json이 있는 회차는 건너뛴다
# 결과: $RES/v4_pilot/<번호>_<이름>/<시나리오>_<방식>_<시드>/  → 통과 기준 채점 $RES/v4_pilot/check.md
cd "$(dirname "$0")"
RES=${RES:-results_$(hostname)}
P=$RES/v4_pilot
S="901 902 903"
source env.sh
export RING=true NOREPORT=1 SEEDS="$S"
run() { env "${@:2}" OUT="$P/$1" ./run_all.sh </dev/null; }
run 1_p3p        VMAX=0.8 SCENS=S1 CONDS=P3p
run 2_p3p_void   VMAX=0.8 SCENS=S1 CONDS=P3p VOID=1.0
run 3_camkill    VMAX=0.8 SCENS=S1 CONDS="P3i P3p" CAMKILL=5.0
run 4_ir30       VMAX=0.8 SCENS=S1 CONDS=P1ir CLIFF_IR_R=0.30
run 5_straight   VMAX=0.8 SCENS=S9 CONDS=P0
run 6_straight_ir VMAX=0.2 SCENS=S9 CONDS=P1ir
run 7_p2i        VMAX=0.8 SCENS=S1 CONDS=P2i
run 8_a_speed    VMAX=0.8 SCENS=S4 CONDS=P2
run 9_person     VMAX=0.8 SCENS=S5 CONDS=P1r
python3 tools/check_v4_pilot.py "$P" | tee "$P/check.md"
