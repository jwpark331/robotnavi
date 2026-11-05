#!/usr/bin/env bash
# 판정 기준 v5 확인 실험 (400회, 모두 월드 2). 계획은 tools/v5_plan.py 한 곳에 있다.
#   ./run_v5.sh                      # 전부. 이미 result.json이 있는 회차는 건너뛰므로 끊겨도 다시 실행하면 이어서 돈다
#   ONLY_GROUPS="WS3 WS8" ./run_v5.sh  # 일부 묶음만
# - 고정 전에는 돌지 않는다: 고정 파일 해시가 없거나 맞지 않으면 멈춘다 (기준 문서 9장 7번).
# - 모든 방식의 운영자 keepout은 비용 띠 마스크(RING=true). 월드 2 매트는 v3 H12 자리(v5_plan.W2_MAT).
# - 순서: 시드 하나를 블록으로, 블록 안에서 (묶음, 시나리오, 방식)을 시드로 섞는다.
# - 장치 결함은 같은 시드로 다시 돌리고(최대 2번 더) 원본은 $RES/v5_faults/에 보관. 가짜 도착은 결함이 아니라 미도착으로 센다.
# 결과: $RES/v5/<묶음>/<시나리오>_<방식>_<시드>/  → $RES/v5/analysis.md
cd "$(dirname "$0")"
RES=${RES:-results_$(hostname)}
ONLY=${ONLY_GROUPS:-}
if ! (cd .. && sha256sum --quiet -c "demo_cliff/$RES/v5_frozen.sha256"); then
  echo "고정 파일 해시가 없거나 맞지 않습니다: $RES/v5_frozen.sha256 (기준 문서 9장 7번)" >&2
  exit 1
fi
source env.sh
if [ -z "$DISPLAY" ] && [ -z "$WAYLAND_DISPLAY" ]; then
  echo "화면 변수(DISPLAY)가 없습니다. 없으면 Gazebo의 깊이카메라가 뜨지 않습니다 (11/05 스모크 시험)." >&2
  exit 1
fi
W2_MAT=$(python3 -c "import sys; sys.path.insert(0, 'tools'); import v5_plan; print(v5_plan.W2_MAT)")
echo "$(date '+%F %T') 시작" >> "$RES/v5_run.log"
batch() {
  python3 tools/v5_plan.py order | while IFS=$'\t' read -r g s c k world vmax ir_r acc delay; do
    [ -n "$ONLY" ] && ! grep -qw -- "$g" <<<"$ONLY" && continue
    [ -f "$RES/v5/$g/${s}_${c}_${k}/result.json" ] && continue
    echo "$(date '+%F %T') $g $s $c $k" >> "$RES/v5_run.log"
    CLIFF_WORLD=$world CLIFF_W2_MAT=$W2_MAT CLIFF_IR_R=$ir_r CLIFF_WHEEL_ACC=$acc CLIFF_SENSE_DELAY=$delay \
      VMAX=$vmax RING=true OUT=$RES/v5/$g SCENS=$s CONDS=$c SEEDS=$k NOREPORT=1 ./run_all.sh </dev/null
  done
}
batch
for pass in 1 2; do
  n=0
  for g in $(ls "$RES/v5" 2>/dev/null); do
    [ -d "$RES/v5/$g" ] || continue
    while IFS=$'\t' read -r d why; do
      [ -z "$d" ] && continue
      dst="$RES/v5_faults/pass$pass/${g}_$(basename "$d")"
      mkdir -p "$(dirname "$dst")" && mv "$d" "$dst" && echo "$why" > "$dst/fault_reason.txt" && n=$((n + 1))
    done < <(python3 tools/analyze_v5.py --faults "$RES/v5/$g")
  done
  echo "== 결함 정리 $pass회차: $n개"
  [ "$n" -eq 0 ] && break
  batch
done
echo "$(date '+%F %T') 끝" >> "$RES/v5_run.log"
python3 tools/analyze_v5.py "$RES/v5" > "$RES/v5/analysis.md" && echo "→ $RES/v5/analysis.md"
