#!/usr/bin/env bash
# 판정 기준 v7 확인 실험 (odom 배율 오차, 280회). 계획은 tools/v7_plan.py 한 곳에 있다.
#   ./run_v7.sh                        # 전부. 이미 result.json이 있는 회차는 건너뛰므로 끊겨도 다시 실행하면 이어서 돈다
#   ONLY_GROUPS="O-w1-s090" ./run_v7.sh
# - 고정 전에는 돌지 않는다: 고정 파일 해시가 없거나 맞지 않으면 멈춘다 (기준 v7 11장).
# - 운영자 keepout은 v6와 같이 비용 띠 마스크(RING=true). 월드 2 매트는 v5 · v6와 같은 자리.
# - 장치 결함은 같은 시드로 다시 돌리고(최대 2번 더) 원본은 $RES/v7_faults/에 보관 (v6와 같은 규칙).
# 결과: $RES/v7/<묶음>/S8_P3y_<시드>/  → $RES/v7/analysis.md
cd "$(dirname "$0")"
RES=${RES:-results_$(hostname)}
ONLY=${ONLY_GROUPS:-}
if ! (cd .. && sha256sum --quiet -c "demo_cliff/$RES/v7_frozen.sha256"); then
  echo "고정 파일 해시가 없거나 맞지 않습니다: $RES/v7_frozen.sha256 (기준 v7 11장)" >&2
  exit 1
fi
source env.sh
if [ -z "$DISPLAY" ] && [ -z "$WAYLAND_DISPLAY" ]; then
  echo "화면 변수(DISPLAY)가 없습니다. 없으면 Gazebo의 깊이카메라가 뜨지 않습니다 (11/05 스모크 시험)." >&2
  exit 1
fi
W2_MAT=$(python3 -c "import sys; sys.path.insert(0, 'tools'); import v7_plan; print(v7_plan.W2_MAT)")
echo "$(date '+%F %T') 시작" >> "$RES/v7_run.log"
batch() {
  python3 tools/v7_plan.py order | while IFS=$'\t' read -r g s c k world yd sc; do
    [ -n "$ONLY" ] && ! grep -qw -- "$g" <<<"$ONLY" && continue
    [ -f "$RES/v7/$g/${s}_${c}_${k}/result.json" ] && continue
    echo "$(date '+%F %T') $g $s $c $k" >> "$RES/v7_run.log"
    mat=""; [ "$world" = 2 ] && mat=$W2_MAT
    CLIFF_WORLD=$world CLIFF_W2_MAT=$mat CLIFF_YIELD_D=$yd CLIFF_ODOM_SCALE=$sc \
      RING=true OUT=$RES/v7/$g SCENS=$s CONDS=$c SEEDS=$k NOREPORT=1 ./run_all.sh </dev/null
  done
}
batch
for pass in 1 2; do
  n=0
  for g in $(ls "$RES/v7" 2>/dev/null); do
    [ -d "$RES/v7/$g" ] || continue
    while IFS=$'\t' read -r d why; do
      [ -z "$d" ] && continue
      dst="$RES/v7_faults/pass$pass/${g}_$(basename "$d")"
      mkdir -p "$(dirname "$dst")" && mv "$d" "$dst" && echo "$why" > "$dst/fault_reason.txt" && n=$((n + 1))
    done < <(python3 tools/analyze_v7.py --faults "$RES/v7/$g")
  done
  echo "== 결함 정리 $pass회차: $n개"
  [ "$n" -eq 0 ] && break
  batch
done
echo "$(date '+%F %T') 끝" >> "$RES/v7_run.log"
python3 tools/analyze_v7.py "$RES/v7" > "$RES/v7/analysis.md" && echo "→ $RES/v7/analysis.md"
