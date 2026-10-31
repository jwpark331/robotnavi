#!/usr/bin/env bash
# 판정 기준 v2 확인 실험 (약 710회). 계획은 tools/v2_plan.py 한 곳에 있다.
#   ./run_v2.sh                 # 전부. 이미 result.json이 있는 회차는 건너뛰므로 끊겨도 다시 실행하면 이어서 돈다
#   ONLY_GROUPS="H1 H4" ./run_v2.sh  # 일부 묶음만
# - 모든 방식의 운영자 keepout은 비용 띠 마스크(RING=true).
# - 순서: 시드 하나를 블록으로, 블록 안에서 (묶음, 시나리오, 방식)을 시드로 섞는다.
# - 장치 결함(tools/find_faults.py, 결과 무관 규칙)은 같은 시드로 다시 돌리고(최대 2번) 원본은 $RES/v2_faults/에 보관.
# 결과: $RES/v2/<묶음>/<시나리오>_<방식>_<시드>/  → $RES/v2/analysis.md
cd "$(dirname "$0")"
RES=${RES:-results_$(hostname)}
ONLY=${ONLY_GROUPS:-}
source env.sh
batch() {
  python3 tools/v2_plan.py order | while IFS=$'\t' read -r g s c k mu void world; do
    [ -n "$ONLY" ] && ! grep -qw "$g" <<<"$ONLY" && continue
    [ -f "$RES/v2/$g/${s}_${c}_${k}/result.json" ] && continue
    if [ "$world" != 1 ] && [ ! -f "ws/src/cliff_demo/worlds/world$world.world" ]; then continue; fi
    CLIFF_WORLD=$world RING=true OUT=$RES/v2/$g SCENS=$s CONDS=$c SEEDS=$k MU=$mu VOID=$void NOREPORT=1 ./run_all.sh </dev/null
  done
}
batch
for pass in 1 2; do
  n=0
  for g in $(ls "$RES/v2" 2>/dev/null); do
    [ -d "$RES/v2/$g" ] || continue
    while IFS=$'\t' read -r d why; do
      [ -z "$d" ] && continue
      dst="$RES/v2_faults/pass$pass/${g}_$(basename "$d")"
      mkdir -p "$(dirname "$dst")" && mv "$d" "$dst" && echo "$why" > "$dst/fault_reason.txt" && n=$((n + 1))
    done < <(python3 tools/find_faults.py "$RES/v2/$g" | grep $'\t')
  done
  echo "== 결함 정리 $pass회차: $n개"
  [ "$n" -eq 0 ] && break
  batch
done
python3 tools/analyze_v2.py "$RES/v2" > "$RES/v2/analysis.md" && echo "→ $RES/v2/analysis.md"
