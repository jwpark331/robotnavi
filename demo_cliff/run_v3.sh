#!/usr/bin/env bash
# 판정 기준 v3 확인 실험 (400회). 계획은 tools/v3_plan.py 한 곳에 있다.
#   ./run_v3.sh                 # 전부. 이미 result.json이 있는 회차는 건너뛰므로 끊겨도 다시 실행하면 이어서 돈다
#   ONLY_GROUPS="H9a H10" ./run_v3.sh  # 일부 묶음만
# - 모든 방식의 운영자 keepout은 비용 띠 마스크(RING=true).
# - 순서: 시드 하나를 블록으로, 블록 안에서 (묶음, 시나리오, 방식)을 시드로 섞는다.
# - 장치 결함은 같은 시드로 다시 돌리고(최대 2번) 원본은 $RES/v3_faults/에 보관. 가짜 도착은 결함이 아니라 미도착으로 센다.
# 결과: $RES/v3/<묶음>/<시나리오>_<방식>_<시드>/  → $RES/v3/analysis.md
cd "$(dirname "$0")"
RES=${RES:-results_$(hostname)}
ONLY=${ONLY_GROUPS:-}
W2_MAT=$(python3 -c "import sys; sys.path.insert(0, 'tools'); import v3_plan; print(v3_plan.W2_MAT)")
source env.sh
batch() {
  python3 tools/v3_plan.py order | while IFS=$'\t' read -r g s c k mu world vmax; do
    [ -n "$ONLY" ] && ! grep -qw "$g" <<<"$ONLY" && continue
    [ -f "$RES/v3/$g/${s}_${c}_${k}/result.json" ] && continue
    mat=""
    [ "$world" = 2 ] && mat=$W2_MAT
    CLIFF_W2_MAT=$mat CLIFF_WORLD=$world VMAX=$vmax RING=true OUT=$RES/v3/$g SCENS=$s CONDS=$c SEEDS=$k MU=$mu NOREPORT=1 ./run_all.sh </dev/null
  done
}
batch
for pass in 1 2; do
  n=0
  for g in $(ls "$RES/v3" 2>/dev/null); do
    [ -d "$RES/v3/$g" ] || continue
    while IFS=$'\t' read -r d why; do
      [ -z "$d" ] && continue
      dst="$RES/v3_faults/pass$pass/${g}_$(basename "$d")"
      mkdir -p "$(dirname "$dst")" && mv "$d" "$dst" && echo "$why" > "$dst/fault_reason.txt" && n=$((n + 1))
    done < <(python3 tools/find_faults.py "$RES/v3/$g" | grep $'\t' | grep -v $'\t가짜 도착')
  done
  echo "== 결함 정리 $pass회차: $n개"
  [ "$n" -eq 0 ] && break
  batch
done
python3 tools/analyze_v3.py "$RES/v3" > "$RES/v3/analysis.md" && echo "→ $RES/v3/analysis.md"
