#!/usr/bin/env bash
# 판정 기준 v4 확인 실험 (500회). 계획은 tools/v4_plan.py 한 곳에 있다.
#   ./run_v4.sh                 # 전부. 이미 result.json이 있는 회차는 건너뛰므로 끊겨도 다시 실행하면 이어서 돈다
#   ONLY_GROUPS="A B24 B30" ./run_v4.sh  # 일부 묶음만
# - 고정 전에는 돌지 않는다: 고정 파일 해시가 없거나 맞지 않으면 멈춘다 (기준 문서 11장).
# - 모든 방식의 운영자 keepout은 비용 띠 마스크(RING=true).
# - 순서: 시드 하나를 블록으로, 블록 안에서 (묶음, 시나리오, 방식)을 시드로 섞는다.
# - 장치 결함과 D의 주입 실패는 같은 시드로 다시 돌리고(최대 2번) 원본은 $RES/v4_faults/에 보관. 가짜 도착은 결함이 아니라 미도착으로 센다.
# 결과: $RES/v4/<묶음>/<시나리오>_<방식>_<시드>/  → $RES/v4/analysis.md
cd "$(dirname "$0")"
RES=${RES:-results_$(hostname)}
ONLY=${ONLY_GROUPS:-}
if ! (cd .. && sha256sum --quiet -c "demo_cliff/$RES/v4_frozen.sha256"); then
  echo "고정 파일 해시가 없거나 맞지 않습니다: $RES/v4_frozen.sha256 (기준 문서 11장)" >&2
  exit 1
fi
source env.sh
batch() {
  python3 tools/v4_plan.py order | while IFS=$'\t' read -r g s c k vmax void camkill ir_r; do
    [ -n "$ONLY" ] && ! grep -qw "$g" <<<"$ONLY" && continue
    [ -f "$RES/v4/$g/${s}_${c}_${k}/result.json" ] && continue
    CLIFF_IR_R=$ir_r CAMKILL=$camkill VOID=$void VMAX=$vmax RING=true OUT=$RES/v4/$g SCENS=$s CONDS=$c SEEDS=$k NOREPORT=1 ./run_all.sh </dev/null
  done
}
batch
for pass in 1 2; do
  n=0
  for g in $(ls "$RES/v4" 2>/dev/null); do
    [ -d "$RES/v4/$g" ] || continue
    while IFS=$'\t' read -r d why; do
      [ -z "$d" ] && continue
      dst="$RES/v4_faults/pass$pass/${g}_$(basename "$d")"
      mkdir -p "$(dirname "$dst")" && mv "$d" "$dst" && echo "$why" > "$dst/fault_reason.txt" && n=$((n + 1))
    done < <(python3 tools/analyze_v4.py --faults "$RES/v4/$g")
  done
  echo "== 결함 정리 $pass회차: $n개"
  [ "$n" -eq 0 ] && break
  batch
done
python3 tools/analyze_v4.py "$RES/v4" > "$RES/v4/analysis.md" && echo "→ $RES/v4/analysis.md"
