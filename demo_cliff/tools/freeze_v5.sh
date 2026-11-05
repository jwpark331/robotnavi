#!/usr/bin/env bash
# 판정 기준 v5 고정 — 판정에 닿는 파일의 해시를 $RES/v5_frozen.sha256에 쓴다. run_v5.sh는 이 해시가 맞을 때만 돈다.
# 박정원 확인 뒤, 사전 시험을 마치고 기준 문서에 예측표 · 설정값을 옮긴 다음에 한 번만 실행한다 (기준 문서 9장 7번).
# 그다음 커밋 · 푸시를 사용자가 하고, 푸시 시각이 첫 회차보다 앞서야 한다.
#   tools/freeze_v5.sh
cd "$(dirname "$0")/../.."     # 저장소 맨 위
RES=${RES:-results_$(hostname)}
OUT="demo_cliff/$RES/v5_frozen.sha256"
if [ -f "$OUT" ]; then echo "이미 고정됐다: $OUT (다시 고정하려면 이유를 기준 문서에 적고 직접 지운다)" >&2; exit 1; fi
FILES=(
  "보이지 않는 계단 — 판정 기준 v5 초안 (2026-11-05).md"
  demo_cliff/tools/v5_plan.py demo_cliff/tools/analyze_v5.py
  demo_cliff/tools/analyze_v2.py demo_cliff/tools/analyze_v3.py demo_cliff/tools/analyze_v4.py demo_cliff/tools/find_faults.py
  demo_cliff/run_v5.sh demo_cliff/run_all.sh demo_cliff/env.sh
  demo_cliff/ws/src/cliff_demo/cliff_demo/geometry.py demo_cliff/ws/src/cliff_demo/cliff_demo/scenario.py
  demo_cliff/ws/src/cliff_demo/cliff_demo/ir_guard.py demo_cliff/ws/src/cliff_demo/cliff_demo/cliff_detector.py
  demo_cliff/ws/src/cliff_demo/launch/demo.launch.py demo_cliff/ws/src/cliff_demo/launch/sim.launch.py
  demo_cliff/ws/src/cliff_demo/urdf/servi.urdf.xacro demo_cliff/ws/src/cliff_demo/params/nav2.yaml
)
for f in demo_cliff/$RES/v5_pilot/brake/acc_*.jsonl; do FILES+=("$f"); done   # 감속 성립 조건이 읽는 제동 시험 결과
sha256sum "${FILES[@]}" > "$OUT" && echo "고정: $OUT ($(wc -l < "$OUT")개 파일)"
