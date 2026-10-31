#!/usr/bin/env bash
# 시나리오 × 조건 × 시드 회차를 차례로 돌리고, 위 카메라 프레임을 영상(mp4)으로 묶은 뒤 리포트를 만든다.
#
#   ./run_all.sh                                # 전부 (S1–S4 × P0 P1 P1i P2 P3m P3 × 시드 1)
#   SEEDS="1 2 3 4 5" ./run_all.sh              # 본 실험 (120회차, 약 4–5시간)
#   SCENS="S2" CONDS="P0 P1" ./run_all.sh       # 일부만
#   SEEDS="1 2 3" ./run_all.sh                  # 시드 여러 개
#   GUI=true SCENS=S3 CONDS=P3 ./run_all.sh     # Gazebo 창 띄워서 보기
#
# 이미 result.json이 있는 회차는 건너뛴다 (다시 돌리려면 그 폴더를 지운다).
cd "$(dirname "$0")"
source env.sh
if uname -r | grep -q cip; then echo "EVL 커널입니다. generic 커널로 부팅 후 실행하세요." >&2; exit 1; fi

RES=${RES:-results}       # 결과 폴더 (다른 컴퓨터는 RES=results_<이름> 으로 따로 쌓는다)
OUT=${OUT:-$RES/runs}
SCENS=${SCENS:-"S1 S2 S3 S4"}
CONDS=${CONDS:-"P0 P1 P1i P2 P3m P3"}
SEEDS=${SEEDS:-"1"}
GUI=${GUI:-false}
PUSH=${PUSH:--1}           # S3 밀림 거리 고정 [m] (음수 = 시드로 0.8–1.0)
DROPOUT=${DROPOUT:-0.75}   # 검은 매트 결측률
VOID=${VOID:-0.0}          # 검은 계단: 계단 구멍 광선 결측률 (본 실험 0)
MU=${MU:-1.0}              # 젖은 바닥: 계단 A 앞 모퉁이 마찰계수 (본 실험 1.0)
PITCH=${PITCH:-0.0}        # 가감속 연동 기울기 오차 [deg/(m/s²)] (본 실험 0)
RING=${RING:-false}        # v2: 모든 방식의 운영자 keepout을 비용 띠 마스크로 (본 실험 false)
VMAX=${VMAX:-0}            # v3: 최고 속도 [m/s] (0 = nav2.yaml 그대로 0.45)
NOREPORT=${NOREPORT:-0}

for s in $SCENS; do for c in $CONDS; do for k in $SEEDS; do
  d="$OUT/${s}_${c}_${k}"
  [ -f "$d/result.json" ] && continue
  tools/kill_sim.sh
  rm -rf "$d"; mkdir -p "$d"
  tools/record_env.sh "$d/machine.txt"     # 어느 컴퓨터에서 돌렸나 (GPU가 다른 노트북 결과와 구분)
  echo "▶ $s $c seed=$k"
  timeout 420 ros2 launch cliff_demo demo.launch.py scenario:=$s condition:=$c seed:=$k \
      out_dir:="$PWD/$d" gui:=$GUI push_dx:=$PUSH mat_dropout:=$DROPOUT void_dropout:=$VOID floor_mu:=$MU pitch_k:=$PITCH keepout_ring:=$RING max_vel:=$VMAX > "$d/launch.log" 2>&1
  tools/kill_sim.sh
  if ls "$d"/frames/*.jpg >/dev/null 2>&1 && ! command -v ffmpeg >/dev/null; then
    python3 -I tools/encode_frames.py "$d" >/dev/null   # ffmpeg가 없으면 OpenCV로 (10/30, JW-laptop)
  elif ls "$d"/frames/*.jpg >/dev/null 2>&1; then
    ffmpeg -y -loglevel error -framerate 8 -i "$d/frames/%05d.jpg" -c:v libx264 -pix_fmt yuv420p -crf 30 \
        -vf "scale=720:-2" -movflags +faststart "$d/video.mp4" && rm -rf "$d/frames"
  fi
  python3 -c "import json,sys; r=json.load(open('$d/result.json')); print(f\"  → {r['outcome']}  {r['time']} s  가장자리 최소 {r['min_edge']} m  복구 {r['recoveries']}회\")" 2>/dev/null \
      || echo "  → 결과 없음 ($d/launch.log 확인)"
done; done; done

# 기본 결과($RES/runs)는 $RES/report.html, 다른 OUT이면 그 폴더 안에 report.html
[ "$NOREPORT" = 1 ] && exit 0
if [ "$OUT" = "$RES/runs" ]; then REPORT=$RES/report.html; else REPORT="$OUT/report.html"; fi
python3 tools/make_report.py "$OUT" "$REPORT"
