#!/usr/bin/env bash
# 새 컴퓨터에서 처음 한 번: 원래 노트북에서 돌린 본 실험(results/runs)과 같은 시드를 다시 돌려,
# 결과가 같은 경향으로 나오는지 확인한다. GPU·CPU가 다르면 렌더링·통신 타이밍이 달라져 회차별 결과는 조금씩 다를 수 있다.
#   S2(사람이 모퉁이를 막음) × {P0 지도만, P1 keepout, P3 제안 계열} × 시드 1–3
#   S3(위치 착각)            × {P1 keepout, P3m 지도 좌표 기억, P3}  × 시드 1–3      → 18회, 약 30분
# 결과: results/calib_<호스트명>/ → tools/compare_calib.py 가 원래 결과와 나란히 비교한다
cd "$(dirname "$0")"
# 결과 폴더: 기본 results. 다른 컴퓨터 결과를 원래 결과와 섞지 않으려면 RES=results_<이름> 으로 따로 쌓는다
RES=${RES:-results}
export RESULTS_DIR="$PWD/$RES"     # 분석 스크립트(tools/analyze*.py)도 같은 폴더를 본다
OUT=$RES/calib_$(hostname)
OUT=$OUT SCENS="S2" CONDS="P0 P1 P3" SEEDS="1 2 3" NOREPORT=1 ./run_all.sh
OUT=$OUT SCENS="S3" CONDS="P1 P3m P3" SEEDS="1 2 3" NOREPORT=1 ./run_all.sh
python3 tools/find_faults.py "$OUT"
python3 tools/compare_calib.py "$OUT" | tee "$OUT/compare.md"
