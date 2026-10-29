"""실험 장치 결함 회차 찾기 — 로봇의 판단이 아니라 시뮬·AMCL 문제로 망가진 회차.

    python3 tools/find_faults.py results/runs           # 목록만
    python3 tools/find_faults.py results/runs --delete  # 지우기 → ./run_all.sh 다시 돌리면 그 회차만 채운다

결함 기준: outcome이 infra_fault/sim_stalled, 또는 launch.log에 'Transform data too old'가 100줄 넘음(AMCL이 스캔 처리를 멈춤),
또는 40줄(≈2 s) 넘게 지연된 뒤 도착 못 함(추락 포함 — 진짜 추락은 0–8줄),
또는 result.json이 없음(시간 초과로 끊김).
"""
import json
import os
import shutil
import sys


def faults(root):
    out = []
    for dirpath, dirnames, filenames in os.walk(root):
        if "launch.log" not in filenames:
            continue
        rj = os.path.join(dirpath, "result.json")
        why = None
        if not os.path.exists(rj):
            why = "결과 없음"
        else:
            r = json.load(open(rj))
            if r["outcome"] in ("infra_fault", "sim_stalled"):
                why = r["outcome"]
            else:
                n = sum(1 for ln in open(os.path.join(dirpath, "launch.log"), errors="ignore") if "Transform data too old" in ln)
                if n > 100:
                    why = f"AMCL TF 멈춤 ({n}줄)"
                elif n > 40 and r["outcome"] != "arrived":
                    # 진짜 추락 회차는 TF 경고 0–8줄. 40줄(≈2 s) 넘으면 컨트롤러가 멈춘 TF로 같은 명령을 계속 낸 것
                    why = f"컨트롤러 TF 지연 {n}줄 뒤 {r['outcome']} (장치 결함 의심)"
                elif r["outcome"] == "arrived" and (r.get("goal_err") or 0) > 1.5:
                    why = f"가짜 도착 (실제로는 목표에서 {r['goal_err']} m)"
        if why:
            out.append((dirpath, why))
    return out


if __name__ == "__main__":
    root = sys.argv[1] if len(sys.argv) > 1 else "results/runs"
    fs = faults(root)
    for d, why in fs:
        print(f"{d}\t{why}")
    print(f"결함 회차 {len(fs)}개")
    if "--delete" in sys.argv:
        for d, _ in fs:
            shutil.rmtree(d)
        print("지움")
