"""S8(계단 옆에서 뒤로 비켜 주기) 참고 지표 — 판정이 아니다. 후진이 실제로 일어났는지 방식별로 센다.

    python3 tools/s8_backup_ref.py <결과 폴더> [--runs]
    예) python3 tools/s8_backup_ref.py results_JW-laptop/v2/H4
        python3 tools/s8_backup_ref.py results_JW-laptop/v5/WS8

폴더 아래의 result.json 중 scenario가 S8인 회차를 모은다 (장치 결함 회차는 find_faults 규칙으로 뺀다, 가짜 도착은 남긴다).
회차마다
- 요구 거리: events의 backup.dist (scenario.py가 Nav2 BackUp에 넘긴 값)
- 실제 후진 거리: backup 시각부터 backup_end + 0.2 s까지 track.json 참값(gt)이 후진 시작 위치에서 벗어난 최대 거리
- 거부: 실제 후진 거리 0.01 m 미만, 또는 backup_end가 stopped이고 launch.log에 'Running backup' 3줄 안에 'Collision Ahead - Exiting DriveOnHeading'이 있음
- 시작 가장자리 거리: 후진 시작 시각의 참값 위치에서 가장 가까운 계단 가장자리까지 (구멍 안이면 음수)
- 안전 실패 · 도착: analyze_v2.safety_fail, analyze_v3.arrived (목표 1.5 m 안)와 같은 정의
월드는 result.json의 world(없으면 1)로 고르고, geometry.py를 analyze_v3.voids()처럼 CLIFF_WORLD 환경변수로 읽는다.
판정 기준 v5 부속 기록(2026-11-10)이 미리 등록한 참고 지표다. 판정 지표(안전 실패)와 성립 조건은 바꾸지 않는다.
"""
import json
import math
import os
import statistics
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from analyze_v2 import safety_fail  # noqa: E402
from analyze_v3 import arrived, voids  # noqa: E402
from analyze_v4 import signed_edge  # noqa: E402
from find_faults import faults  # noqa: E402

TAIL = 0.2        # backup_end 뒤에 더 보는 시간 [s]
REFUSE_D = 0.01   # 이보다 덜 움직였으면 후진 거부 [m]
COLLISION = "Collision Ahead - Exiting DriveOnHeading"


def event(r, kind):
    return next((e for e in r["events"] if e["kind"] == kind), None)


def log_collision_backup(run_dir):
    """launch.log에서 'Running backup' 직후(behavior_server 줄 3줄 안, 사이에 'no acceleration' 안내 줄이 끼는 경우 포함)
    'Collision Ahead'로 끝난 후진이 있나. 복구 동작의 후진도 세므로, 거부 판정에는 backup_end가 stopped인 것과 함께 쓴다."""
    path = os.path.join(run_dir, "launch.log")
    if not os.path.exists(path):
        return False
    lines = [ln for ln in open(path, errors="ignore") if "[behavior_server" in ln]
    for i, a in enumerate(lines):
        if "Running backup" in a and any(COLLISION in b for b in lines[i + 1:i + 4]):
            return True
    return False


def measure(r):
    """한 회차의 참고 지표. 후진 기록이 없으면 None."""
    bk = event(r, "backup")
    if bk is None:
        return None
    end = event(r, "backup_end")
    t1 = (end["t"] if end else r["time"]) + TAIL
    tr = json.load(open(os.path.join(r["dir"], "track.json")))
    rects = voids(r.get("world", 1))
    start = min(tr, key=lambda p: abs(p["t"] - bk["t"]))
    x0, y0 = start["gt"][0], start["gt"][1]
    moved = max((math.hypot(p["gt"][0] - x0, p["gt"][1] - y0) for p in tr if bk["t"] <= p["t"] <= t1), default=0.0)
    stopped = end is not None and end.get("result") == "stopped"
    return dict(req=bk["dist"], moved=moved, edge0=signed_edge(x0, y0, rects),
                refused=moved < REFUSE_D or (stopped and log_collision_backup(r["dir"])),
                stopped=stopped, collision=log_collision_backup(r["dir"]))


def load(root):
    bad = {os.path.abspath(d) for d, _ in faults(root) if not _.startswith("가짜 도착")}
    out = []
    for dirpath, _, filenames in os.walk(root):
        if "result.json" not in filenames or os.path.abspath(dirpath) in bad:
            continue
        r = json.load(open(os.path.join(dirpath, "result.json")))
        if r["scenario"] == "S8":
            r["dir"] = dirpath
            out.append(r)
    return sorted(out, key=lambda r: (r["condition"], r["seed"])), len(bad)


def med(vs, fmt="{:.3f}"):
    return fmt.format(statistics.median(vs)) if vs else "–"


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    if not args:
        print(__doc__)
        sys.exit(1)
    root = args[0]
    runs, n_bad = load(root)
    print(f"# S8 후진 참고 지표 (판정 아님)\n\n결과 폴더: `{root}` — S8 회차 {len(runs)}개 (장치 결함으로 뺀 회차 {n_bad}개)\n")
    print("| 방식 | 회차 | 후진 기록 | 요구 거리 중앙값 | 실제 후진 중앙값 | 실제 후진 최대 | 거부 회차 | 시작 가장자리 중앙값 | 안전 실패 | 도착 |")
    print("|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|")
    rows = {}
    for r in runs:
        rows.setdefault(r["condition"], []).append((r, measure(r)))
    for cond, rs in rows.items():
        ms = [m for _, m in rs if m]
        print(f"| {cond} | {len(rs)} | {len(ms)} | {med([m['req'] for m in ms])} m | {med([m['moved'] for m in ms])} m | "
              f"{max((m['moved'] for m in ms), default=0):.3f} m | {sum(m['refused'] for m in ms)} | "
              f"{med([m['edge0'] for m in ms])} m | {sum(safety_fail(r) for r, _ in rs)} | {sum(arrived(r) for r, _ in rs)} |")
    print("\n거부 = 실제 후진 0.01 m 미만, 또는 backup_end가 stopped이고 launch.log에 'Running backup' 3줄 안에 'Collision Ahead'. "
          "실제 후진 = backup ~ backup_end + 0.2 s 사이 참값의 최대 이동. 시작 가장자리 = 후진 시작 위치에서 계단 가장자리까지.")
    if "--runs" in sys.argv:
        print("\n| 방식 | 시드 | 요구 | 실제 | 시작 가장자리 | backup_end | 로그 Collision Ahead 후진 | 거부 | 결과 | min_edge |")
        print("|---|---:|---:|---:|---:|---|---|---|---|---:|")
        for cond, rs in rows.items():
            for r, m in rs:
                if m is None:
                    print(f"| {cond} | {r['seed']} | – | – | – | 후진 기록 없음 | | | {r['outcome']} | {r['min_edge']} |")
                    continue
                print(f"| {cond} | {r['seed']} | {m['req']:.3f} | {m['moved']:.3f} | {m['edge0']:.3f} | "
                      f"{'stopped' if m['stopped'] else 'succeeded'} | {'예' if m['collision'] else '아니오'} | "
                      f"{'예' if m['refused'] else '아니오'} | {r['outcome']} | {r['min_edge']} |")


if __name__ == "__main__":
    main()
