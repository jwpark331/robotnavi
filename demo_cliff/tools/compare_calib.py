"""보정 실행 비교: 새 컴퓨터에서 돌린 회차를 원래 노트북의 같은 회차(results/runs, 같은 시나리오·방식·시드)와 나란히 본다.

    python3 tools/compare_calib.py results/calib_<호스트명>

판정은 주 지표와 같은 '안전 실패(추락 또는 걸침)'와 도착 여부로 한다. 시뮬레이션은 같은 기계에서도 회차마다 조금씩 다르므로
회차 하나하나가 같을 필요는 없고, 방식별 경향이 같은지를 본다.
"""
import json
import os
import statistics as st
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from analyze import load  # noqa: E402
from find_faults import faults  # noqa: E402


def unsafe(r):
    return r["outcome"] == "fall" or r["min_edge"] < 0.15


def main():
    d = sys.argv[1]
    new = load(d)
    ref_dir = os.path.join(HERE, "..", "results", "runs")   # 늘 원래 노트북 결과와 비교 (RESULTS_DIR와 무관)
    rows = []
    for r in sorted(new, key=lambda r: (r["scenario"], r["condition"], r["seed"])):
        p = os.path.join(ref_dir, f"{r['scenario']}_{r['condition']}_{r['seed']}", "result.json")
        if os.path.exists(p):
            rows.append((r, json.load(open(p))))
    machine = os.path.join(d, rows[0][0] and f"{rows[0][0]['scenario']}_{rows[0][0]['condition']}_{rows[0][0]['seed']}", "machine.txt") if rows else ""
    print(f"# 보정 실행 비교 — {os.path.basename(os.path.normpath(d))}\n")
    if machine and os.path.exists(machine):
        print("새 컴퓨터:\n\n```\n" + open(machine).read().strip() + "\n```\n")
    print(f"비교한 회차 {len(rows)}개 (장치 결함으로 뺀 회차 {len(faults(d))}개)\n")
    print("| 회차 | 원래 노트북 | 새 컴퓨터 | 안전 판정 같음 | 도착 여부 같음 |")
    print("|---|---|---|---|---|")
    same_safe = same_arr = 0
    for n, o in rows:
        a = unsafe(o) == unsafe(n)
        b = (o["outcome"] == "arrived") == (n["outcome"] == "arrived")
        same_safe += a
        same_arr += b
        fmt = lambda r: f"{r['outcome']} {r['time']:.0f} s, 가장자리 {r['min_edge']:.2f} m"  # noqa: E731
        print(f"| {n['scenario']}_{n['condition']}_{n['seed']} | {fmt(o)} | {fmt(n)} | {'✓' if a else '✗'} | {'✓' if b else '✗'} |")
    if not rows:
        print("\n비교할 회차가 없습니다.")
        return
    print(f"\n- 안전 판정(추락·걸침 여부)이 같은 회차: **{same_safe}/{len(rows)}**")
    print(f"- 도착 여부가 같은 회차: **{same_arr}/{len(rows)}**")
    both = [(n["time"] - o["time"]) for n, o in rows if n["outcome"] == o["outcome"] == "arrived"]
    if both:
        print(f"- 둘 다 도착한 회차의 시간 차 (새 − 원래) 중앙값: {st.median(both):+.1f} s ({len(both)}쌍)")
    print("\n## 방식별 경향\n")
    print("| 시나리오·방식 | 원래: 안전 실패 / 도착 | 새: 안전 실패 / 도착 | 원래 노트북에서 나온 경향 |")
    print("|---|---|---|---|")
    expect = {("S2", "P0"): "계단으로 떨어진다", ("S3", "P3m"): "유령 낭떠러지에 갇힌다 (도착 못 함)",
              ("S2", "P3"): "안 떨어지고 도착", ("S3", "P3"): "안 떨어지고 도착",
              ("S2", "P1"): "대체로 도착", ("S3", "P1"): "일부 떨어짐"}
    keys = sorted({(n["scenario"], n["condition"]) for n, _ in rows})
    ok = True
    for k in keys:
        g = [(n, o) for n, o in rows if (n["scenario"], n["condition"]) == k]
        ou, oa = sum(unsafe(o) for _, o in g), sum(o["outcome"] == "arrived" for _, o in g)
        nu, na = sum(unsafe(n) for n, _ in g), sum(n["outcome"] == "arrived" for n, _ in g)
        print(f"| {k[0]} {k[1]} | {ou}/{len(g)} / {oa}/{len(g)} | {nu}/{len(g)} / {na}/{len(g)} | {expect.get(k, '')} |")
        if k in (("S2", "P0"), ("S3", "P3m"), ("S2", "P3"), ("S3", "P3")) and abs(ou - nu) > 1:
            ok = False
    print()
    if ok and same_safe >= len(rows) - 2:
        print("**판정: 같은 경향.** 핵심 대비(지도만 → 떨어짐, 지도 좌표 기억 → 갇힘, 제안 → 안전)가 원래 노트북과 같다. "
              "이 컴퓨터에서 새 실험을 돌려도 된다. 다만 기존 폴더에 섞지 말고 새 폴더에 쌓고, 비교는 같은 컴퓨터 결과끼리 한다.")
    else:
        print("**판정: 차이가 있다.** 회차별 영상(video.mp4)과 machine.txt를 확인하고, 이 컴퓨터 결과는 원래 결과와 따로 분석한다. "
              "장치 결함(TF 경고)이 잦으면 README의 'GPU·성능 차이' 항목을 본다.")


if __name__ == "__main__":
    main()
