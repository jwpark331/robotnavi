"""판정 기준 v6 확인 실험 뒤 탐색 분석 (11-20, 판정에 쓰지 않는다 — 고정 파일 아님).

    python3 tools/explore_v6.py [결과 폴더]     # 기본 results_<호스트>/v6

판정이 난 뒤, 발표 질의에 대비해 원인만 살핀다. 판정 문장 · 기준은 바꾸지 않는다.
  A. ① 로봇의 후진 여유가 0.25 m 아래로 내려간 회차의 원인 분해
     기억이 남긴 여유 = 후진 시작 참값 자세에서 등 방향으로 a(갈 수 있다고 계산한 거리)만큼 간 점의 가장자리 거리.
       기억한 가장자리가 정확하면 0.30 m (격자 0.1 m라 ±0.05). 요청이 a보다 짧으면 실제로는 거기까지 안 간다.
     실행 오차 = (odom 후진 − 목표: 멈춤 지연) + (참값 후진 − odom 후진: 슬립)
  B. ⑤ 로봇이 후진 묶음에서 계단을 못 찍은 회차
  C. ⑤ 로봇의 안전 실패 회차
  D. ② 로봇이 비켜 준 / 거부한 회차의 시작 위치
"""
import json
import math
import os
import socket
import statistics
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from analyze_v2 import safety_fail  # noqa: E402
from analyze_v3 import voids  # noqa: E402
from analyze_v4 import signed_edge  # noqa: E402
from analyze_v6 import load, meas, backup_margin  # noqa: E402

GROUPS = ["Y-w1-d055", "Y-w1-d110", "Y-w2-d055", "Y-w2-d110"]


def ev(r, kind):
    return next((e for e in r["events"] if e["kind"] == kind), None)


def track(r):
    return json.load(open(os.path.join(r["dir"], "track.json")))


def at(tr, t):
    return min(tr, key=lambda p: abs(p["t"] - t))


def sb_moved(r):
    p = os.path.join(r["dir"], "safe_backup.jsonl")
    if not os.path.exists(p):
        return None
    for line in open(p):
        rec = json.loads(line)
        if rec["kind"] == "end":
            return rec["moved_odom"]
    return None


def part_a(data):
    print("## A. ① 로봇 후진 여유 분해 (판정 아님)\n")
    print("| 회차 | e0 | a | 기억이 남긴 여유 | 목표 | odom 후진 | 참값 후진 | 멈춤 지연 | 슬립 | 후진 중 최소 여유 | 0.25 아래 |")
    print("|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|")
    rows = []
    for (g, s, c, k), r in sorted(data.items()):
        if c != "P3y":
            continue
        bk = ev(r, "backup")
        tr = track(r)
        rects = voids(r.get("world", 1))
        p0 = at(tr, bk["t"])
        e0 = signed_edge(p0["gt"][0], p0["gt"][1], rects)
        yp = r["yield_plan"]
        x, y, th = p0["gt"][:3]                     # gt = [x, y, yaw, pitch]
        left = signed_edge(x - yp["a"] * math.cos(th), y - yp["a"] * math.sin(th), rects)
        mo = sb_moved(r)
        mg = meas(r)["moved"]
        m = backup_margin(r)
        row = dict(run=f"{g} {k}", e0=e0, a=yp["a"], left=left, target=yp["target"], odom=mo, gt=mg,
                   lag=mo - yp["target"], slip=mg - mo, margin=m, low=m < 0.25)
        rows.append(row)
        if row["low"]:
            print(f"| {row['run']} | {e0:.3f} | {yp['a']:.3f} | {row['left']:.3f} | {yp['target']:.3f} | {mo:.3f} | "
                  f"{mg:.3f} | {row['lag']:+.3f} | {row['slip']:+.3f} | {m:.3f} | 예 |")
    print()
    for name, sel in (("0.25 아래 회차", [x for x in rows if x["low"]]), ("나머지 회차", [x for x in rows if not x["low"]]),
                      ("① 전체", rows)):
        print(f"- {name} ({len(sel)}회): 기억이 남긴 여유 중앙값 {statistics.median(x['left'] for x in sel):.3f} m "
              f"(최소 {min(x['left'] for x in sel):.3f}), 멈춤 지연 중앙값 {statistics.median(x['lag'] for x in sel):+.3f} m, "
              f"슬립 중앙값 {statistics.median(x['slip'] for x in sel):+.3f} m (최대 {max(x['slip'] for x in sel):+.3f}), "
              f"기억이 남긴 여유 0.25 아래 {sum(x['left'] < 0.25 for x in sel)}회 · 0.15 아래 {sum(x['left'] < 0.15 for x in sel)}회")
    print("\n묶음별 ('기억이 남긴 여유'가 0.30 m에서 얼마나 벗어나는지):\n")
    for g in GROUPS:
        sel = [x for x in rows if x["run"].startswith(g)]
        print(f"- {g}: 기억이 남긴 여유 {min(x['left'] for x in sel):.3f} ~ {max(x['left'] for x in sel):.3f} m, "
              f"0.25 아래 {sum(x['low'] for x in sel)}회")
    print()


def part_b(data):
    print("## B. ⑤ 로봇이 후진 묶음에서 계단을 못 찍은 회차 (판정 아님)\n")
    print("| 회차 | 찍힌 칸 최대 | 깊이 점 메시지 수 | 후진 시작 가장자리 e0 | 후진 결과 | 결과 |")
    print("|---|---:|---:|---:|---|---|")
    for (g, s, c, k), r in sorted(data.items()):
        if c != "P2nav" or (r["drop_marks_max"] or 0) > 0:
            continue
        bk = ev(r, "backup")
        tr = track(r)
        e0 = signed_edge(*at(tr, bk["t"])["gt"][:2], voids(r.get("world", 1))) if bk else float("nan")
        be = ev(r, "backup_end")
        print(f"| {g} {k} | {r['drop_marks_max']} | {r['drop_msgs']} | {e0:.3f} | {be['result'] if be else '–'} | {r['outcome']} |")
    print()
    for g in GROUPS:
        rs = [r for (gg, s, c, k), r in data.items() if gg == g and c == "P2nav"]
        ts = [r["drop_marks_t"] for r in rs if r["drop_marks_t"] is not None]
        print(f"- {g}: 찍힌 회차 {sum((r['drop_marks_max'] or 0) > 0 for r in rs)}/{len(rs)}, "
              f"처음 찍힌 시각 중앙값 {statistics.median(ts) if ts else float('nan'):.1f} s, "
              f"찍힌 칸 최대 중앙값 {statistics.median(r['drop_marks_max'] or 0 for r in rs)}")
    print()


def part_c(data):
    print("## C. ⑤ 로봇의 안전 실패 회차 (판정 아님)\n")
    print("| 회차 | 결과 | 최소 가장자리 | 그때 시각 | 후진 구간 | 후진 중 최소 | 실제 후진 | 거부 |")
    print("|---|---|---:|---:|---|---:|---:|---|")
    for (g, s, c, k), r in sorted(data.items()):
        if c != "P2nav" or s != "S8" or not safety_fail(r):
            continue
        tr = track(r)
        rects = voids(r.get("world", 1))
        es = [(signed_edge(p["gt"][0], p["gt"][1], rects), p["t"]) for p in tr]
        emin, tmin = min(es)
        bk, be = ev(r, "backup"), ev(r, "backup_end")
        span = f"{bk['t']:.1f}–{be['t'] if be else r['time']:.1f}" if bk else "–"
        m = meas(r)
        print(f"| {g} {k} | {r['outcome']} | {emin:.3f} | {tmin:.1f} | {span} | {backup_margin(r):.3f} | "
              f"{m['moved']:.3f} | {m['refused']} |")
    print()


def part_d(data):
    print("## D. ② 로봇: Nav2 후진이 멈춘 회차와 후진 시작 가장자리 (판정 아님)\n")
    print("'멈춤'은 s8_backup_ref의 거부 정의(0.01 m 미만, 또는 Collision Ahead로 멈춤)다. "
          "1.10 m에서는 0.2 m 넘게 물러난 뒤 멈춘 회차도 들어간다.\n")
    print("| 묶음 | 회차 | 멈춤 | 실제 후진 중앙값 | 시작 가장자리 e0 중앙값 (범위) | 멈춘 회차 e0 범위 | 끝까지 간 회차 e0 범위 |")
    print("|---|---:|---:|---:|---|---|---|")
    for g in GROUPS:
        rs = [r for (gg, s, c, k), r in data.items() if gg == g and c == "P3"]
        out = []
        for r in rs:
            bk = ev(r, "backup")
            e0 = signed_edge(*at(track(r), bk["t"])["gt"][:2], voids(r.get("world", 1)))
            out.append((e0, meas(r)["refused"]))
        rf = [e for e, x in out if x]
        ok = [e for e, x in out if not x]
        rng = lambda v: f"{min(v):.3f} ~ {max(v):.3f}" if v else "–"  # noqa: E731
        print(f"| {g} | {len(out)} | {len(rf)} | {statistics.median(meas(r)['moved'] for r in rs):.3f} m | "
              f"{statistics.median(e for e, _ in out):.3f} ({rng([e for e, _ in out])}) | "
              f"{rng(rf)} | {rng(ok)} |")
    print()


def main():
    root = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, "..", f"results_{socket.gethostname()}", "v6")
    data = {}
    for g in GROUPS:
        data.update(load(root, g))
    print(f"# 판정 기준 v6 — 탐색 분석 (판정 뒤, 판정에 쓰지 않음)\n\n결과 폴더: `{os.path.relpath(root, os.path.join(HERE, '..'))}`\n")
    part_a(data)
    part_b(data)
    part_c(data)
    part_d(data)


if __name__ == "__main__":
    main()
