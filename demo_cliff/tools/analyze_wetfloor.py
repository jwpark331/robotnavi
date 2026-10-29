"""젖은 바닥 실험 분석표 (마크다운) — results/wetfloor + 비교 기준 results/runs (μ 1.0).

    python3 tools/analyze_wetfloor.py > results/wetfloor/analysis.md

보는 것
  - 결과: 추락 / 걸침(중심이 가장자리 15 cm 안, 안 떨어짐) / 도착 / 갇힘·멈춤
  - 미끄러짐 크기: 바퀴 인코더 odom이 실제 위치에서 얼마나 어긋났나 (방향 °, 위치 cm)
  - 기억 유령: 기억된 낭떠러지 칸을 로봇이 믿는 자기 위치 기준으로 실제 세상에 옮겼을 때 계단 가장자리 앞
    10–70 cm 바닥 띠에 떨어지는 칸 수 (odom 기억은 odom 자세, map 기억은 AMCL 자세 기준). 회차 중 최대값.
"""
import bisect
import json
import math
import os
import statistics as st
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "ws", "src", "cliff_demo"))
from analyze import ROOT, load, pct  # noqa: E402
from analyze_darkstair import conf_until_end  # noqa: E402
from cliff_demo import geometry as G  # noqa: E402

WF = os.path.join(ROOT, "wetfloor")
CONDS = ["P1", "P1i", "P2", "P3m", "P3", "P3i"]
MUS = ["1.0", "0.1", "0.05", "0.02"]


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def offset(e):
    """world = R(dθ)·odom + T 의 (Tx, Ty, dθ). 처음과 달라진 만큼이 odom 어긋남."""
    gx, gy, gth = e["gt"][:3]
    ox, oy, oth = e["odom"]
    d = gth - oth
    c, s = math.cos(d), math.sin(d)
    return gx - (c * ox - s * oy), gy - (s * ox + c * oy), d


def in_band(x, y, lo=0.10, hi=0.70):
    """계단 가장자리 앞 바닥 띠 (가장자리에서 lo–hi m). 기억이 밀리면 여기에 유령이 생긴다.
    검은 매트·햇빛 구역(센서형 가짜)은 이 띠 밖이라 섞이지 않는다."""
    d = min(math.hypot(max(x0 - x, 0, x - x1), max(y0 - y, 0, y - y1)) for x0, y0, x1, y1, _ in G.VOIDS.values())
    return lo <= d <= hi


def enrich(d, r):
    """회차 폴더 d의 track·cliff 로그로 미끄러짐·유령 칸을 계산한다 (추락이면 추락 0.5 s 전까지)."""
    tr = [e for e in json.load(open(os.path.join(d, "track.json"))) if e["t"] <= r["time"] - 0.5 and e.get("odom")]
    r["conf"] = conf_until_end(d, r)
    if not tr:
        r["slip_deg"] = r["slip_cm"] = 0.0
        r["ghost"] = None
        return r
    a = offset(tr[0])
    r["slip_deg"] = max(abs(math.degrees(wrap(offset(e)[2] - a[2]))) for e in tr)
    r["slip_cm"] = max(100 * math.hypot(offset(e)[0] - a[0], offset(e)[1] - a[1]) for e in tr)
    w = G.WET_PATCH
    r["wet_s"] = 0.1 * sum(1 for e in tr if w[0] <= e["gt"][0] <= w[2] and w[1] <= e["gt"][1] <= w[3])
    r["ghost"] = None
    cj = os.path.join(d, "cliff.jsonl")
    if os.path.exists(cj):
        raw = [json.loads(x) for x in open(cj) if x.strip()]
        meta = raw[0]
        ts = [e["t"] for e in tr]
        best = 0
        for l in raw[1:]:
            t = l["t"] - r["t0_sim"]
            if t > r["time"] - 0.5 or not l["lethal"]:
                continue
            e = tr[min(bisect.bisect_left(ts, t), len(tr) - 1)]
            bel = e["odom"] if meta["frame"] == "odom" else e["est"]
            if not bel:
                continue
            gx, gy, gth = e["gt"][:3]
            dth = gth - bel[2]
            c, s = math.cos(dth), math.sin(dth)
            n = 0
            for v in l["lethal"]:
                ix, iy = v // 1000, v % 1000
                X = meta["ox"] + (ix + 0.5) * meta["res"] - bel[0]
                Y = meta["oy"] + (iy + 0.5) * meta["res"] - bel[1]
                n += in_band(gx + c * X - s * Y, gy + s * X + c * Y)
            best = max(best, n)
        r["ghost"] = best
    return r


def runs_at(mu, scen):
    dirs = [os.path.join(WF, f"mu_{mu}")] + ([os.path.join(ROOT, "runs")] if mu == "1.0" else [])
    out = []
    for d in dirs:
        if not os.path.isdir(d):
            continue
        for r in load(d):
            if r["scenario"] == scen and r["condition"] in CONDS:
                out.append(enrich(os.path.join(d, f"{r['scenario']}_{r['condition']}_{r['seed']}"), r))
    return out


def cell(rs):
    if not rs:
        return "–"
    n = len(rs)
    f = sum(r["outcome"] == "fall" for r in rs)
    near = sum(1 for r in rs if r["outcome"] != "fall" and r["min_edge"] < 0.15)
    a = [r for r in rs if r["outcome"] == "arrived"]
    other = n - f - len(a)
    s = f"추락 {f}/{n}" + (f" · 걸침 {near}" if near else "") + f" · 도착 {len(a)}" + (f" · 실패 {other}" if other else "")
    if a:
        s += f" · {st.mean(r['time'] for r in a):.0f} s"
    return ("🔴 " if f else "") + s


def med(xs, fmt="{:.1f}"):
    xs = [x for x in xs if x is not None]
    return (fmt.format(st.median(xs)) + " (최대 " + fmt.format(max(xs)) + ")") if xs else "–"


def main():
    data = {(mu, s): runs_at(mu, s) for mu in MUS for s in ("S1", "S2")}
    sel = lambda rs, **kw: [r for r in rs if all(r[k] == v for k, v in kw.items())]  # noqa: E731
    print("# 젖은 바닥 실험 결과\n")
    print(f"계단 A 바로 앞 모퉁이 {G.WET_PATCH}의 마찰계수 μ를 낮췄다. μ 1.0은 본 실험(보통 바닥). "
          "Gazebo에서는 μ ≈ 0.07 아래부터 바퀴가 미끄러지므로 0.05(비눗물·기름)와 0.02(얼음)를 시험했다.\n")

    print("## 1. 미끄러짐이 실제로 생겼나 (전 조건, 추락 전까지)\n")
    print("| μ | 시나리오 | 회차 | odom 방향 어긋남 ° (중앙값, 최대) | odom 위치 어긋남 cm | 위치추정(AMCL) 최대 오차 cm | 젖은 구간 체류 s |")
    print("|---|---|---|---|---|---|---|")
    for s in ("S2", "S1"):
        for mu in MUS:
            rs = data[(mu, s)]
            if s == "S1" and mu == "1.0":
                rs = [r for r in rs if r["seed"] in (1, 2, 3)]
            if not rs:
                continue
            print(f"| {mu} | {s} | {len(rs)} | {med([r['slip_deg'] for r in rs])} | {med([r['slip_cm'] for r in rs], '{:.0f}')} | "
                  f"{med([100 * r['amcl_max'] for r in rs], '{:.0f}')} | {med([r.get('wet_s') for r in rs], '{:.0f}')} |")

    print("\n## 2. S2 (모퉁이를 사람이 막아 젖은 구간에서 기다림) × 방식 (μ 0.02·0.05는 각 10회)\n")
    print("| 방식 | μ 1.0 (보통 바닥) | μ 0.1 | μ 0.05 | μ 0.02 |")
    print("|---|---|---|---|---|")
    for c in CONDS:
        print(f"| {c} | " + " | ".join(cell(sel(data[(mu, 'S2')], condition=c)) for mu in MUS) + " |")
    print("\n- 걸침: 떨어지진 않았지만 로봇 중심이 가장자리 15 cm 안까지 간 회차 (몸체가 허공에 걸침).")

    print("\n## 3. S1 (사람 없이 젖은 모퉁이를 돌아 지나감, 시드 1–3)\n")
    print("| 방식 | μ 1.0 | μ 0.02 |")
    print("|---|---|---|")
    for c in CONDS:
        base = [r for r in sel(data[("1.0", "S1")], condition=c) if r["seed"] in (1, 2, 3)]
        print(f"| {c} | {cell(base)} | {cell(sel(data[('0.02', 'S1')], condition=c))} |")

    print("\n## 4. 방식별 합계 (S2, μ 0.02 + 0.05 = 20회) 와 기억 유령\n")
    print("| 방식 | 추락률 (95% CI) | 걸침 | 도착률 | 가장자리 최소 (중앙값 / 최솟값) | 유령 칸: 가장자리 앞 10–70 cm (중앙값, 최대) | 가짜 낭떠러지 FP |")
    print("|---|---|---|---|---|---|---|")
    for c in CONDS:
        rs = sel(data[("0.02", "S2")] + data[("0.05", "S2")], condition=c)
        if not rs:
            continue
        n = len(rs)
        f = sum(r["outcome"] == "fall" for r in rs)
        near = sum(1 for r in rs if r["outcome"] != "fall" and r["min_edge"] < 0.15)
        a = sum(r["outcome"] == "arrived" for r in rs)
        me = [r["min_edge"] for r in rs]
        cs = [r["conf"] for r in rs if r["conf"]]
        print(f"| {c} | {pct(f, n)} | {near} | {pct(a, n)} | {st.median(me):.2f} / {min(me):.2f} m | "
              f"{med([r['ghost'] for r in rs], '{:.0f}')} | {sum(x['FP'] for x in cs):,} |" if cs else
              f"| {c} | {pct(f, n)} | {near} | {pct(a, n)} | {st.median(me):.2f} / {min(me):.2f} m | – | – |")
    print("\n유령 칸 비교 (보통 바닥 S2, 본 실험):")
    for c in ("P2", "P3m", "P3", "P3i"):
        rs = sel(data[("1.0", "S2")], condition=c)
        if rs:
            print(f"- {c}: {med([r['ghost'] for r in rs], '{:.0f}')}")

    print("\n## 5. 추락·걸침 회차 상세\n")
    print("| μ | 시나리오 | 방식 | 시드 | 결과 | 시각 | 가장자리 최소 | odom 방향 어긋남 | 유령 칸 | IR 급정지 |")
    print("|---|---|---|---|---|---|---|---|---|---|")
    for (mu, s), rs in sorted(data.items()):
        for r in sorted(rs, key=lambda r: (r["condition"], r["seed"])):
            if mu == "1.0" or not (r["outcome"] == "fall" or r["min_edge"] < 0.15):
                continue
            print(f"| {mu} | {s} | {r['condition']} | {r['seed']} | {r['outcome']} | {r['time']:.0f} s | {r['min_edge']:.2f} m | "
                  f"{r['slip_deg']:.1f}° | {r['ghost'] if r['ghost'] is not None else '–'} | {len(r['ir_stops']) if r['ir_stops'] else '–'} |")

    fdir = os.path.join(ROOT, "wetfloor_faults")
    moved = []
    for dirpath, _, files in os.walk(fdir) if os.path.isdir(fdir) else []:
        if "fault_reason.txt" in files:
            rj = os.path.join(dirpath, "result.json")
            out = json.load(open(rj))["outcome"] if os.path.exists(rj) else "결과 없음"
            moved.append((os.path.relpath(dirpath, fdir), out, open(os.path.join(dirpath, "fault_reason.txt")).read().strip()))
    print(f"\n실험 장치 결함으로 다시 돌린 회차: {len(moved)}개 (원본은 results/wetfloor_faults/에 보관)")
    for p, out, why in sorted(moved):
        print(f"- {p}: 원래 결과 {out} — {why}")


if __name__ == "__main__":
    main()
