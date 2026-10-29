"""움직이는 사람 실험 분석표 (마크다운) — results/walkers + 서 있는 사람 기준 (본 실험 S2·S4).

    python3 tools/analyze_walkers.py > results/walkers/analysis.md

S5 ↔ S2 (계단 A 앞 모퉁이에 사람, keepout A·B), S6 ↔ S4 (사람 + 계단 B keepout 누락)을 짝지어 본다.
서 있는 사람 기준의 P3i는 본 실험에 없어서 추가 실험의 보통 조건(wetfloor/mu_1.0의 S2, darkstair/void_0.0의 S4)을 쓴다.
"""
import json
import math
import os
import statistics as st
import sys
from math import comb

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from analyze import ROOT, load, pct  # noqa: E402

CONDS = ["P0", "P1", "P1i", "P2", "P3m", "P3", "P3i"]
PAIR = {"S5": "S2", "S6": "S4"}


def fisher(a, n1, b, n2):
    """두 비율 차이의 Fisher 정확 검정 (양측 p)."""
    K, N = a + b, n1 + n2
    pobs = comb(n1, a) * comb(n2, b) / comb(N, K)
    return min(1.0, sum(comb(n1, x) * comb(n2, K - x) / comb(N, K) for x in range(max(0, K - n2), min(K, n1) + 1)
                        if comb(n1, x) * comb(n2, K - x) / comb(N, K) <= pobs * (1 + 1e-9)))


def sign_p(k, n):
    """부호 검정 (양측 p): n쌍 중 한쪽이 나은 쌍이 k."""
    return min(1.0, 2 * sum(comb(n, i) for i in range(0, min(k, n - k) + 1)) / 2 ** n) if n else 1.0


def encounter(d, r):
    """사람과 마주친 정도: 로봇 중심 1 m 안에 사람이 있던 시간, 가장 가까이 온 거리 (추락 전까지)."""
    tr = [e for e in json.load(open(os.path.join(d, "track.json"))) if e["t"] <= r["time"]]
    near = [min((math.hypot(q[0] - e["gt"][0], q[1] - e["gt"][1]) for q in e.get("ped", [])), default=9.0) for e in tr]
    r["near_s"] = 0.1 * sum(x < 1.0 for x in near)
    r["near_min"] = min(near) if near else 9.0
    r["yields"] = sum(w["yield_n"] for w in r.get("walkers", []))
    return r


def moving():
    d = os.path.join(ROOT, "walkers")
    return [encounter(os.path.join(d, f"{r['scenario']}_{r['condition']}_{r['seed']}"), r) for r in load(d)]


def standing():
    rs = [r for r in load(os.path.join(ROOT, "runs")) if r["scenario"] in ("S2", "S4")]
    for sub, s in (("wetfloor/mu_1.0", "S2"), ("darkstair/void_0.0", "S4")):
        p = os.path.join(ROOT, sub)
        if os.path.isdir(p):
            rs += [r for r in load(p) if r["scenario"] == s and r["condition"] == "P3i"]
    return rs


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


def main():
    mv, sd = moving(), standing()
    sel = lambda rs, **kw: [r for r in rs if all(r[k] == v for k, v in kw.items())]  # noqa: E731
    print("# 움직이는 사람 실험 결과\n")
    print("서 있는 사람 무리(S2·S4) 대신, 6명이 시드마다 무작위 위치·시각에 나타나 복도·모퉁이·사무실 문 사이를 걸어 다닌다. "
          "계단·벽은 피해 걷고, 로봇 앞에선 멈춰 비켜 주고, 모퉁이 등에선 0–5초 서 있다가, 몇 구간 걸은 뒤 사라졌다 다른 곳에서 다시 나타난다.\n")
    print(f"회차: 움직이는 사람 {len(mv)}개, 서 있는 사람 기준 {len(sd)}개\n")

    print("## 1. 사람과 실제로 마주쳤나\n")
    print("| 시나리오 | 회차 | 사람이 로봇 1 m 안에 있던 시간 (중앙값, 최대) | 가장 가까이 온 거리 (중앙값) | 1 m 안까지 온 회차 | 사람이 비켜 준 횟수 (중앙값) |")
    print("|---|---|---|---|---|---|")
    for s in ("S5", "S6"):
        rs = sel(mv, scenario=s)
        if not rs:
            continue
        ns = [r["near_s"] for r in rs]
        print(f"| {s} | {len(rs)} | {st.median(ns):.1f} s ({max(ns):.1f} s) | {st.median(r['near_min'] for r in rs):.2f} m | "
              f"{sum(r['near_min'] < 1.0 for r in rs)}/{len(rs)} | {st.median(r['yields'] for r in rs):.0f} |")

    print("\n## 2. 시나리오 × 방식 (각 10회) — 서 있는 사람과 비교\n")
    print("| 방식 | S2 서 있는 사람 (모퉁이) | **S5 움직이는 사람** | S4 서 있는 사람 + 계단 B 누락 | **S6 움직이는 사람 + 계단 B 누락** |")
    print("|---|---|---|---|---|")
    for c in CONDS:
        print(f"| {c} | {cell(sel(sd, scenario='S2', condition=c))} | {cell(sel(mv, scenario='S5', condition=c))} | "
              f"{cell(sel(sd, scenario='S4', condition=c))} | {cell(sel(mv, scenario='S6', condition=c))} |")
    print("\n- 걸침: 떨어지진 않았지만 로봇 중심이 가장자리 15 cm 안까지 간 회차 (몸체가 허공에 걸침).")

    print("\n## 3. 방식별 합계 (S5+S6, 20회)\n")
    print("| 방식 | 추락률 (95% CI) | 걸침 | 도착률 | 가장자리 최소 (중앙값 / 최솟값) | 도착 시간 평균 | 서 있는 사람(S2+S4) 추락 |")
    print("|---|---|---|---|---|---|---|")
    for c in CONDS:
        rs = sel(mv, condition=c)
        if not rs:
            continue
        n = len(rs)
        f = sum(r["outcome"] == "fall" for r in rs)
        near = sum(1 for r in rs if r["outcome"] != "fall" and r["min_edge"] < 0.15)
        a = [r for r in rs if r["outcome"] == "arrived"]
        me = [r["min_edge"] for r in rs]
        ref = sel(sd, condition=c)
        fr = f"{sum(r['outcome'] == 'fall' for r in ref)}/{len(ref)}" if ref else "–"
        print(f"| {c} | {pct(f, n)} | {near} | {pct(len(a), n)} | {st.median(me):.2f} / {min(me):.2f} m | "
              f"{st.mean(r['time'] for r in a):.1f} s | {fr} |" if a else
              f"| {c} | {pct(f, n)} | {near} | {pct(0, n)} | {st.median(me):.2f} / {min(me):.2f} m | – | {fr} |")

    print("\n## 4. 현장 방식 vs 제안 방식 (S5+S6, 같은 시드 짝)\n")
    print("| 비교 | 추락 (Fisher p) | 짝: 현장만 추락 / 제안만 추락 (부호검정 p) | 도착 (Fisher p) | 가장자리 여유: 제안이 더 먼 쌍 (p, 중앙값 차) | 둘 다 도착한 쌍 시간 차 중앙값 (제안 − 현장) |")
    print("|---|---|---|---|---|---|")
    key = {(r["scenario"], r["condition"], r["seed"]): r for r in mv}
    for A in ("P1", "P1i"):
        for B in ("P3", "P3i"):
            ra, rb = sel(mv, condition=A), sel(mv, condition=B)
            if not ra or not rb:
                continue
            fa = sum(r["outcome"] == "fall" for r in ra)
            fb = sum(r["outcome"] == "fall" for r in rb)
            aa = sum(r["outcome"] == "arrived" for r in ra)
            ab = sum(r["outcome"] == "arrived" for r in rb)
            pairs = [(x, key[(x["scenario"], B, x["seed"])]) for x in ra if (x["scenario"], B, x["seed"]) in key]
            oa = sum(x["outcome"] == "fall" and y["outcome"] != "fall" for x, y in pairs)
            ob = sum(y["outcome"] == "fall" and x["outcome"] != "fall" for x, y in pairs)
            de = [y["min_edge"] - x["min_edge"] for x, y in pairs]
            far = sum(v > 0 for v in de)
            dt = [y["time"] - x["time"] for x, y in pairs if x["outcome"] == y["outcome"] == "arrived"]
            print(f"| {A} → {B} | {fa}/{len(ra)} vs {fb}/{len(rb)} (p = {fisher(fa, len(ra), fb, len(rb)):.3f}) | "
                  f"{oa} / {ob} (p = {sign_p(ob, oa + ob):.3f}) | {aa} vs {ab} (p = {fisher(aa, len(ra), ab, len(rb)):.3f}) | "
                  f"{far}/{len(de)} (p = {sign_p(far, sum(v != 0 for v in de)):.3f}, {st.median(de):+.2f} m) | "
                  + (f"{st.median(dt):+.1f} s ({len(dt)}쌍) |" if dt else "– |"))

    print("\n## 5. 추락·걸침 회차 상세\n")
    print("| 시나리오 | 방식 | 시드 | 결과 | 시각 | 위치 | 가장자리 최소 | 사람 1 m 안 시간 |")
    print("|---|---|---|---|---|---|---|---|")
    for r in sorted(mv, key=lambda r: (r["condition"], r["scenario"], r["seed"])):
        if r["outcome"] == "fall" or r["min_edge"] < 0.15:
            f = next((e for e in r["events"] if e["kind"] == "fall"), None)
            loc = f"({f['x']:.1f}, {f['y']:.1f})" if f else "–"
            print(f"| {r['scenario']} | {r['condition']} | {r['seed']} | {r['outcome']} | {r['time']:.0f} s | {loc} | "
                  f"{r['min_edge']:.2f} m | {r['near_s']:.1f} s |")

    fdir = os.path.join(ROOT, "walkers_faults")
    moved = []
    for dirpath, _, files in os.walk(fdir) if os.path.isdir(fdir) else []:
        if "fault_reason.txt" in files:
            rj = os.path.join(dirpath, "result.json")
            out = json.load(open(rj))["outcome"] if os.path.exists(rj) else "결과 없음"
            moved.append((os.path.relpath(dirpath, fdir), out, open(os.path.join(dirpath, "fault_reason.txt")).read().strip()))
    print(f"\n실험 장치 결함으로 다시 돌린 회차: {len(moved)}개 (원본은 results/walkers_faults/에 보관)")
    for p, out, why in sorted(moved):
        print(f"- {p}: 원래 결과 {out} — {why}")


if __name__ == "__main__":
    main()
