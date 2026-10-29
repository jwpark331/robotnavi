"""결과 분석표 (마크다운) — results/runs + results/sweep.

    python3 tools/analyze.py            # 화면에
    python3 tools/analyze.py > results/analysis.md
"""
import json
import math
import os
import statistics as st
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "ws", "src", "cliff_demo"))
from cliff_demo import geometry as G  # noqa: E402

ROOT = os.environ.get("RESULTS_DIR") or os.path.join(HERE, "..", "results")   # 다른 결과 폴더: RESULTS_DIR=...
BAD = ("infra_fault", "sim_stalled")
sys.path.insert(0, HERE)
from find_faults import faults  # noqa: E402  (장치 결함 판정을 리포트·분석에서 똑같이 쓴다)


def load(d):
    out = []
    if not os.path.isdir(d):
        return out
    bad_dirs = {os.path.abspath(x) for x, _ in faults(d)}
    for name in sorted(os.listdir(d)):
        p = os.path.join(d, name)
        rj = os.path.join(p, "result.json")
        if not os.path.exists(rj) or os.path.abspath(p) in bad_dirs:
            continue
        r = json.load(open(rj))
        if r["outcome"] in BAD:
            continue
        tr = json.load(open(os.path.join(p, "track.json")))
        r["amcl_max"] = max([math.hypot(e["gt"][0] - e["est"][0], e["gt"][1] - e["est"][1])
                             for e in tr if e["est"] and e["gt"][3] > -0.1] or [0.0])
        cj = os.path.join(p, "cliff.jsonl")
        r["conf"] = None
        if os.path.exists(cj):
            ls = [json.loads(x) for x in open(cj) if x.strip()]
            ls = [x for x in ls if not x.get("meta")]
            if ls:
                r["conf"] = ls[-1]["conf"]
        ij = os.path.join(p, "ir.jsonl")
        r["ir_stops"] = []
        if os.path.exists(ij):
            for x in open(ij):
                if x.strip():
                    r["ir_stops"] += json.loads(x)["stops"]
        out.append(r)
    return out


def wilson(k, n, z=1.96):
    if n == 0:
        return (0, 0)
    p = k / n
    d = 1 + z * z / n
    c = p + z * z / (2 * n)
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n))
    return (max(0.0, (c - h) / d), min(1.0, (c + h) / d))


def pct(k, n):
    lo, hi = wilson(k, n)
    return f"{k}/{n} ({100 * k / n:.0f} %, 95% CI {100 * lo:.0f}–{100 * hi:.0f} %)" if n else "–"


def main():
    runs = load(os.path.join(ROOT, "runs"))
    conds = [c for c in G.CONDITIONS if any(r["condition"] == c for r in runs)]
    scens = [s for s in G.SCENARIOS if any(r["scenario"] == s for r in runs)]
    sel = lambda **kw: [r for r in runs if all(r[k] == v for k, v in kw.items())]  # noqa: E731
    seeds = sorted({r["seed"] for r in runs})
    print(f"# 결과 분석\n\n회차 {len(runs)}개 · 시드 {seeds} · 조건 {conds}\n")

    print("## 1. 시나리오 × 조건 (추락 / 도착 / 갇힘·포기, 도착 평균 시간)\n")
    print("| | " + " | ".join(f"{c} {G.CONDITIONS[c]['label']}" for c in conds) + " |")
    print("|---|" + "---|" * len(conds))
    for s in scens:
        row = [f"**{s}** {G.SCENARIOS[s]['label']}"]
        for c in conds:
            rs = sel(scenario=s, condition=c)
            if not rs:
                row.append("–")
                continue
            n = len(rs)
            f = sum(r["outcome"] == "fall" for r in rs)
            a = [r for r in rs if r["outcome"] == "arrived"]
            t = f"{st.mean(r['time'] for r in a):.0f} s" if a else "–"
            st_edge = sum(1 for r in rs if r["outcome"] not in ("fall", "arrived") and r["min_edge"] < 0.6)
            st_else = n - f - len(a) - st_edge
            cell = (f"추락 {f}/{n} · 도착 {len(a)}/{n}" + (f" · 가장자리 앞 갇힘 {st_edge}" if st_edge else "")
                    + (f" · 다른 곳 멈춤 {st_else}" if st_else else "") + f" · {t}")
            row.append(("🔴 " if f else "") + cell)
        print("| " + " | ".join(row) + " |")

    print("\n실패 유형: 추락 / 가장자리 앞 갇힘(가장자리 0.6 m 안에서 Nav2가 더 못 감 — 안전 정지) / 다른 곳 멈춤(계단과 먼 곳)")
    print("\n## 2. 조건별 합계 (전 시나리오)\n")
    print("| 조건 | 추락률 | 도착률 | 가장자리 최소 (중앙값 / 최솟값) | AMCL 최대 오차 중앙값 |")
    print("|---|---|---|---|---|")
    for c in conds:
        rs = sel(condition=c)
        n = len(rs)
        f = sum(r["outcome"] == "fall" for r in rs)
        a = sum(r["outcome"] == "arrived" for r in rs)
        me = [r["min_edge"] for r in rs]
        print(f"| {c} {G.CONDITIONS[c]['label']} | {pct(f, n)} | {pct(a, n)} | {st.median(me):.2f} / {min(me):.2f} m | "
              f"{100 * st.median(r['amcl_max'] for r in rs):.0f} cm |")

    print("\n## 3. 같은 시드끼리 짝지은 비교 (둘 다 도착한 회차의 시간 차, 추락 차)\n")
    print("| 비교 | 짝 수 | 시간 차 평균 (B−A) | A만 추락 | B만 추락 |")
    print("|---|---|---|---|---|")
    for a_c, b_c in (("P1", "P3"), ("P1i", "P3"), ("P2", "P3"), ("P3m", "P3"), ("P1", "P1i")):
        if a_c not in conds or b_c not in conds:
            continue
        pairs = [(ra, rb) for ra in sel(condition=a_c) for rb in sel(condition=b_c)
                 if ra["scenario"] == rb["scenario"] and ra["seed"] == rb["seed"]]
        both = [(ra, rb) for ra, rb in pairs if ra["outcome"] == rb["outcome"] == "arrived"]
        dt = f"{st.mean(rb['time'] - ra['time'] for ra, rb in both):+.1f} s ({len(both)}쌍)" if both else "–"
        fa = sum(ra["outcome"] == "fall" and rb["outcome"] != "fall" for ra, rb in pairs)
        fb = sum(rb["outcome"] == "fall" and ra["outcome"] != "fall" for ra, rb in pairs)
        print(f"| {a_c} → {b_c} | {len(pairs)} | {dt} | {fa} | {fb} |")

    if "P1i" in conds:
        print("\n## 4. 하향 IR (P1i) 급정지\n")
        print("| 시나리오 | 급정지 횟수 (회차 평균) | 정지 시간 합 평균 | 매트 위 급정지 | 추락 | 가장자리 최소 |")
        print("|---|---|---|---|---|---|")
        for s in scens:
            rs = sel(scenario=s, condition="P1i")
            if not rs:
                continue
            ns = [len(r["ir_stops"]) for r in rs]
            ts = [sum(b - a for a, b, _ in r["ir_stops"]) for r in rs]
            mats = sum(sum(1 for *_, m in r["ir_stops"] if m) for r in rs)
            print(f"| {s} | {st.mean(ns):.1f} | {st.mean(ts):.1f} s | {mats} | {sum(r['outcome'] == 'fall' for r in rs)}/{len(rs)} | "
                  f"{min(r['min_edge'] for r in rs):.2f} m |")

    print("\n## 5. 깊이카메라 낭떠러지 판정 (셀-프레임 합계)\n")
    print("| 조건 | TP | FP (가짜) | FN | 정밀도 | 재현율 |")
    print("|---|---|---|---|---|---|")
    for c in conds:
        cs = [r["conf"] for r in sel(condition=c) if r["conf"]]
        if not cs:
            continue
        T = {k: sum(x[k] for x in cs) for k in ("TP", "FP", "FN", "TN")}
        P = T["TP"] / max(1, T["TP"] + T["FP"])
        R = T["TP"] / max(1, T["TP"] + T["FN"])
        print(f"| {c} | {T['TP']:,} | {T['FP']:,} | {T['FN']:,} | {100 * P:.1f} % | {100 * R:.1f} % |")

    sw = os.path.join(ROOT, "sweep")
    push = {}
    drop = {}
    if os.path.isdir(sw):
        for name in sorted(os.listdir(sw)):
            kind, _, v = name.partition("_")
            for r in load(os.path.join(sw, name)):
                (push if kind == "push" else drop).setdefault(float(v), []).append(r)
    if push:
        print("\n## 6. 밀림 거리 sweep (S3) — 추락 / 회차, 가장자리 최소 평균, AMCL 최대 오차 평균\n")
        pc = sorted({r["condition"] for rs in push.values() for r in rs}, key=list(G.CONDITIONS).index)
        print("| 밀림 [m] | " + " | ".join(pc) + " |")
        print("|---|" + "---|" * len(pc))
        for v in sorted(push):
            row = [f"{v:.1f}"]
            for c in pc:
                rs = [r for r in push[v] if r["condition"] == c]
                if not rs:
                    row.append("–")
                    continue
                f = sum(r["outcome"] == "fall" for r in rs)
                row.append(("🔴 " if f else "") + f"{f}/{len(rs)} · {st.mean(r['min_edge'] for r in rs):.2f} m · "
                           f"오차 {100 * st.mean(r['amcl_max'] for r in rs):.0f} cm")
            print("| " + " | ".join(row) + " |")
        for c in pc:
            fv = sorted(v for v, rs in push.items() if any(r["condition"] == c and r["outcome"] == "fall" for r in rs))
            print(f"\n- {c}: " + (f"처음 추락한 밀림 거리 {fv[0]:.1f} m" if fv else "모든 밀림 거리에서 추락 없음"))
    if drop:
        print("\n## 7. 매트 결측률 sweep (S1) — 가짜 낭떠러지 평균, 도착 / 회차, 도착 평균 시간\n")
        dc = sorted({r["condition"] for rs in drop.values() for r in rs}, key=list(G.CONDITIONS).index)
        print("| 결측률 | " + " | ".join(dc) + " |")
        print("|---|" + "---|" * len(dc))
        for v in sorted(drop):
            row = [f"{100 * v:.0f} %"]
            for c in dc:
                rs = [r for r in drop[v] if r["condition"] == c]
                if not rs:
                    row.append("–")
                    continue
                fp = st.mean((r["conf"] or {}).get("FP", 0) for r in rs)
                a = [r for r in rs if r["outcome"] == "arrived"]
                row.append(f"가짜 {fp:,.0f} · 도착 {len(a)}/{len(rs)}" + (f" · {st.mean(r['time'] for r in a):.0f} s" if a else ""))
            print("| " + " | ".join(row) + " |")

    ex = []
    for d in (os.path.join(ROOT, "runs"), sw):
        for dirpath, _, files in os.walk(d) if os.path.isdir(d) else []:
            if "result.json" in files and json.load(open(os.path.join(dirpath, "result.json")))["outcome"] in BAD:
                ex.append(os.path.relpath(dirpath, ROOT))
    print(f"\n실험 장치 결함으로 제외: {len(ex)}개 {ex if ex else ''}")


if __name__ == "__main__":
    main()
