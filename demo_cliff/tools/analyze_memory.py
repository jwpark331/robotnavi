"""기억 시나리오(H1)·개선안 P3h(H2) 분석 — 판정은 '판정 기준 (2026-10-20)'에 미리 적은 규칙으로만 한다.

    python3 tools/analyze_memory.py > results/memory/analysis.md

주 지표: 안전 실패율 = 추락 또는 걸침(로봇 중심이 가장자리 15 cm 안, min_edge < 0.15)
부 지표: 도착률 (S8은 마지막 목적지인 사무실 기준)
판정: 안전 실패율이 낮고 Fisher 양측 p < 0.05, 도착률이 10 %p 넘게 낮지 않으면 '이점'.
      안전 실패율 차이가 유의하지 않고 도착률 차이가 10 %p 이내면 '동등'. 안전 실패율이 유의하게 높거나 도착률이 10 %p 넘게 낮으면 '불리'.
"""
import json
import os
import statistics as st
import sys
from math import comb

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from analyze import ROOT, load, pct  # noqa: E402

CONDS = ["P1", "P1i", "P2", "P3", "P3i", "P3h"]


def fisher(a, n1, b, n2):
    K, N = a + b, n1 + n2
    if n1 == 0 or n2 == 0:
        return 1.0
    pobs = comb(n1, a) * comb(n2, b) / comb(N, K)
    return min(1.0, sum(comb(n1, x) * comb(n2, K - x) / comb(N, K) for x in range(max(0, K - n2), min(K, n1) + 1)
                        if comb(n1, x) * comb(n2, K - x) / comb(N, K) <= pobs * (1 + 1e-9)))


def unsafe(r):
    return r["outcome"] == "fall" or r["min_edge"] < 0.15


def counts(rs):
    n = len(rs)
    return n, sum(unsafe(r) for r in rs), sum(r["outcome"] == "fall" for r in rs), sum(r["outcome"] == "arrived" for r in rs)


def verdict(test, ref):
    """test가 ref보다 이점 / 동등 / 불리 (미리 정한 규칙)."""
    n1, u1, _, a1 = counts(test)
    n2, u2, _, a2 = counts(ref)
    if not n1 or not n2:
        return "–", 1.0
    p = fisher(u1, n1, u2, n2)
    da = 100 * (a1 / n1 - a2 / n2)
    if u1 / n1 < u2 / n2 and p < 0.05 and da >= -10:
        v = "✅ 이점"
    elif u1 / n1 > u2 / n2 and p < 0.05 or da < -10:
        v = "❌ 불리"
    elif da >= -10:
        v = "＝ 동등"
    else:
        v = "❌ 불리"
    return f"{v} (안전 실패 {u1}/{n1} vs {u2}/{n2}, p = {p:.3f}; 도착 {100 * a1 / n1:.0f} % vs {100 * a2 / n2:.0f} %)", p


def cell(rs):
    if not rs:
        return "–"
    n, u, f, a = counts(rs)
    near = u - f
    s = f"안전 실패 {u}/{n}" + (f" (추락 {f}" + (f", 걸침 {near})" if near else ")") if u else "") + f" · 도착 {a}"
    return ("🔴 " if u else "") + s


def main():
    mem = load(os.path.join(ROOT, "memory"))
    dark = [r for r in load(os.path.join(ROOT, "darkstair", "void_1.0")) if r["scenario"] in ("S3", "S4")]
    main_runs = load(os.path.join(ROOT, "runs"))
    p3h_norm = load(os.path.join(ROOT, "p3h_normal"))
    sel = lambda rs, **kw: [r for r in rs if all(r[k] == v for k, v in kw.items())]  # noqa: E731

    print("# 기억 시나리오(H1)와 개선안 P3h(H2) 결과\n")
    print("판정 기준은 실험 전에 고정했다 (`보이지 않는 계단 — 판정 기준 (2026-10-20).md`). "
          "**주 지표: 안전 실패율(추락 + 걸침)**, 부 지표: 도착률.\n")

    print("## H1. 기억이 필요한 상황 (계단 A keepout 누락)\n")
    print("- **S7 계단 옆 목적지:** 계단 A 모서리 너머(가장자리에서 약 0.55 m)로 간다. 모서리를 돌 때 계단이 시야 밖으로 나간다.")
    print("- **S8 계단 옆에서 비켜 주기:** S7 지점에 도착하면 앞에 사람들이 나타나고, 로봇이 0.6–0.9 m 뒤로 물러난다"
          "(Nav2 BackUp, 등 뒤가 계단). 그다음 사무실로 간다.\n")
    print("| 방식 | S7 계단 옆 목적지 | S8 비켜 주기 | S8 물러나기가 멈췄나 (멈춤 / 끝까지) | 가장자리 최소 (중앙값 / 최솟값) |")
    print("|---|---|---|---|---|")
    for c in CONDS:
        s7, s8 = sel(mem, scenario="S7", condition=c), sel(mem, scenario="S8", condition=c)
        bk = [next((e.get("result") for e in r["events"] if e["kind"] == "backup_end"), None) for r in s8]
        me = [r["min_edge"] for r in s7 + s8]
        print(f"| {c} | {cell(s7)} | {cell(s8)} | {sum(b == 'stopped' for b in bk)} / {sum(b == 'succeeded' for b in bk)} | "
              + (f"{st.median(me):.2f} / {min(me):.2f} m |" if me else "– |"))
    print("\n- 안전 실패 = 추락 또는 걸침(로봇 중심이 가장자리 15 cm 안). 물러나다 가장자리를 넘은 로봇은 대개 z 기준 '추락' 전에 걸린 채 멈춘다.")

    print("\n### H1 판정 (S7+S8, 같은 시드 20회씩)\n")
    print("| 비교 | 판정 |")
    print("|---|---|")
    for B in ("P3", "P3i", "P3h"):
        for A in ("P2", "P1i", "P1"):
            v, _ = verdict(sel(mem, condition=B), sel(mem, condition=A))
            print(f"| {B} vs {A} | {v} |")
    print("\n**S8만 (물러나기 상황):**\n")
    print("| 비교 | 판정 |")
    print("|---|---|")
    for B in ("P3", "P3i", "P3h"):
        v, _ = verdict(sel(mem, condition=B, scenario="S8"), sel(mem, condition="P2", scenario="S8"))
        print(f"| {B} vs P2 | {v} |")

    print("\n## H2. 개선안 P3h ('값 없음' 주의 칸을 비싼 길로 Nav2에 보냄)\n")
    print("| 상황 | P1i (현장 표준) | P2 | P3 | P3i | **P3h** |")
    print("|---|---|---|---|---|---|")
    dark_ref = {"P1i": sel(main_runs, condition="P1i", scenario="S3") + sel(main_runs, condition="P1i", scenario="S4")}
    for c in ("P2", "P3", "P3i", "P3h"):
        dark_ref[c] = sel(dark, condition=c)
    norm = {c: sel(main_runs, condition=c) for c in ("P1i", "P2", "P3")}
    norm["P3h"] = p3h_norm
    print("| 검은 계단 (S3·S4, 결측 100 %) | " + " | ".join(cell(dark_ref.get(c, [])) for c in ("P1i", "P2", "P3", "P3i", "P3h")) + " |")
    print("| 보통 (S1–S4) | " + " | ".join(cell(norm.get(c, [])) if c != "P3i" else "(S2–S4만 30회)" for c in ("P1i", "P2", "P3", "P3i", "P3h")) + " |")
    na = {c: (100 * counts(v)[3] / counts(v)[0] if v else float("nan")) for c, v in norm.items()}
    print("\n### H2 판정\n")
    v1, p1 = verdict(dark_ref["P3h"], dark_ref["P3"])
    print(f"- 예측 1 — 검은 계단에서 P3보다 안전 실패율이 유의하게 낮다: {v1}")
    ok2 = na.get("P3h", 0) >= na.get("P3", 0) - 10
    print(f"- 예측 2 — 보통 상황 도착률이 P3보다 10 %p 넘게 낮지 않다: P3h {na.get('P3h', float('nan')):.0f} % vs P3 {na['P3']:.0f} % → "
          + ("✅ 충족" if ok2 else "❌ 미충족"))
    print(f"- **H2 {'지지' if (v1.startswith('✅') and ok2) else '불지지'}**")
    print("\n참고 비교 (같은 규칙):")
    for ref in ("P1i", "P2", "P3i"):
        v, _ = verdict(dark_ref["P3h"], dark_ref[ref])
        print(f"- 검은 계단 P3h vs {ref}: {v}")
    for ref in ("P1i", "P2"):
        v, _ = verdict(norm["P3h"], norm[ref])
        print(f"- 보통 P3h vs {ref}: {v}")

    fdir = os.path.join(ROOT, "memory_faults")
    moved = []
    for dirpath, _, files in os.walk(fdir) if os.path.isdir(fdir) else []:
        if "fault_reason.txt" in files:
            rj = os.path.join(dirpath, "result.json")
            r = json.load(open(rj)) if os.path.exists(rj) else None
            moved.append((os.path.relpath(dirpath, fdir), r, open(os.path.join(dirpath, "fault_reason.txt")).read().strip()))
    bad = [(p, r) for p, r, _ in moved if r and unsafe(r)]
    print(f"\n실험 장치 결함으로 다시 돌린 회차: {len(moved)}개 (원본은 results/memory_faults/에 보관). "
          f"그중 안전 실패(추락·걸침)였던 회차: {len(bad)}개" + (" → " + ", ".join(f"{p} ({r['outcome']}, {r['min_edge']:.2f} m)" for p, r in bad) if bad else ""))
    for p, r, why in sorted(moved, key=lambda x: x[0]):
        print(f"- {p}: 원래 결과 {r['outcome'] if r else '결과 없음'} — {why}")


if __name__ == "__main__":
    main()
