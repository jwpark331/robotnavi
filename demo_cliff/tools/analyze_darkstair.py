"""검은 계단 실험 분석표 (마크다운) — results/darkstair + 비교 기준 results/runs.

    python3 tools/analyze_darkstair.py > results/darkstair/analysis.md

P0·P1·P1i는 깊이카메라를 안 쓰고, P2·P3의 결측 0 %는 본 실험과 같은 코드·설정이라 results/runs의 S3·S4 회차를 기준으로 쓴다.
"""
import json
import os
import statistics as st
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from analyze import ROOT, load, pct, wilson  # noqa: E402

DS = os.path.join(ROOT, "darkstair")
SCENS = ("S3", "S4")
SCEN_LABEL = {"S3": "S3 위치 착각 (keepout 어긋남)", "S4": "S4 표시 누락 (계단 B keepout 없음)"}


def conf_until_end(d, r):
    """판정 혼동행렬을 회차 끝(추락이면 추락 시각)까지만 센다. 추락 뒤에는 구덩이 속에서 기울어진 카메라가
    엉뚱한 판정을 쏟아내서 가짜 낭떠러지가 부풀려진다 (본 실험은 깊이카메라 조건이 떨어진 적이 없어 영향 없음)."""
    p = os.path.join(d, "cliff.jsonl")
    if not os.path.exists(p):
        return None
    ls = [json.loads(x) for x in open(p) if x.strip()]
    ls = [x for x in ls if not x.get("meta") and x["t"] <= r["t0_sim"] + r["time"]]
    return ls[-1]["conf"] if ls else None


def runs_at(v):
    """결측률 v에서의 회차들. 0 %는 본 실험(results/runs)에 P3i(결측 0 %) 대조를 더한다."""
    dirs = [os.path.join(DS, f"void_{v}")] + ([os.path.join(ROOT, "runs")] if v == "0.0" else [])
    rs = []
    for d in dirs:
        if not os.path.isdir(d):
            continue
        for r in load(d):
            if r["scenario"] in SCENS:
                r["conf"] = conf_until_end(os.path.join(d, f"{r['scenario']}_{r['condition']}_{r['seed']}"), r)
                rs.append(r)
    return rs


def cell(rs):
    if not rs:
        return "–"
    n = len(rs)
    f = sum(r["outcome"] == "fall" for r in rs)
    a = [r for r in rs if r["outcome"] == "arrived"]
    edge = sum(1 for r in rs if r["outcome"] not in ("fall", "arrived") and r["min_edge"] < 0.6)
    other = n - f - len(a) - edge
    s = f"추락 {f}/{n} · 도착 {len(a)}" + (f" · 갇힘 {edge}" if edge else "") + (f" · 멈춤 {other}" if other else "")
    if a:
        s += f" · {st.mean(r['time'] for r in a):.0f} s"
    return ("🔴 " if f else "") + s


def main():
    base, dark = runs_at("0.0"), runs_at("1.0")
    sel = lambda rs, **kw: [r for r in rs if all(r[k] == v for k, v in kw.items())]  # noqa: E731
    print("# 검은 계단 실험 결과\n")
    print("계단 구멍 안으로 들어간 깊이카메라 광선이 반사를 못 받는다(결측)고 보고, 그 비율을 바꿨다. "
          "S3·S4는 keepout이 어긋나거나 없어서 깊이카메라만이 계단을 막을 수 있는 상황이다.\n")
    print(f"회차: 결측 100 % {len(dark)}개, 결측 0 % 기준 {len(base)}개 (본 실험 S3·S4 + P3i 대조)\n")

    cols = [("P1", "0.0", "P1 keepout"), ("P1i", "0.0", "P1i keepout+IR"),
            ("P2", "0.0", "P2 · 보통 계단"), ("P2", "1.0", "P2 · **검은 계단**"),
            ("P3", "0.0", "P3 · 보통 계단"), ("P3", "1.0", "P3 · **검은 계단**"),
            ("P3i", "0.0", "P3i · 보통 계단"), ("P3i", "1.0", "P3i · **검은 계단**")]
    data = {"0.0": base, "1.0": dark}
    print("## 1. 시나리오 × 방식 (각 10회)\n")
    print("P1·P1i는 깊이카메라를 쓰지 않아 계단 색과 무관하다 (본 실험 결과).\n")
    print("| | " + " | ".join(c[2] for c in cols) + " |")
    print("|---|" + "---|" * len(cols))
    for s in SCENS:
        print(f"| {SCEN_LABEL[s]} | " + " | ".join(cell(sel(data[v], scenario=s, condition=c)) for c, v, _ in cols) + " |")

    print("\n## 2. 방식별 합계 (S3+S4, 20회)\n")
    print("| 방식 | 계단 | 추락률 (95% CI) | 걸침 | 도착률 | 가장자리 최소 (중앙값 / 최솟값) | 계단 칸 판정 (TP / 놓침 FN) | 가짜 낭떠러지 FP |")
    print("|---|---|---|---|---|---|---|---|")
    for c, v, _ in cols:
        rs = sel(data[v], condition=c)
        if not rs:
            continue
        n = len(rs)
        f = sum(r["outcome"] == "fall" for r in rs)
        a = sum(r["outcome"] == "arrived" for r in rs)
        me = [r["min_edge"] for r in rs]
        cs = [r["conf"] for r in rs if r["conf"]]
        conf = (f"{sum(x['TP'] for x in cs):,} / {sum(x['FN'] for x in cs):,}" if cs else "–")
        fp = f"{sum(x['FP'] for x in cs):,}" if cs else "–"
        near = sum(1 for r in rs if r["outcome"] != "fall" and r["min_edge"] < 0.15)
        print(f"| {c} | {'검은' if v == '1.0' else '보통'} | {pct(f, n)} | {near} | {pct(a, n)} | "
              f"{st.median(me):.2f} / {min(me):.2f} m | {conf} | {fp} |")

    print("\n- 걸침: 떨어지진 않았지만 로봇 중심이 가장자리 15 cm 안까지 간 회차 (반지름 25 cm라 몸체가 허공에 걸친 상태).")
    print("- 가짜 낭떠러지는 추락 시각까지만 셌다 (추락 뒤 구덩이 속 카메라 판정은 뺌).")
    print("\n## 3. 같은 시드끼리: 보통 계단 → 검은 계단에서 결과가 바뀐 회차\n")
    print("| 방식 | 시나리오 | 짝 수 | 새로 추락 | 추락 → 안 떨어짐 | 도착 → 도착 못 함 |")
    print("|---|---|---|---|---|---|")
    for c in ("P2", "P3", "P3i"):
        for s in SCENS:
            pairs = [(a, b) for a in sel(base, condition=c, scenario=s) for b in sel(dark, condition=c, scenario=s)
                     if a["seed"] == b["seed"]]
            if not pairs:
                continue
            nf = [b["seed"] for a, b in pairs if b["outcome"] == "fall" and a["outcome"] != "fall"]
            fx = [b["seed"] for a, b in pairs if a["outcome"] == "fall" and b["outcome"] != "fall"]
            na = [b["seed"] for a, b in pairs if a["outcome"] == "arrived" and b["outcome"] != "arrived"]
            fmt = lambda xs: f"{len(xs)} (시드 {', '.join(map(str, sorted(xs)))})" if xs else "0"  # noqa: E731
            print(f"| {c} | {s} | {len(pairs)} | {fmt(nf)} | {fmt(fx)} | {fmt(na)} |")

    print("\n## 4. 결측률 sweep (P3, S3·S4, 시드 1–3) — 몇 %부터 무너지나\n")
    print("| 계단 결측률 | S3 | S4 | 합계 추락 | 가장자리 최소 평균 |")
    print("|---|---|---|---|---|")
    for v in ("0.0", "0.5", "0.7", "0.9", "1.0"):
        rs = [r for r in runs_at(v) if r["condition"] == "P3" and r["seed"] in (1, 2, 3)]
        if not rs:
            continue
        f = sum(r["outcome"] == "fall" for r in rs)
        print(f"| {100 * float(v):.0f} % | {cell(sel(rs, scenario='S3'))} | {cell(sel(rs, scenario='S4'))} | "
              f"{'🔴 ' if f else ''}{f}/{len(rs)} | {st.mean(r['min_edge'] for r in rs):.2f} m |")

    print("\n## 5. 하향 IR을 더한 P3i의 급정지\n")
    print("| 계단 | 시나리오 | 급정지 (회차 평균) | 정지 시간 합 평균 | 매트 위 급정지 | 추락 |")
    print("|---|---|---|---|---|---|")
    for v in ("0.0", "1.0"):
        for s in SCENS:
            rs = sel(data[v], condition="P3i", scenario=s)
            if not rs:
                continue
            ns = [len(r["ir_stops"]) for r in rs]
            ts = [sum(b - a for a, b, _ in r["ir_stops"]) for r in rs]
            mats = sum(sum(1 for *_, m in r["ir_stops"] if m) for r in rs)
            print(f"| {'검은' if v == '1.0' else '보통'} | {s} | {st.mean(ns):.1f} | {st.mean(ts):.1f} s | {mats} | "
                  f"{sum(r['outcome'] == 'fall' for r in rs)}/{len(rs)} |")

    fdir = os.path.join(ROOT, "darkstair_faults")
    moved = []
    for dirpath, _, files in os.walk(fdir) if os.path.isdir(fdir) else []:
        if "fault_reason.txt" in files:
            moved.append((os.path.relpath(dirpath, fdir), open(os.path.join(dirpath, "fault_reason.txt")).read().strip()))
    print(f"\n실험 장치 결함으로 다시 돌린 회차: {len(moved)}개 (원본은 results/darkstair_faults/에 보관)")
    for p, why in sorted(moved):
        print(f"- {p}: {why}")


if __name__ == "__main__":
    main()
