"""피드백 1 방어 실험 분석 — 주 지표: 안전 실패율(추락 + 걸침), 부 지표: 도착률 (판정 기준 2026-10-20과 같은 규칙).

    python3 tools/analyze_fb1.py > results/fb1/analysis.md
"""
import json
import math
import os
import re
import statistics as st
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from analyze import ROOT, load  # noqa: E402
from analyze_memory import cell, counts, unsafe, verdict  # noqa: E402

FB = os.path.join(ROOT, "fb1")


def L(*parts, scen=None, cond=None):
    p = os.path.join(ROOT, *parts)
    rs = load(p) if os.path.isdir(p) else []
    return [r for r in rs if (scen is None or r["scenario"] in scen) and (cond is None or r["condition"] == cond)]


def last_log(d):
    p = os.path.join(d, "cliff.jsonl")
    if not os.path.exists(p):
        return {}
    ls = [json.loads(x) for x in open(p) if x.strip()][1:]
    return ls[-1] if ls else {}


def logs(root, rs):
    return [last_log(os.path.join(root, f"{r['scenario']}_{r['condition']}_{r['seed']}")) for r in rs]


def fp(root, rs):
    ls = [x for x in logs(root, rs) if x.get("conf")]
    return st.mean(x["conf"]["FP"] for x in ls) if ls else float("nan")


def tmean(rs):
    a = [r["time"] for r in rs if r["outcome"] == "arrived"]
    return f"{st.mean(a):.1f} s" if a else "–"


def main():
    print("# 피드백 1 방어 실험 결과\n")
    print("주 지표: 안전 실패율(추락 + 걸침, 걸침 = 중심이 가장자리 15 cm 안), 부 지표: 도착률. "
          "판정 규칙은 `판정 기준 (2026-10-20)`과 같다 (같은 시드, Fisher 양측 p < 0.05, 도착률 10 %p).\n")

    # ---------------- §2-1 ----------------
    print("## §2-1. 값이 안 나오는 낭떠러지 → P3+ (3상태 · 정지거리 속도 제한 · IR 기억 · 후진)\n")
    dark = {"P1i": L("runs", scen=("S3", "S4"), cond="P1i"), "P2": L("darkstair", "void_1.0", cond="P2"),
            "P3": L("darkstair", "void_1.0", cond="P3"), "P3i": L("darkstair", "void_1.0", cond="P3i"),
            "P3+": L("fb1", "dark", cond="P3p")}
    norm = {"P1i": L("runs", cond="P1i"), "P2": L("runs", cond="P2"), "P3": L("runs", cond="P3"), "P3+": L("fb1", "normal", cond="P3p")}
    s8 = {"P2": L("memory", scen=("S8",), cond="P2"), "P3": L("memory", scen=("S8",), cond="P3"),
          "P3i": L("memory", scen=("S8",), cond="P3i"), "P3+": L("fb1", "s8", cond="P3p")}
    print("| 상황 | " + " | ".join(dark) + " |")
    print("|---|" + "---|" * len(dark))
    print("| 검은 계단 (S3·S4) | " + " | ".join(cell(v) for v in dark.values()) + " |")
    print("| 보통 (S1–S4) | " + " | ".join(cell(norm.get(k, [])) if k in norm else "–" for k in dark) + " |")
    print("| 뒤로 비켜 주기 (S8) | " + " | ".join(cell(s8.get(k, [])) if k in s8 else "–" for k in dark) + " |")
    print("\n**판정 (미리 정한 규칙):**\n")
    for ref in ("P3", "P3i", "P1i", "P2"):
        v, _ = verdict(dark["P3+"], dark[ref])
        print(f"- 검은 계단 P3+ vs {ref}: {v}")
    for ref in ("P3", "P1i", "P2"):
        v, _ = verdict(norm["P3+"], norm[ref])
        print(f"- 보통 P3+ vs {ref}: {v}")
    print("\n**대가 (피드백: '검은 매트 위에서 느려진다 — 시간 손실을 정직하게 보고'):**\n")
    print("| 상황 | P3 도착 시간 평균 | P3+ 도착 시간 평균 | P3+ 속도 상한이 걸린 프레임 비율 (중앙값) | P3+ IR 기억 칸 (중앙값) |")
    print("|---|---|---|---|---|")
    for s in ("S1", "S2", "S3", "S4"):
        a = [r for r in norm["P3"] if r["scenario"] == s]
        b = [r for r in norm["P3+"] if r["scenario"] == s]
        lg = logs(os.path.join(FB, "normal"), b)
        cl = [x["cap_low"] / max(1, x.get("n", 1)) for x in lg if "cap_low" in x]
        ir = [x.get("ir_cells", 0) for x in lg]
        print(f"| {s} | {tmean(a)} | {tmean(b)} | " + (f"{100 * st.median(cl):.1f} %" if cl else "–") + " | "
              + (f"{st.median(ir):.0f}" if ir else "–") + " |")
    lgd = logs(os.path.join(FB, "dark"), dark["P3+"])
    if lgd:
        print(f"\n검은 계단 P3+: IR 기억 칸 중앙값 {st.median(x.get('ir_cells', 0) for x in lgd):.0f}, "
              f"속도 상한 최소값 중앙값 {st.median(x.get('cap', 0) for x in lgd):.2f} m/s (마지막 기록)")

    # ---------------- §2-3 ----------------
    print("\n## §2-3. 필터와 기억을 따로 바꾸는 2×2 설계\n")
    grid = {("단일 프레임", "기억 없음"): "P2", ("단일 프레임", "odom 기억"): "P2o",
            ("베이즈 필터", "기억 없음"): "P3n", ("베이즈 필터", "odom 기억"): "P3", ("베이즈 필터", "map 기억"): "P3m"}

    def runs_for(c, scens):
        if c in ("P2o", "P3n"):
            return L("fb1", "factorial", scen=scens, cond=c) + L("fb1", "s8", scen=scens, cond=c)
        if "S8" in scens:
            return L("memory", scen=scens, cond=c)
        return L("runs", scen=scens, cond=c)

    def root_for(c, scens):
        if c in ("P2o", "P3n"):
            return os.path.join(FB, "factorial") if "S8" not in scens else os.path.join(FB, "s8")
        return os.path.join(ROOT, "memory" if "S8" in scens else "runs")

    print("| 방식 | 필터 | 기억 | S1·S2 (평상시·사람) | S1·S2 가짜 낭떠러지 (회차 평균) | S8 뒤로 비켜 주기 |")
    print("|---|---|---|---|---|---|")
    for (f, m), c in grid.items():
        a = runs_for(c, ("S1", "S2"))
        b = runs_for(c, ("S8",))
        print(f"| {c} | {f} | {m} | {cell(a)} | {fp(root_for(c, ('S1', 'S2')), a):,.0f} | {cell(b) if b else '–'} |")
    print("\n**피드백의 예측:**\n")
    p2o, p2 = runs_for("P2o", ("S1", "S2")), runs_for("P2", ("S1", "S2"))
    v, _ = verdict(p2o, p2)
    print(f"- 예측 1 — P2o는 가짜 칸이 기억에 쌓여 막힌다 (→ '필터는 기억의 전제조건'): P2o vs P2 (S1·S2) {v}")
    p3n, p3 = runs_for("P3n", ("S8",)), runs_for("P3", ("S8",))
    v, _ = verdict(p3n, p3)
    print(f"- 예측 2 — P3n은 뒤로 움직일 때 떨어진다 (→ '기억이 필요하다'): P3n vs P3 (S8) {v}")
    v, _ = verdict(runs_for("P3n", ("S1", "S2")), runs_for("P3", ("S1", "S2")))
    print(f"- 참고 — 전진 주행(S1·S2)에서 P3n vs P3: {v}")

    # ---------------- §2-2 ----------------
    print("\n## §2-2. 상관된 오차: 가속할 때 차체가 들림 (기울기 3°/(m/s²) 주입 — 피드백의 1.5°는 판정 기준 아래라 효과가 없어 스트레스 값 사용)\n")
    print("| 방식 | 기울기 없음 (S1·S2) | 가짜 낭떠러지 | **기울기 주입 (S1·S2)** | 가짜 낭떠러지 | 도착 시간 (없음 → 주입) |")
    print("|---|---|---|---|---|---|")
    base = {"P2": (L("runs", scen=("S1", "S2"), cond="P2"), os.path.join(ROOT, "runs")),
            "P3": (L("runs", scen=("S1", "S2"), cond="P3"), os.path.join(ROOT, "runs")),
            "P3+": (L("fb1", "normal", scen=("S1", "S2"), cond="P3p"), os.path.join(FB, "normal"))}
    pit = {"P2": L("fb1", "pitch", cond="P2"), "P3": L("fb1", "pitch", cond="P3"), "P3+": L("fb1", "pitch", cond="P3p")}
    for c in ("P2", "P3", "P3+"):
        b, broot = base[c]
        p = pit[c]
        print(f"| {c} | {cell(b)} | {fp(broot, b):,.0f} | {cell(p)} | {fp(os.path.join(FB, 'pitch'), p):,.0f} | {tmean(b)} → {tmean(p)} |")
    lgp = logs(os.path.join(FB, "pitch"), pit["P3+"])
    if lgp:
        ok = [x["plane_ok"] / max(1, x["plane_n"]) for x in lgp if x.get("plane_n")]
        if ok:
            print(f"\nP3+ 지면 평면 추정 성공률 (회차 중앙값): {100 * st.median(ok):.0f} %")
    print("\n**판정:**\n")
    for c in ("P2", "P3", "P3+"):
        v, _ = verdict(pit[c], base[c][0])
        print(f"- {c}: 기울기 주입 vs 없음 — {v}")
    for ref in ("P2", "P3"):
        v, _ = verdict(pit["P3+"], pit[ref])
        print(f"- 기울기 주입 상태 P3+ vs {ref}: {v}")

    # ---------------- 결함 ----------------
    fdir = os.path.join(ROOT, "fb1_faults")
    moved = []
    for dirpath, _, files in os.walk(fdir) if os.path.isdir(fdir) else []:
        if "fault_reason.txt" in files:
            rj = os.path.join(dirpath, "result.json")
            r = json.load(open(rj)) if os.path.exists(rj) else None
            moved.append((os.path.relpath(dirpath, fdir), r, open(os.path.join(dirpath, "fault_reason.txt")).read().strip()))
    bad = [(p, r) for p, r, _ in moved if r and unsafe(r)]
    print(f"\n실험 장치 결함으로 다시 돌린 회차: {len(moved)}개 (원본은 results/fb1_faults/에 보관). 그중 안전 실패였던 회차: {len(bad)}개"
          + (" → " + ", ".join(f"{p} ({r['outcome']}, {r['min_edge']:.2f} m)" for p, r in bad) if bad else ""))
    for p, r, why in sorted(moved, key=lambda x: x[0]):
        print(f"- {p}: 원래 결과 {r['outcome'] if r else '결과 없음'} — {why}")

    # 결함으로 뺀 회차 중 'TF 경고가 시작되기 14초 넘게 전부터 이미 멈춰 있던' 회차는 장치 결함이 아니라 실제로 갇힌 실패로 본다
    print("\n### 보수적 재분류: TF 경고 전에 이미 갇혀 있던 결함 회차 → 실제 실패(미도착)로 셈\n")
    stuck = []
    for p, r, _ in moved:
        d = os.path.join(fdir, p)
        if not r or not os.path.exists(os.path.join(d, "launch.log")):
            continue
        lines = open(os.path.join(d, "launch.log"), errors="ignore").read().splitlines()
        st_ = lambda l: float(m.group(1)) if (m := re.search(r"\[(\d{10}\.\d+)\]", l)) else None  # noqa: E731
        go = next((st_(x) for x in lines if "Navigating to goal" in x), None)
        too = [st_(x) - go for x in lines if go and "Transform data too old" in x and st_(x)]
        if not too:
            continue
        tr = json.load(open(os.path.join(d, "track.json")))
        seg = [e for e in tr if too[0] - 15 <= e["t"] <= too[0]]
        if seg and seg[0]["t"] <= too[0] - 14:
            mv = max(math.hypot(e["gt"][0] - seg[0]["gt"][0], e["gt"][1] - seg[0]["gt"][1]) for e in seg)
            if mv < 0.10:
                stuck.append((p, r))
    for p, r in sorted(stuck):
        print(f"- {p}: {r['condition']} {r['outcome']}, 가장자리 최소 {r['min_edge']:.2f} m, 멈춘 위치 {r['final_gt']}")
    groups = {"P3+ 검은 계단": ("dark_", "P3p", dark["P3+"]), "P3+ 보통": ("normal_", "P3p", norm["P3+"]),
              "P3+ 기울기 주입": ("pitch_", "P3p", pit["P3+"])}
    print("\n| 묶음 | 안전 실패 | 도착률 (원래 → 재분류 포함) |")
    print("|---|---|---|")
    for name, (pre, c, rs) in groups.items():
        extra = [r for p, r in stuck if os.path.basename(p).startswith(pre) and r["condition"] == c]
        n, u, _, a_ = counts(rs)
        u2 = u + sum(unsafe(r) for r in extra)
        print(f"| {name} | {u}/{n} → {u2}/{n + len(extra)} | {100 * a_ / n:.0f} % → {100 * a_ / (n + len(extra)):.0f} % |")


if __name__ == "__main__":
    main()
