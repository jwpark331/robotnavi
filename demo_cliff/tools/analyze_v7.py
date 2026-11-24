"""판정 기준 v7 (odom 배율 오차) 분석 → stdout (markdown)

    python3 tools/analyze_v7.py results_<호스트>/v7            # 결과표 · 칸 판정 · 허용 오차
    python3 tools/analyze_v7.py --faults results_<호스트>/v7/<묶음>   # 장치 결함 회차 (run_v7.sh가 쓴다)

지표 정의는 v6와 같다 (analyze_v6.backup_margin, s8_backup_ref.measure, analyze_v2.safety_fail).
"""
import json
import math
import os
import statistics
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import analyze_v6 as A6  # noqa: E402
from analyze_v2 import safety_fail  # noqa: E402
from analyze_v5 import real_faults  # noqa: E402
import v7_plan as V  # noqa: E402


def odom_ratio(r):
    """후진 구간의 (odom 이동 / 참값 이동). 후진이 5 cm 미만이면 None."""
    bk = next((e for e in r["events"] if e["kind"] == "backup"), None)
    if bk is None:
        return None
    end = next((e for e in r["events"] if e["kind"] == "backup_end"), None)
    t1 = (end["t"] if end else r["time"]) + 0.2
    tr = [p for p in json.load(open(os.path.join(r["dir"], "track.json")))
          if bk["t"] <= p["t"] <= t1 and p.get("odom")]
    if len(tr) < 2:
        return None
    gt = math.hypot(tr[-1]["gt"][0] - tr[0]["gt"][0], tr[-1]["gt"][1] - tr[0]["gt"][1])
    od = math.hypot(tr[-1]["odom"][0] - tr[0]["odom"][0], tr[-1]["odom"][1] - tr[0]["odom"][1])
    return od / gt if gt >= 0.05 else None


def med(vs, fmt="{:.3f}"):
    vs = [v for v in vs if v is not None]
    return fmt.format(statistics.median(vs)) if vs else "–"


def load_all(root):
    data = {}
    for g in sorted(os.listdir(root)):
        if os.path.isdir(os.path.join(root, g)):
            data.update(A6.load(root, g))
    return data


def cell(data, g):
    return [r for (gg, s, c, k), r in data.items() if gg == g]


def main(root):
    data = load_all(root)
    groups = sorted({k[0] for k in data})
    print("# 판정 기준 v7 결과 — odom 배율 오차\n")
    print("| 묶음 | s | 회차 | 후진 요청 | odom/참값 (후진) | 성립 | 안전 실패 | 캐스터 선 침범 | 남긴 여유 중앙값 · 최저 | 실제 후진 중앙값 |")
    print("|---|---|---|---|---|---|---|---|---|---|")
    rows = {}
    base = {}
    for g in groups:
        rs = cell(data, g)
        if abs(rs[0].get("odom_scale", 1.0) - 1.0) < 1e-6:
            rr = [x for x in (odom_ratio(r) for r in rs) if x is not None]
            base[rs[0].get("world", 1)] = statistics.median(rr) if rr else None
    for g in groups:
        rs = cell(data, g)
        sc = rs[0].get("odom_scale", 1.0)
        req = sum(1 for r in rs if any(e["kind"] == "backup" for e in r["events"]))
        ratio = [odom_ratio(r) for r in rs]
        rmed = statistics.median([x for x in ratio if x is not None]) if any(x is not None for x in ratio) else None
        margins = [A6.backup_margin(r) for r in rs]
        fail = sum(1 for r in rs if safety_fail(r))
        caster = sum(1 for m in margins if m is not None and m < V.CASTER)
        moved = [m["moved"] for m in (A6.meas(r) for r in rs) if m]
        b = base.get(rs[0].get("world", 1))
        ok = (req >= math.ceil(0.95 * len(rs)) and (abs(sc - 1.0) < 1e-6 or (
            rmed is not None and b is not None and abs(rmed - sc * b) <= V.SCALE_TOL)))
        if abs(sc - 1.0) < 1e-6:
            ok = ok and fail == 0
        rows[g] = dict(scale=sc, n=len(rs), fail=fail, ok=ok)
        mn = min((m for m in margins if m is not None), default=None)
        print(f"| {g} | {sc:.2f} | {len(rs)} | {req} | {rmed:.3f} |" if rmed is not None else f"| {g} | {sc:.2f} | {len(rs)} | {req} | – |", end="")
        print(f" {'통과' if ok else '미달'} | {fail} | {caster} | {med(margins)} · {mn:.3f} | {med(moved)} |" if mn is not None
              else f" {'통과' if ok else '미달'} | {fail} | {caster} | – | {med(moved)} |")

    print("\n## 칸 판정 (두 월드 합)\n")
    print("| s | 회차 | 안전 실패 | 성립 | 판정 |")
    print("|---|---|---|---|---|")
    verdict = {}
    for s in V.SCALES:
        gs = [g for g, v in rows.items() if abs(v["scale"] - s) < 1e-6]
        if not gs:
            continue
        n = sum(rows[g]["n"] for g in gs)
        f = sum(rows[g]["fail"] for g in gs)
        ok = all(rows[g]["ok"] for g in gs) and len(gs) == 2
        v = "시험 안 됨" if not ok else ("버팀" if f <= V.HOLD_MAX else "무너짐")
        verdict[s] = v
        print(f"| {s:.2f} | {n} | {f} | {'통과' if ok else '미달'} | {v} |")

    def tol(side):
        best = 0.0
        for s in sorted((s for s in verdict if side(s)), key=lambda s: abs(1 - s)):
            if verdict[s] != "버팀":
                break
            best = abs(1 - s)
        return best

    if verdict:
        both = min(tol(lambda s: s <= 1.0), tol(lambda s: s >= 1.0))
        print(f"\n- 허용 오차 (양쪽): ±{both * 100:.0f} %")
        print(f"- 위험한 쪽(s < 1, 생각보다 더 움직임): {tol(lambda s: s <= 1.0) * 100:.0f} %")
        print(f"- 반대쪽(s > 1): {tol(lambda s: s >= 1.0) * 100:.0f} %")
        print("- 버팀 = 40회 중 안전 실패 ≤ 1 (실패율 95 % 단측 상한 약 11.6 %). '시험 안 됨' 칸에서 넓히기를 멈춘다.")


if __name__ == "__main__":
    if sys.argv[1] == "--faults":
        for d, why in real_faults(sys.argv[2]):
            print(f"{d}\t{why}")
    else:
        main(sys.argv[1])
