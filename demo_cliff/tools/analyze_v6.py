"""판정 기준 v6 확인 실험 분석 — 미리 정한 비교(tools/v6_plan.py)만 판정한다.

    python3 tools/analyze_v6.py [결과 폴더]           # 기본 results_<호스트>/v6
    python3 tools/analyze_v6.py --faults <묶음 폴더>   # 결함 회차 목록 (run_v6.sh가 쓴다)

- 검정 · 신뢰구간은 v5와 같다 (짝지은 McNemar 정확 검정, Holm 3개, Newcombe 방법 10).
- 판정 규칙 (기준 v6 5장):
  가설 1 (안전 실패): 앞이 낮고 보정 p < 0.05면 이점.
  가설 2 (비켜 줌) · 가설 3 (도착): 앞이 많고 보정 p < 0.05이고, 같은 회차들에서 앞의 안전 실패 수가 뒤보다 많지 않으면 이점.
  반대 방향으로 유의하면 불리. 성립 조건(4장)에 못 미치면 시험 안 됨.
- 실제 후진 거리 · 거부는 s8_backup_ref.measure와 같은 정의다.
- 장치 결함은 analyze_v5.real_faults와 같다 (find_faults + 카메라 0프레임).
"""
import json
import math
import os
import socket
import statistics
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from analyze_v2 import holm, mcnemar, safety_fail  # noqa: E402
from analyze_v3 import arrived, paired_ci, table2, voids  # noqa: E402
from analyze_v4 import signed_edge  # noqa: E402
from analyze_v5 import real_faults  # noqa: E402
from s8_backup_ref import measure  # noqa: E402
import v6_plan as V  # noqa: E402

SHARE = 0.95          # 요청 기록 · 기억 · ⑤ 계단 찍힘: 칸마다 19/20 이상
HAZARD_MIN = 10       # 위험 배치: d = 0.55에서 ③의 안전 실패 10/20 이상
GOAL_TOL = 0.10       # 목적지 위치: 가장자리 거리 중앙값이 d ± 0.10 m
REAR_LOOK = 1.5       # 기억 확인: 등 뒤 반원 1.5 m 안의 낭떠러지 칸
GT_TOPIC = "/ground_truth"
NAMES = {"P3y": "① 기억으로 물러남", "P3": "② 후진을 Nav2에", "P3n": "③ 기억 없음",
         "P1irb": "④ 앞·뒤 IR", "P2nav": "⑤ Nav2 깊이 레이어"}


def load(root, group):
    d = os.path.join(root, group)
    out = {}
    if not os.path.isdir(d):
        return out
    bad = {os.path.abspath(x) for x, _ in real_faults(d)}
    for name in os.listdir(d):
        p = os.path.join(d, name)
        rj = os.path.join(p, "result.json")
        if os.path.exists(rj) and os.path.abspath(p) not in bad:
            r = json.load(open(rj))
            r["dir"] = p
            out[(group, r["scenario"], r["condition"], r["seed"])] = r
    return out


_meas = {}


def meas(r):
    if r["dir"] not in _meas:
        _meas[r["dir"]] = measure(r)
    return _meas[r["dir"]]


def yielded(r):
    m = meas(r)
    return m is not None and m["moved"] >= V.YIELD_OK


METRICS = {"safety": safety_fail, "arrival": arrived, "yield": yielded}


def pick(data, groups, scens, cond):
    return {(g, s, k): r for (g, s, c, k), r in data.items() if g in groups and s in scens and c == cond}


# ---------- 성립 조건 기록 ----------
def rear_memory(r):
    """후진 요청 시각에 기억(cliff.jsonl의 낭떠러지 칸)이 등 뒤 반원 1.5 m 안에 있었나. 기록이 없으면 None."""
    bk = next((e for e in r["events"] if e["kind"] == "backup"), None)
    path = os.path.join(r["dir"], "cliff.jsonl")
    if bk is None or not os.path.exists(path):
        return None
    t_abs = r["t0_sim"] + bk["t"]
    meta, last = None, None
    for ln in open(path):
        rec = json.loads(ln)
        if rec.get("meta"):
            meta = rec
        elif rec.get("t", 1e18) <= t_abs:
            last = rec
    tr = json.load(open(os.path.join(r["dir"], "track.json")))
    p = min(tr, key=lambda q: abs(q["t"] - bk["t"]))
    if meta is None or last is None or not p.get("odom"):
        return False
    x, y, yaw = p["odom"]
    for code in last["lethal"]:
        ix, iy = divmod(code, 1000)
        cx, cy = meta["ox"] + (ix + 0.5) * meta["res"], meta["oy"] + (iy + 0.5) * meta["res"]
        dx, dy = cx - x, cy - y
        if dx * math.cos(yaw) + dy * math.sin(yaw) < 0 and math.hypot(dx, dy) <= REAR_LOOK:
            return True
    return False


def used_ground_truth(r):
    """① 로봇의 safe_backup 노드 구독 목록에 참값이 있나. 기록이 없으면 None."""
    path = os.path.join(r["dir"], "safe_backup.jsonl")
    if not os.path.exists(path):
        return None
    for ln in open(path):
        rec = json.loads(ln)
        if rec.get("kind") == "start":
            return GT_TOPIC in rec.get("subscriptions", [])
    return None


def cell_runs(data, g, s, c):
    return [r for (gg, ss, cc, _), r in data.items() if gg == g and ss == s and cc == c]


def gate(data, name, groups, scens, a, b):
    out = []
    if name in ("가설 1", "가설 2"):
        for g in groups:
            d0 = V.GROUPS[g]["yield_d"]
            allr = [r for (gg, *_), r in data.items() if gg == g]
            edges = [meas(r)["edge0"] for r in allr if meas(r)]
            if not edges or abs(statistics.median(edges) - d0) > GOAL_TOL:
                out.append(f"{g} 목적지 {statistics.median(edges) if edges else float('nan'):.2f} m")
            for c in (a, b):
                rr = cell_runs(data, g, "S8", c)
                n = sum(meas(r) is not None for r in rr)
                if rr and n < SHARE * len(rr):
                    out.append(f"{g} {c} 요청 {n}/{len(rr)}")
            if name == "가설 1" and d0 == 0.55:
                rr = cell_runs(data, g, "S8", "P3n")
                k = sum(map(safety_fail, rr))
                if k < HAZARD_MIN:
                    out.append(f"{g} 위험 배치 P3n {k}/{len(rr)}")
            if name == "가설 2":
                for c in ("P3y", "P3"):
                    rr = cell_runs(data, g, "S8", c)
                    n = sum(bool(rear_memory(r)) for r in rr)
                    if rr and n < SHARE * len(rr):
                        out.append(f"{g} {c} 기억 {n}/{len(rr)}")
        ys = [r for (gg, ss, cc, _), r in data.items() if gg.startswith("Y") and cc == "P3y"]
        gt = [used_ground_truth(r) for r in ys]
        if any(x is not False for x in gt):
            out.append("① 참값 미사용 확인 안 됨" if any(x is None for x in gt) else "① 참값 구독")
    if name == "가설 3":
        for s in scens:
            rr = cell_runs(data, "N", s, "P2nav")
            n = sum((r.get("drop_marks_max") or 0) > 0 for r in rr)
            if rr and n < SHARE * len(rr):
                out.append(f"⑤ 계단 찍힘 {s} {n}/{len(rr)}")
    return out


# ---------- 비교 · 판정 ----------
def compare(data, groups, scens, a, b, metric):
    A = {(g, s, k): r for (g, s, k), r in pick(data, groups, scens, a).items()}
    B = {(g, s, k): r for (g, s, k), r in pick(data, groups, scens, b).items()}
    f = METRICS[metric]
    pairs = sorted(set(A) & set(B))
    out = dict(ka=sum(map(f, A.values())), na=len(A), kb=sum(map(f, B.values())), nb=len(B), npair=len(pairs))
    if not pairs:
        return out
    t = table2(A, B, pairs, f)
    ts = table2(A, B, pairs, safety_fail)
    out.update(p=mcnemar(t[1], t[2]), ci=paired_ci(*t),
               sa=ts[0] + ts[1], sb=ts[0] + ts[2])            # 같은 짝 회차들의 안전 실패 수
    out["ka"], out["kb"] = t[0] + t[1], t[0] + t[2]
    out["na"] = out["nb"] = len(pairs)
    return out


def verdict(m, metric, pa):
    ra, rb = m["ka"] / m["na"], m["kb"] / m["nb"]
    better = ra < rb if metric == "safety" else ra > rb
    worse = ra > rb if metric == "safety" else ra < rb
    if better and pa < 0.05 and (metric == "safety" or m["sa"] <= m["sb"]):
        return "✅ 이점"
    if worse and pa < 0.05:
        return "❌ 불리"
    return "차이 검출 못 함"


def print_primary(data):
    print("| # | 비교 | 지표 | 짝 | 앞 | 뒤 | 차이 앞−뒤 (95 % CI) | p | 보정 p | 안전 실패 앞 / 뒤 | 성립 | 판정 |")
    print("|---|---|---|---:|---:|---:|---|---:|---:|---|---|---|")
    ms = [compare(data, g, s, a, b, met) for _, g, s, a, b, met in V.PRIMARY]
    gates = [gate(data, n, g, s, a, b) for n, g, s, a, b, _ in V.PRIMARY]
    adj = holm([m.get("p", 1.0) if not gt else 1.0 for m, gt in zip(ms, gates)])
    label = {"safety": "안전 실패", "yield": f"비켜 줌 (≥ {V.YIELD_OK:.2f} m)", "arrival": "도착"}
    for (n, g, s, a, b, met), m, gt, pa in zip(V.PRIMARY, ms, gates, adj):
        head = f"| {n} | {NAMES[a]} 대 {NAMES[b]} ({'·'.join(s)}) | {label[met]}"
        if "p" not in m:
            print(f"{head} | 0 | – | – | – | – | – | – | – | 데이터 없음 |")
            continue
        ci = m["ci"]
        print(f"{head} | {m['npair']} | {m['ka']}/{m['na']} | {m['kb']}/{m['nb']} | "
              f"{100 * (m['ka'] / m['na'] - m['kb'] / m['nb']):+.0f} %p ({100 * ci[0]:+.0f} ~ {100 * ci[1]:+.0f}) | "
              f"{m['p']:.3g} | {pa:.3g} | {m['sa']} / {m['sb']} | {'탈락: ' + ', '.join(gt) if gt else '통과'} | "
              f"{'시험 안 됨' if gt else verdict(m, met, pa)} |")
    print()


def backup_margin(r):
    """후진 중(요청 ~ 끝 + 0.2 s) 로봇 중심과 가장자리 사이 최소 거리 [m]."""
    bk = next((e for e in r["events"] if e["kind"] == "backup"), None)
    if bk is None:
        return None
    end = next((e for e in r["events"] if e["kind"] == "backup_end"), None)
    t1 = (end["t"] if end else r["time"]) + 0.2
    rects = voids(r.get("world", 1))
    tr = json.load(open(os.path.join(r["dir"], "track.json")))
    es = [signed_edge(p["gt"][0], p["gt"][1], rects) for p in tr if bk["t"] <= p["t"] <= t1]
    return min(es) if es else None


def med(vs, fmt="{:.3f}"):
    vs = [v for v in vs if v is not None]
    return fmt.format(statistics.median(vs)) if vs else "–"


def print_backup_table(data):
    print("| 묶음 | 로봇 | 회차 | 요청 기록 | 기억 있음 | 안전 실패 | 추락 | 후진 거부 | 비켜 줌 | 실제 후진 중앙값 (최대) | 후진 중 가장자리 최소 중앙값 (최소) | 목적지 가장자리 중앙값 |")
    print("|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|")
    for g in V.Y_ALL:
        for c in V.ROBOTS_Y:
            rr = cell_runs(data, g, "S8", c)
            if not rr:
                continue
            ms = [meas(r) for r in rr]
            mv = [m["moved"] for m in ms if m]
            mg = [backup_margin(r) for r in rr]
            mem = sum(bool(rear_memory(r)) for r in rr) if c in ("P3y", "P3", "P3n") else "–"
            print(f"| {g} | {NAMES[c]} | {len(rr)} | {sum(m is not None for m in ms)} | {mem} | {sum(map(safety_fail, rr))} | "
                  f"{sum(r['outcome'] == 'fall' for r in rr)} | {sum(bool(m and m['refused']) for m in ms)} | "
                  f"{sum(map(yielded, rr))} | {med(mv)} ({max(mv) if mv else float('nan'):.3f}) m | "
                  f"{med(mg)} ({min([x for x in mg if x is not None], default=float('nan')):.3f}) m | "
                  f"{med(m['edge0'] for m in ms if m)} m |")
    print()


def print_margin(data):
    rr = [r for (g, s, c, _), r in data.items() if g.startswith("Y") and c == "P3y"]
    mg = [backup_margin(r) for r in rr]
    bad = sum(1 for x in mg if x is not None and x < V.MARGIN_OK)
    ok = bad <= V.MARGIN_MAX_BAD
    print(f"① 로봇 회차 {len(rr)}개 중 후진 중 가장자리 {V.MARGIN_OK:.2f} m 아래: **{bad}회** "
          f"(기준 {V.MARGIN_MAX_BAD}회 이하) → {'충족' if ok else '넘음'}\n")
    plans = [r.get("yield_plan") for r in rr if r.get("yield_plan")]
    if plans:
        print("| 묶음 | 회차 | 계산한 갈 수 있는 거리 a 중앙값 | 실제 후진 목표 중앙값 |")
        print("|---|---:|---:|---:|")
        for g in V.Y_ALL:
            ps = [r["yield_plan"] for (gg, s, c, _), r in data.items() if gg == g and c == "P3y" and r.get("yield_plan")]
            if ps:
                print(f"| {g} | {len(ps)} | {med(p['a'] for p in ps)} m | {med(p['target'] for p in ps)} m |")
        print()


def print_nav2(data):
    print("| 시나리오 | 로봇 | 회차 | 도착 | 안전 실패 | ⑤ 계단 구멍 안쪽 찍힘 (회차) | 찍힌 칸 최대 중앙값 |")
    print("|---|---|---:|---:|---:|---:|---:|")
    for s in V.GROUPS["N"]["scens"]:
        for c in V.GROUPS["N"]["conds"]:
            rr = cell_runs(data, "N", s, c)
            if not rr:
                continue
            marks = [r.get("drop_marks_max") for r in rr]
            print(f"| {s} | {NAMES[c]} | {len(rr)} | {sum(map(arrived, rr))} | {sum(map(safety_fail, rr))} | "
                  f"{'–' if c != 'P2nav' else sum((m or 0) > 0 for m in marks)} | "
                  f"{'–' if c != 'P2nav' else med(marks, '{:.0f}')} |")
    print()


def print_faults(root):
    base = os.path.join(os.path.dirname(os.path.abspath(root)), "v6_faults")
    rows = []
    if os.path.isdir(base):
        for pass_name in sorted(os.listdir(base)):
            for name in sorted(os.listdir(os.path.join(base, pass_name))):
                d = os.path.join(base, pass_name, name)
                why = open(os.path.join(d, "fault_reason.txt")).read().strip()
                rj = os.path.join(d, "result.json")
                r = json.load(open(rj)) if os.path.exists(rj) else None
                rows.append((pass_name, name, why, r["outcome"] if r else "–", "예" if r and safety_fail(r) else "아니오"))
    print(f"다시 돌린 장치 결함 회차: {len(rows)}개\n")
    if rows:
        print("| 단계 | 회차 | 이유 | 원래 결과 | 안전 실패였나 |")
        print("|---|---|---|---|---|")
        for row in rows:
            print("| " + " | ".join(row) + " |")
        print()


def main():
    if sys.argv[1:2] == ["--faults"]:
        for d, why in real_faults(sys.argv[2]):
            print(f"{d}\t{why}")
        return
    args = [x for x in sys.argv[1:] if not x.startswith("--")]
    root = args[0] if args else os.path.join(HERE, "..", f"results_{socket.gethostname()}", "v6")
    data = {}
    for g in V.GROUPS:
        data.update(load(root, g))
    print(f"# 판정 기준 v6 — 확인 실험 판정\n\n결과 폴더: `{os.path.relpath(root)}`\n")
    print("## 주 가설 (Holm 보정 3개)\n")
    print_primary(data)
    print("## 후진 묶음 칸별 기록 (6-2 · 6-3 나란히 보고 포함, 판정 아님)\n")
    print_backup_table(data)
    print("## 부 기준: ① 로봇이 약속한 여유를 지키는가 (6-1)\n")
    print_margin(data)
    print("## 평상시 묶음 (가설 3 칸별)\n")
    print_nav2(data)
    print("## 장치 결함\n")
    print_faults(root)
    left = sum(len(real_faults(os.path.join(root, g))) for g in V.GROUPS if os.path.isdir(os.path.join(root, g)))
    print(f"남아 있는 장치 결함 회차 (분석에서 뺌): {left}")


if __name__ == "__main__":
    main()
