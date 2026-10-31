"""판정 기준 v3 확인 실험 분석 — 미리 정한 비교(tools/v3_plan.py)만 판정한다.

    python3 tools/analyze_v3.py [결과 폴더]      # 기본 results_<호스트>/v3

- 판정 규칙은 v2와 같다 (analyze_v2.verdict). 같은 시나리오 · 시드끼리 McNemar 정확 검정(양측), 짝이 깨지면 Fisher.
- v2와 다른 점:
  ① 시험 성립 조건: 칸마다 회차의 80 % 이상이 가장자리 1.0 m 안, 속도를 올린 칸은 접근 속도 중앙값이 설정값의 70 % 이상.
     못 넘으면 "시험 안 됨"이고, Holm 계산에는 p = 1로 넣는다.
  ② 차이의 신뢰구간은 짝을 반영한다 (Newcombe 1998 방법 10). 판정은 검정으로 한다.
  ③ 가짜 도착(arrived인데 목표에서 1.5 m 넘게 떨어짐)은 결함이 아니라 미도착으로 센다. 안전 판정은 그대로 둔다.
"""
import importlib.util
import json
import math
import os
import socket
import statistics
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from analyze_v2 import fisher, holm, mcnemar, newcombe, safety_fail, verdict, wilson  # noqa: E402
from find_faults import faults  # noqa: E402
import v3_plan as V  # noqa: E402

GOAL_OK = 1.5                 # 이보다 목표에서 멀면 가짜 도착
NEAR_EDGE, NEAR_SHARE = 1.0, 0.8
SPEED_SHARE = 0.7
BAND = (0.6, 1.0)             # 접근 속도를 재는 구간 [m]
TELEPORT = 2.5                # 이보다 빠른 구간은 순간이동(S3 밀림)이라 뺀다 [m/s]
IR_NEAR = 0.6                 # 로봇 중심이 계단에서 이 안일 때 시작한 IR 정지를 '계단 근처'로 센다 [m]
_voids = {}


def voids(world):
    """월드의 계단 사각형들. geometry.py가 환경변수로 월드를 고르므로 월드마다 따로 읽는다."""
    if world not in _voids:
        os.environ["CLIFF_WORLD"] = str(world)
        path = os.path.join(HERE, "..", "ws", "src", "cliff_demo", "cliff_demo", "geometry.py")
        spec = importlib.util.spec_from_file_location(f"geometry_w{world}", path)
        m = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(m)
        _voids[world] = [v[:4] for v in m.VOIDS.values()]
    return _voids[world]


def edge(x, y, rects):
    return min(math.hypot(max(a - x, 0, x - c), max(b - y, 0, y - d)) for a, b, c, d in rects)


def arrived(r):
    return r["outcome"] == "arrived" and (r.get("goal_err") or 0) <= GOAL_OK


def track(r):
    return json.load(open(os.path.join(r["dir"], "track.json")))


def approach_speed(tr, rects):
    """계단에서 1.0–0.6 m 구간의 평균 속도 (참값 위치). 그 구간에 들어가지 않은 회차는 None."""
    vs = []
    for p, q in zip(tr, tr[1:]):
        dt = q["t"] - p["t"]
        if dt <= 0:
            continue
        v = math.hypot(q["gt"][0] - p["gt"][0], q["gt"][1] - p["gt"][1]) / dt
        if v <= TELEPORT and BAND[0] <= edge(p["gt"][0], p["gt"][1], rects) < BAND[1]:
            vs.append(v)
    return sum(vs) / len(vs) if vs else None


def near_ir_stop(r, tr, rects):
    """매트 밖 IR 정지가 계단 근처에서 시작됐나. 풀리지 않은 정지는 ir.jsonl에 없어 추락 회차에서는 적게 잡힐 수 있다."""
    path = os.path.join(r["dir"], "ir.jsonl")
    if not os.path.exists(path) or not tr:
        return False
    for ln in open(path):
        for start, _, on_mat in json.loads(ln)["stops"]:
            t = start - (r.get("t0_sim") or 0)
            p = min(tr, key=lambda q: abs(q["t"] - t))
            if not on_mat and abs(p["t"] - t) < 2 and edge(p["gt"][0], p["gt"][1], rects) < IR_NEAR:
                return True
    return False


def real_faults(d):
    return [(x, why) for x, why in faults(d) if not why.startswith("가짜 도착")]


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
            out[(r["scenario"], r["condition"], r["seed"])] = r
    return out


def paired_ci(both, a_only, b_only, neither, z=1.96):
    """짝지은 두 비율 차이 p_A − p_B의 95 % 신뢰구간 (Newcombe 1998 방법 10)."""
    n = both + a_only + b_only + neither
    ka, kb = both + a_only, both + b_only
    p1, p2 = ka / n, kb / n
    l1, u1 = wilson(ka, n, z)
    l2, u2 = wilson(kb, n, z)
    den = ka * (n - ka) * kb * (n - kb)
    phi = (both * neither - a_only * b_only) / math.sqrt(den) if den else 0.0
    d = p1 - p2
    return (d - math.sqrt(max(0.0, (p1 - l1) ** 2 - 2 * phi * (p1 - l1) * (u2 - p2) + (u2 - p2) ** 2)),
            d + math.sqrt(max(0.0, (u1 - p1) ** 2 - 2 * phi * (u1 - p1) * (p2 - l2) + (p2 - l2) ** 2)))


def table2(A, B, pairs, f):
    both = sum(f(A[x]) and f(B[x]) for x in pairs)
    a_only = sum(f(A[x]) and not f(B[x]) for x in pairs)
    b_only = sum(f(B[x]) and not f(A[x]) for x in pairs)
    return both, a_only, b_only, len(pairs) - both - a_only - b_only


def compare(data_a, data_b, scens, a, b, metric):
    A = {(s, k): r for (s, c, k), r in data_a.items() if c == a and s in scens}
    B = {(s, k): r for (s, c, k), r in data_b.items() if c == b and s in scens}
    f = safety_fail if metric == "safety" else arrived
    pairs = sorted(set(A) & set(B))
    ka, na = sum(map(f, A.values())), len(A)
    kb, nb = sum(map(f, B.values())), len(B)
    arr_a, arr_b = sum(map(arrived, A.values())), sum(map(arrived, B.values()))
    if not (na and nb):
        return dict(ka=ka, na=na, kb=kb, nb=nb)
    if pairs and len(pairs) == na == nb:
        t = table2(A, B, pairs, f)
        p, test = mcnemar(t[1], t[2]), "McNemar"
        ci, arr_ci = paired_ci(*t), paired_ci(*table2(A, B, pairs, arrived))
    else:
        p, test = fisher(ka, na - ka, kb, nb - kb), "Fisher (짝 불완전)"
        ci, arr_ci = newcombe(ka, na, kb, nb), newcombe(arr_a, na, arr_b, nb)
    return dict(ka=ka, na=na, kb=kb, nb=nb, p=p, test=test, ci=ci, arr=(arr_a, na, arr_b, nb), arr_ci=arr_ci)


def cell_gate(data, scens, cond, world, vmax):
    """시험 성립 조건. 못 넘은 칸을 글로 돌려준다 (넘으면 빈 목록)."""
    rects, out = voids(world), []
    for s in scens:
        rs = [r for (ss, c, _), r in data.items() if ss == s and c == cond]
        if not rs:
            continue
        near = sum(r["min_edge"] < NEAR_EDGE for r in rs)
        if near < NEAR_SHARE * len(rs):
            out.append(f"근접 {s} {cond} {near}/{len(rs)}")
        if vmax:
            vs = [v for v in (approach_speed(track(r), rects) for r in rs) if v is not None]
            med = statistics.median(vs) if vs else 0.0
            if med < SPEED_SHARE * vmax:
                out.append(f"속도 {s} {cond} {med:.2f} m/s")
    return out


def group_info(g):
    c = V.GROUPS[g]
    return c.get("world", 1), c.get("vmax", 0)


def print_rows(rows, get, adjust):
    print("| # | 시나리오 | 속도 | A 대 B | 지표 | A | B | 차이 A−B (95 % CI) | 검정 | p | 보정 p | 도착 A / B (차이 CI) | 성립 | 판정 |")
    print("|---|---|---|---|---|---|---|---|---|---|---|---|---|---|")
    ms, gates = [], []
    for _, ga, gb, sc, a, b, met in rows:
        ms.append(compare(get(ga), get(gb), sc, a, b, met))
        gates.append(cell_gate(get(ga), sc, a, *group_info(ga)) + cell_gate(get(gb), sc, b, *group_info(gb)))
    ps = [m["p"] if "p" in m and not g else 1.0 for m, g in zip(ms, gates)]
    adj = holm(ps) if adjust else ps
    for (name, ga, gb, sc, a, b, met), m, g, pa in zip(rows, ms, gates, adj):
        vmax = group_info(ga)[1] or 0.45
        head = f"| {name} | {'·'.join(sc)} | {vmax} | {a} 대 {b} | {'안전 실패' if met == 'safety' else '도착'}"
        if "p" not in m:
            print(f"{head} | – | – | – | – | – | – | – | – | 데이터 없음 |")
            continue
        ci, (aa, an, ab, bn) = m["ci"], m["arr"]
        verdict_txt = "시험 안 됨" if g else verdict(m, met, pa)
        print(f"{head} | {m['ka']}/{m['na']} | {m['kb']}/{m['nb']} | "
              f"{100 * (m['ka'] / m['na'] - m['kb'] / m['nb']):+.0f} %p ({100 * ci[0]:+.0f} ~ {100 * ci[1]:+.0f}) | "
              f"{m['test']} | {m['p']:.4f} | {pa:.4f} | "
              f"{aa}/{an} · {ab}/{bn} ({100 * m['arr_ci'][0]:+.0f} ~ {100 * m['arr_ci'][1]:+.0f}) | "
              f"{'탈락: ' + ', '.join(g) if g else '통과'} | {verdict_txt} |")
    print()


def print_cells(get):
    print("| 묶음 | 시나리오 | 방식 | 회차 | 안전 실패 | 추락 | 도착 | 가장자리 최소 중앙값 | 계단 근처 IR 정지 회차 | 접근 속도 중앙값 | 도착 시간 중앙값 |")
    print("|---|---|---|---:|---:|---:|---:|---:|---:|---:|---:|")
    for g, c in V.GROUPS.items():
        rects = voids(c.get("world", 1))
        for s in c["scens"]:
            for p in c["conds"]:
                rs = [r for (ss, cc, _), r in get(g).items() if ss == s and cc == p]
                if not rs:
                    continue
                trs = [track(r) for r in rs]
                vs = [v for v in (approach_speed(t, rects) for t in trs) if v is not None]
                ts = [r["time"] for r in rs if arrived(r)]
                print(f"| {g} | {s} | {p} | {len(rs)} | {sum(map(safety_fail, rs))} | {sum(r['outcome'] == 'fall' for r in rs)} | "
                      f"{sum(map(arrived, rs))} | {statistics.median(r['min_edge'] for r in rs):.2f} m | "
                      f"{sum(near_ir_stop(r, t, rects) for r, t in zip(rs, trs))} | "
                      f"{f'{statistics.median(vs):.2f} m/s' if vs else '–'} | {f'{statistics.median(ts):.1f} s' if ts else '–'} |")
    print()


def print_faults(root):
    """결함으로 다시 돌린 회차를 모두 공개한다. 안전 실패였던 회차는 보수적 수치를 내는 데 쓴다."""
    base = os.path.join(os.path.dirname(os.path.abspath(root)), "v3_faults")
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
    args = [x for x in sys.argv[1:] if not x.startswith("--")]
    root = args[0] if args else os.path.join(HERE, "..", f"results_{socket.gethostname()}", "v3")
    cache = {}
    get = lambda g: cache.setdefault(g, load(root, g))  # noqa: E731
    print(f"# 판정 기준 v3 — 확인 실험 판정\n\n결과 폴더: `{os.path.relpath(root)}`\n")
    print("## 주 가설 (Holm 보정)\n")
    print_rows(V.PRIMARY, get, True)
    print("## 부 가설 · 참고 (보정 없음, 결론 문장에 쓰지 않는다)\n")
    print_rows(V.SECONDARY, get, False)
    print("## 칸별 수치 (참고)\n")
    print_cells(get)
    print("## 장치 결함\n")
    print_faults(root)
    left = sum(len(real_faults(os.path.join(root, g))) for g in V.GROUPS if os.path.isdir(os.path.join(root, g)))
    print(f"남아 있는 장치 결함 회차 (분석에서 뺌): {left}")


if __name__ == "__main__":
    main()
