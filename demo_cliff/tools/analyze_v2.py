"""판정 기준 v2 확인 실험 분석 — 미리 정한 비교(tools/v2_plan.py)만 판정한다.

    python3 tools/analyze_v2.py [결과 폴더]      # 기본 results_<호스트>/v2
    python3 tools/analyze_v2.py <폴더> --pilot   # 사전 실험 폴더로 계산만 시험 (판정 아님)

- 같은 시나리오 · 시드끼리 짝지어 McNemar 정확 검정(양측). 짝이 다 안 맞으면 Fisher 정확 검정(양측)으로 바꾸고 그렇다고 적는다.
- 주 가설은 Holm 보정. 차이는 Newcombe(하이브리드 점수) 95 % 신뢰구간.
- 이점: 지표가 좋은 쪽이고 보정 후 p < 0.05, 도착률 차이(A − B) 신뢰구간 하한이 −20 %p보다 위.
  불리: 지표가 유의하게 나쁘거나 도착률 차이 신뢰구간 상한이 0 아래. 그 밖은 '차이 검출 못 함'("동등"이라고 쓰지 않는다).
- 장치 결함(find_faults.py, 결과 무관 규칙) 회차는 뺀다.
"""
import json
import math
import os
import socket
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from find_faults import faults  # noqa: E402
import v2_plan as V  # noqa: E402


def safety_fail(r):
    return r["outcome"] == "fall" or r["min_edge"] < 0.15


def arrived(r):
    return r["outcome"] == "arrived"


def load(root, group):
    d = os.path.join(root, group)
    out = {}
    if not os.path.isdir(d):
        return out
    bad = {os.path.abspath(x) for x, _ in faults(d)}
    for name in os.listdir(d):
        p = os.path.join(d, name)
        rj = os.path.join(p, "result.json")
        if os.path.exists(rj) and os.path.abspath(p) not in bad:
            r = json.load(open(rj))
            out[(r["scenario"], r["condition"], r["seed"])] = r
    return out


def binom_cdf(k, n):
    return sum(math.comb(n, i) for i in range(k + 1)) / 2 ** n


def mcnemar(b, c):
    n = b + c
    return 1.0 if n == 0 else min(1.0, 2 * binom_cdf(min(b, c), n))


def fisher(a, b, c, d):
    r1, r2, c1, n = a + b, c + d, a + c, a + b + c + d
    pr = lambda x: math.comb(r1, x) * math.comb(r2, c1 - x) / math.comb(n, c1)  # noqa: E731
    p0 = pr(a)
    return min(1.0, sum(pr(x) for x in range(max(0, c1 - r2), min(r1, c1) + 1) if pr(x) <= p0 * (1 + 1e-9)))


def wilson(k, n, z=1.96):
    if n == 0:
        return (0.0, 1.0)
    p = k / n
    d = 1 + z * z / n
    c = p + z * z / (2 * n)
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n))
    return ((c - h) / d, (c + h) / d)


def newcombe(k1, n1, k2, n2):
    """p1 − p2 의 95 % 신뢰구간 (Newcombe 하이브리드 점수, 독립 표본 — 짝을 무시해 보수적)."""
    p1, p2 = k1 / n1, k2 / n2
    l1, u1 = wilson(k1, n1)
    l2, u2 = wilson(k2, n2)
    d = p1 - p2
    return (d - math.sqrt((p1 - l1) ** 2 + (u2 - p2) ** 2), d + math.sqrt((u1 - p1) ** 2 + (p2 - l2) ** 2))


def holm(ps):
    order = sorted(range(len(ps)), key=lambda i: ps[i])
    adj, run = [0.0] * len(ps), 0.0
    for rank, i in enumerate(order):
        run = max(run, min(1.0, (len(ps) - rank) * ps[i]))
        adj[i] = run
    return adj


def compare(data, scens, a, b, metric):
    A = {(s, k): r for (s, c, k), r in data.items() if c == a and s in scens}
    B = {(s, k): r for (s, c, k), r in data.items() if c == b and s in scens}
    f = safety_fail if metric == "safety" else arrived
    pairs = sorted(set(A) & set(B))
    ka, na = sum(f(r) for r in A.values()), len(A)
    kb, nb = sum(f(r) for r in B.values()), len(B)
    if pairs and len(pairs) == na == nb:
        bb = sum(f(A[x]) and not f(B[x]) for x in pairs)
        cc = sum(f(B[x]) and not f(A[x]) for x in pairs)
        p, test = mcnemar(bb, cc), "McNemar"
    else:
        p, test = fisher(ka, na - ka, kb, nb - kb), "Fisher (짝 불완전)"
    arr_a, arr_b = sum(map(arrived, A.values())), sum(map(arrived, B.values()))
    return dict(ka=ka, na=na, kb=kb, nb=nb, p=p, test=test,
                ci=newcombe(ka, na, kb, nb) if na and nb else (float("nan"),) * 2,
                arr=(arr_a, na, arr_b, nb), arr_ci=newcombe(arr_a, na, arr_b, nb) if na and nb else (float("nan"),) * 2)


def verdict(m, metric, p_adj):
    better = (m["ka"] / m["na"] < m["kb"] / m["nb"]) if metric == "safety" else (m["ka"] / m["na"] > m["kb"] / m["nb"])
    lo, hi = m["arr_ci"]
    if better and p_adj < 0.05 and lo > -0.20:
        return "✅ 이점"
    if (not better and p_adj < 0.05 and m["ka"] != m["kb"]) or hi < 0:
        return "❌ 불리"
    return "차이 검출 못 함"


def main():
    args = [x for x in sys.argv[1:] if not x.startswith("--")]
    root = args[0] if args else os.path.join(HERE, "..", f"results_{socket.gethostname()}", "v2")
    cache = {}
    get = lambda g: cache.setdefault(g, load(root, g))  # noqa: E731
    title = "사전 실험 데이터로 계산 시험 (판정 아님)" if "--pilot" in sys.argv else "확인 실험 판정"
    print(f"# 판정 기준 v2 — {title}\n\n결과 폴더: `{os.path.relpath(root)}`\n")
    for label, rows, adjust in (("주 가설 (Holm 보정)", V.PRIMARY, True), ("부 가설 · 참고 (보정 없음)", V.SECONDARY, False)):
        ms = [compare(get(g), sc, a, b, met) for _, g, sc, a, b, met in rows]
        ok = [m["na"] > 0 and m["nb"] > 0 for m in ms]
        ps = holm([m["p"] for m, o in zip(ms, ok) if o]) if adjust else [m["p"] for m, o in zip(ms, ok) if o]
        it = iter(ps)
        print(f"## {label}\n")
        print("| # | 시나리오 | A 대 B | 지표 | A | B | 차이 A−B (95 % CI) | 검정 | p | 보정 p | 도착 A / B (차이 CI) | 판정 |")
        print("|---|---|---|---|---|---|---|---|---|---|---|---|")
        for (name, g, sc, a, b, met), m, o in zip(rows, ms, ok):
            if not o:
                print(f"| {name} | {'·'.join(sc)} | {a} 대 {b} | {met} | – | – | – | – | – | – | – | 데이터 없음 |")
                continue
            pa = next(it)
            ci = m["ci"]
            aa, an, ab, bn = m["arr"]
            print(f"| {name} | {'·'.join(sc)} | {a} 대 {b} | {'안전 실패' if met == 'safety' else '도착'} | {m['ka']}/{m['na']} | "
                  f"{m['kb']}/{m['nb']} | {100 * (m['ka'] / m['na'] - m['kb'] / m['nb']):+.0f} %p ({100 * ci[0]:+.0f} ~ {100 * ci[1]:+.0f}) | "
                  f"{m['test']} | {m['p']:.4f} | {pa:.4f} | {aa}/{an} · {ab}/{bn} ({100 * m['arr_ci'][0]:+.0f} ~ {100 * m['arr_ci'][1]:+.0f}) | "
                  f"{verdict(m, met, pa)} |")
        print()
    nf = 0
    for g in V.GROUPS:
        d = os.path.join(root, g)
        if os.path.isdir(d):
            nf += len(faults(d))
    print(f"남아 있는 장치 결함 회차 (분석에서 뺌): {nf}")


if __name__ == "__main__":
    main()
