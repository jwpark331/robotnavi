"""판정 기준 v4 확인 실험 분석 — 미리 정한 비교(tools/v4_plan.py)만 판정한다.

    python3 tools/analyze_v4.py [결과 폴더]      # 기본 results_<호스트>/v4
    python3 tools/analyze_v4.py --faults <묶음 폴더>   # 결함 회차 목록 (run_v4.sh가 쓴다)

- 판정 · 신뢰구간 · 성립 조건은 v3와 같다 (analyze_v2.verdict, analyze_v3.paired_ci, analyze_v3.cell_gate).
- v3와 다른 점:
  ① P3p 칸은 속도 조건에서 뺀다 (느려지는 것이 처치 자체다). 접근 속도는 참고로 적는다. F의 P3i 칸은 근접 · 속도 조건을 보지 않는다.
     A의 P2i · P3i 칸은 속도 조건을 같은 시드의 B24 P1ir 칸으로 대신 본다 (사전 시험 8번 미달 뒤의 결정, 기준 문서 3장).
  ② E에는 사람 근접 조건을 더한다: 칸마다 회차의 80 % 이상이 사람 중심과 1.0 m 안.
  ③ D의 주입 실패(camera_kill 기록 없음, 종료 확인 안 됨)도 장치 결함이다.
  ④ p는 과학 표기로 찍고, 부 가설 표에는 판정과 보정 p 칸을 두지 않는다.
"""
import json
import math
import os
import socket
import statistics
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from analyze_v2 import fisher, holm, mcnemar, newcombe, safety_fail, verdict  # noqa: E402
from analyze_v3 import IR_NEAR, NEAR_SHARE, approach_speed, arrived, cell_gate, near_ir_stop, paired_ci, table2, track, voids  # noqa: E402
from find_faults import faults  # noqa: E402
import v4_plan as V  # noqa: E402

PERSON_NEAR = 1.0             # 사람 근접 조건: 사람 중심과 이 안 [m]
CLOSE_DIST = 0.50             # 스침: 중심 거리가 이보다 작음 (틈 5 cm)
BAND_LIMIT = 0.30             # 금지 띠: 가장자리 최소가 이보다 작으면 침범
EDGE_FAIL = 0.15              # 걸침 기준
STOP_V = 0.1                  # 첫 정지: 속도가 이 아래로 두 번 연속 떨어진 때 [m/s]
CRUISE_SHARE = 0.8            # 설정 속도의 이 비율에 닿으면 달리는 중, 그 아래로 두 번 연속 떨어지면 감속 시작
TELEPORT = 2.5                # 이보다 빠른 구간은 순간이동(S3 밀림)이라 뺀다 [m/s]
METRICS = {"safety": safety_fail, "arrival": arrived, "contact": lambda r: (r.get("contact_n") or 0) > 0}


def real_faults(d):
    out = [(x, why) for x, why in faults(d) if not why.startswith("가짜 도착")]
    if V.GROUPS.get(os.path.basename(os.path.normpath(d)), {}).get("camkill") is not None:
        bad = {x for x, _ in out}
        for name in sorted(os.listdir(d)):
            p, rj = os.path.join(d, name), os.path.join(d, name, "result.json")
            if p in bad or not os.path.exists(rj):
                continue
            kills = [e for e in json.load(open(rj))["events"] if e["kind"] == "camera_kill"]
            if not (kills and kills[0]["dead"]):
                out.append((p, "주입 실패 (camera_kill 없음)" if not kills else "주입 실패 (종료 확인 안 됨)"))
    return out


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


def compare(data_a, data_b, scens, a, b, metric):
    A = {(s, k): r for (s, c, k), r in data_a.items() if c == a and s in scens}
    B = {(s, k): r for (s, c, k), r in data_b.items() if c == b and s in scens}
    f = METRICS[metric]
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


def gate(get, group, scens, cond):
    """시험 성립 조건. 못 넘은 칸을 글로 돌려준다. P3p 칸은 속도 조건을 빼고, speed_ref가 있는 묶음은 그 묶음의 P1ir 칸으로 본다."""
    c = V.GROUPS[group]
    data = get(group)
    ref = c.get("speed_ref")
    out = cell_gate(data, scens, cond, 1, 0 if cond == "P3p" or ref else c.get("vmax", 0))
    if ref:
        out += [f"{x} ({ref})" for x in cell_gate(get(ref), scens, "P1ir", 1, c["vmax"]) if x.startswith("속도")]
    if group.startswith("E"):
        for s in scens:
            rs = [r for (ss, cc, _), r in data.items() if ss == s and cc == cond]
            near = sum((r.get("min_person") or 9e9) < PERSON_NEAR for r in rs)
            if rs and near < NEAR_SHARE * len(rs):
                out.append(f"사람 근접 {s} {cond} {near}/{len(rs)}")
    return out


def label(group, cond):
    ir_r = V.GROUPS[group].get("ir_r")
    return f"{cond}({ir_r:.2f})" if ir_r is not None and cond == "P1ir" else cond


def speed_label(group):
    return f"{V.GROUPS[group].get('vmax') or 0.45} m/s"


def row_data(rows, get):
    ms, gates = [], []
    for _, ga, gb, sc, a, b, met in rows:
        ms.append(compare(get(ga), get(gb), sc, a, b, met))
        gates.append(sorted(set(gate(get, ga, sc, a) + gate(get, gb, sc, b))))
    return ms, gates


def print_primary(get):
    print("| # | 시나리오 | 속도 | 앞 대 뒤 | 앞 안전 실패 | 뒤 안전 실패 | 차이 앞−뒤 (95 % CI) | 검정 | p | 보정 p | 도착 앞 / 뒤 (차이 CI) | 성립 | 판정 |")
    print("|---|---|---|---|---|---|---|---|---|---|---|---|---|")
    ms, gates = row_data(V.PRIMARY, get)
    adj = holm([m["p"] if "p" in m and not g else 1.0 for m, g in zip(ms, gates)])
    for (name, ga, gb, sc, a, b, met), m, g, pa in zip(V.PRIMARY, ms, gates, adj):
        head = f"| {name} | {'·'.join(sc)} | {speed_label(ga)} | {label(ga, a)} 대 {label(gb, b)}"
        if "p" not in m:
            print(f"{head} | – | – | – | – | – | – | – | – | 데이터 없음 |")
            continue
        ci, (aa, an, ab, bn) = m["ci"], m["arr"]
        print(f"{head} | {m['ka']}/{m['na']} | {m['kb']}/{m['nb']} | "
              f"{100 * (m['ka'] / m['na'] - m['kb'] / m['nb']):+.0f} %p ({100 * ci[0]:+.0f} ~ {100 * ci[1]:+.0f}) | "
              f"{m['test']} | {m['p']:.3g} | {pa:.3g} | "
              f"{aa}/{an} · {ab}/{bn} ({100 * m['arr_ci'][0]:+.0f} ~ {100 * m['arr_ci'][1]:+.0f}) | "
              f"{'탈락: ' + ', '.join(g) if g else '통과'} | {'시험 안 됨' if g else verdict(m, met, pa)} |")
    print()


def print_a_old_rule(get):
    """A의 속도 조건은 사전 시험을 보고 바꿨다. 바꾸기 전 규칙(칸마다 70 %)으로 보면 어떤지 함께 찍는다."""
    c = V.GROUPS["A"]
    bad = [x for p in c["conds"] for x in cell_gate(get("A"), c["scens"], p, 1, c["vmax"]) if x.startswith("속도")]
    print(f"A를 사전 시험 전 규칙(P2i · P3i 칸마다 설정값의 70 %)으로 보면: {'시험 안 됨 (' + ', '.join(bad) + ')' if bad else '통과'}. "
          f"A의 판정은 실제 접근 속도가 아니라 최고 속도 설정에 관한 것이다 (칸별 표의 접근 속도를 함께 적는다).\n")


def print_secondary(get):
    print("| # | 앞 대 뒤 | 앞 접촉 | 뒤 접촉 | 차이 앞−뒤 (95 % CI) | 검정 | p | 성립 |")
    print("|---|---|---|---|---|---|---|---|")
    ms, gates = row_data(V.SECONDARY, get)
    for (name, ga, gb, _, a, b, _), m, g in zip(V.SECONDARY, ms, gates):
        head = f"| {name} | {label(ga, a)} {speed_label(ga)} 대 {label(gb, b)} {speed_label(gb)}"
        if "p" not in m:
            print(f"{head} | – | – | – | – | – | 데이터 없음 |")
            continue
        ci = m["ci"]
        print(f"{head} | {m['ka']}/{m['na']} | {m['kb']}/{m['nb']} | "
              f"{100 * (m['ka'] / m['na'] - m['kb'] / m['nb']):+.0f} %p ({100 * ci[0]:+.0f} ~ {100 * ci[1]:+.0f}) | "
              f"{m['test']} | {m['p']:.3g} | {'탈락: ' + ', '.join(g) if g else '통과'} |")
    print()


def signed_edge(x, y, rects):
    """중심에서 가장자리까지 거리. 구멍 안이면 음수 (result.json의 min_edge와 같은 부호)."""
    best = 9e9
    for a, b, c, d in rects:
        dx, dy = max(a - x, 0.0, x - c), max(b - y, 0.0, y - d)
        best = min(best, -min(x - a, c - x, y - b, d - y) if dx == dy == 0 else math.hypot(dx, dy))
    return best


def speeds(tr):
    """track의 구간 속도. 순간이동 구간은 None."""
    out = [None]
    for p, q in zip(tr, tr[1:]):
        dt = q["t"] - p["t"]
        v = math.hypot(q["gt"][0] - p["gt"][0], q["gt"][1] - p["gt"][1]) / dt if dt > 0 else 0.0
        out.append(v if v <= TELEPORT else None)
    return out


def first_intrusion(tr, rects):
    """복구 0회 상태에서 걸침 기준 안에 처음 든 때의 직전 0.5 s 최고 속도. 해당 없으면 None."""
    vs = speeds(tr)
    for i, p in enumerate(tr):
        if signed_edge(p["gt"][0], p["gt"][1], rects) < EDGE_FAIL:
            if p["rec"] != 0:
                return None
            w = [vs[k] for k in range(i + 1) if vs[k] is not None and tr[k]["t"] >= p["t"] - 0.5]
            return max(w) if w else 0.0
    return None


def ir_starts(r):
    path = os.path.join(r["dir"], "ir_start.jsonl")
    return [json.loads(ln) for ln in open(path)] if os.path.exists(path) else []


def ir_stop_speed(r, tr, rects):
    """계단 근처에서 처음 시작한 IR 정지의 직전 0.5 s 최고 속도. 없으면 None."""
    vs, t0 = speeds(tr), r.get("t0_sim") or 0
    for s in ir_starts(r):
        if s["on_mat"] or not s["gt"] or signed_edge(s["gt"][0], s["gt"][1], rects) >= IR_NEAR:
            continue
        w = [v for q, v in zip(tr, vs) if v is not None and s["t"] - t0 - 0.5 <= q["t"] <= s["t"] - t0]
        return max(w) if w else s["v"]
    return None


def first_stop(r, tr, rects):
    """F의 첫 정지 여유와 제동 거리. 설정 속도의 80 %에 닿은 뒤 처음 멈출 때까지 본다 (추락 회차는 추락 판정까지).
    track은 기록 간격이 흔들려 속도가 튀므로 두 번 연속으로 본다. 설정 속도의 80 %에 못 닿았으면 (None, None)."""
    vs = speeds(tr)
    cruise = CRUISE_SHARE * r["max_vel"]
    i0 = next((i for i, v in enumerate(vs) if v is not None and v >= cruise), None)
    if i0 is None:
        return None, None

    def low(i, lim):
        return all(v is not None and v < lim for v in vs[i:i + 2])

    margin, brake_edge = 9e9, None
    for i in range(i0, len(tr)):
        if tr[i]["t"] > r["time"]:
            break
        e = signed_edge(tr[i]["gt"][0], tr[i]["gt"][1], rects)
        margin = min(margin, e)
        if brake_edge is None and low(i, cruise):
            brake_edge = e
        if low(i, STOP_V):
            break
    start = next((s for s in ir_starts(r) if not s["on_mat"] and s["gt"]), None)
    if start:
        brake_edge = signed_edge(start["gt"][0], start["gt"][1], rects)
    return margin, brake_edge - margin if brake_edge is not None else None


def cap_share(r):
    """P3p에서 속도 상한(0.40 m/s 아래)이 걸린 프레임 비율. 없으면 None."""
    path = os.path.join(r["dir"], "cliff.jsonl")
    if not os.path.exists(path):
        return None
    last = None
    for ln in open(path):
        m = json.loads(ln)
        if "cap_low" in m:
            last = m
    return last["cap_low"] / last["n"] if last and last["n"] else None


def med(vs, fmt="{:.2f}"):
    vs = [v for v in vs if v is not None]
    return fmt.format(statistics.median(vs)) if vs else "–"


def print_cells(get):
    print("| 묶음 | 시나리오 | 방식 | 회차 | 안전 실패 | 추락 | 도착 | 가장자리 최소 중앙값 | 금지 띠 침범 | 첫 접근 침범 (속도 중앙값) | 계단 근처 IR 정지 회차 | IR 정지 직전 속도 | 접근 속도 중앙값 | 상한 걸린 프레임 | 사람 최소 거리 | 접촉 | 스침 | 스침 최고 속도 | 도착 시간 중앙값 |")
    print("|---|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|")
    for g, c in V.GROUPS.items():
        rects = voids(1)
        for s in c["scens"]:
            for p in c["conds"]:
                rs = [r for (ss, cc, _), r in get(g).items() if ss == s and cc == p]
                if not rs:
                    continue
                trs = [track(r) for r in rs]
                intr = [first_intrusion(t, rects) for t in trs]
                person = [r.get("min_person") for r in rs if r.get("min_person") is not None]
                close = [r["close_v"] for r in rs if (r.get("min_person") or 9e9) < CLOSE_DIST]
                print(f"| {g} | {s} | {label(g, p)} | {len(rs)} | {sum(map(safety_fail, rs))} | {sum(r['outcome'] == 'fall' for r in rs)} | "
                      f"{sum(map(arrived, rs))} | {statistics.median(r['min_edge'] for r in rs):.2f} m | "
                      f"{sum(r['min_edge'] < BAND_LIMIT for r in rs)} | "
                      f"{sum(v is not None for v in intr)} ({med(intr)} m/s) | "
                      f"{sum(near_ir_stop(r, t, rects) for r, t in zip(rs, trs))} | "
                      f"{med(ir_stop_speed(r, t, rects) for r, t in zip(rs, trs))} m/s | "
                      f"{med(approach_speed(t, rects) for t in trs)} m/s | "
                      f"{med((cap_share(r) for r in rs), '{:.0%}') if p == 'P3p' else '–'} | "
                      f"{f'{min(person):.3f} m' if person else '–'} | {sum(METRICS['contact'](r) for r in rs) if person else '–'} | "
                      f"{len(close) if person else '–'} | {f'{max(close):.2f} m/s' if close else '–'} | "
                      f"{med((r['time'] for r in rs if arrived(r)), '{:.1f} s')} |")
    print()


def print_stop_curve(get):
    print("| 설정 속도 | 방식 | 회차 | 안전 실패 | 추락 | 접근 속도 중앙값 | 첫 정지 여유 중앙값 (최소) | 제동 거리 중앙값 | 성립 |")
    print("|---:|---|---:|---:|---:|---:|---:|---:|---|")
    rects = voids(1)
    for g, c in V.GROUPS.items():
        if not g.startswith("F"):
            continue
        for p in c["conds"]:
            rs = [r for (_, cc, _), r in get(g).items() if cc == p]
            if not rs:
                continue
            trs = [track(r) for r in rs]
            stops = [first_stop(r, t, rects) for r, t in zip(rs, trs)]
            margins = [m for m, _ in stops if m is not None]
            bad = [] if p == "P3i" else gate(get, g, c["scens"], p)   # 카메라로 보고 멀리서 멈추는 것이 결과라 P3i는 조건을 보지 않는다
            print(f"| {c['vmax']} m/s | {label(g, p)} | {len(rs)} | {sum(map(safety_fail, rs))} | {sum(r['outcome'] == 'fall' for r in rs)} | "
                  f"{med(approach_speed(t, rects) for t in trs)} m/s | "
                  f"{f'{statistics.median(margins):.3f} ({min(margins):.3f}) m' if margins else '–'} | "
                  f"{med((b for _, b in stops), '{:.3f} m')} | {'탈락: ' + ', '.join(bad) if bad else '통과'} |")
    print()


def print_repeat(get):
    """C의 P1ir과 B24의 P1ir은 설정이 같다 (VOID는 cliff_detector에만 간다). 같은 시드의 재현 차이."""
    b, c = get("B24"), get("C")
    keys = sorted(k for k in b if k in c and k[1] == "P1ir")
    if keys:
        same = sum(safety_fail(b[k]) == safety_fail(c[k]) for k in keys)
        print(f"같은 설정 같은 시드의 안전 판정 일치: {same}/{len(keys)} (B P1ir(0.24) {sum(safety_fail(b[k]) for k in keys)}, C P1ir {sum(safety_fail(c[k]) for k in keys)})\n")


def print_faults(root):
    """결함으로 다시 돌린 회차를 모두 공개한다. 안전 실패였던 회차는 보수적 수치를 내는 데 쓴다."""
    base = os.path.join(os.path.dirname(os.path.abspath(root)), "v4_faults")
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
    root = args[0] if args else os.path.join(HERE, "..", f"results_{socket.gethostname()}", "v4")
    cache = {}
    get = lambda g: cache.setdefault(g, load(root, g))  # noqa: E731
    print(f"# 판정 기준 v4 — 확인 실험 판정\n\n결과 폴더: `{os.path.relpath(root)}`\n")
    print("## 주 가설 (Holm 보정)\n")
    print_primary(get)
    print_a_old_rule(get)
    print("## 부 가설 E: 접촉 회차 (보정 없음, 결론 문장에 쓰지 않는다)\n")
    print_secondary(get)
    print("## 부 가설 F: 정지 여유와 제동 거리 (판정 없음)\n")
    print_stop_curve(get)
    print("## 칸별 수치 (참고)\n")
    print_cells(get)
    print("## 같은 설정 재현 (참고)\n")
    print_repeat(get)
    print("## 장치 결함\n")
    print_faults(root)
    left = sum(len(real_faults(os.path.join(root, g))) for g in V.GROUPS if os.path.isdir(os.path.join(root, g)))
    print(f"남아 있는 장치 결함 회차 (분석에서 뺌): {left}")


if __name__ == "__main__":
    main()
