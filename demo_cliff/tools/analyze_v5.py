"""판정 기준 v5 확인 실험 분석 — 미리 정한 비교(tools/v5_plan.py)만 판정한다.

    python3 tools/analyze_v5.py [결과 폴더]           # 기본 results_<호스트>/v5
    python3 tools/analyze_v5.py --faults <묶음 폴더>   # 결함 회차 목록 (run_v5.sh가 쓴다)

- 검정 · 신뢰구간 · 판정은 v4와 같다 (analyze_v2.verdict, analyze_v3.paired_ci). B1 · B2는 목표가 구멍 안이라 도착 조건을 보지 않는다.
- 시험 성립 조건은 기준 v5 4장을 그대로 옮겼다. 비교 지표(안전 실패 · 도착)는 보지 않는다.
  W1: S3 밀림 기록이 방식마다 95 % 이상, 위치 오차 조작(AMCL 추정과 참값의 거리가 0.5 m 이상 벌어졌다가 0.2 m 아래로 돌아옴)이
      S3 · S3j 각각 방식마다 80 % 이상.
  W2 · W3: S8 P1r의 안전 실패 50 % 이상, 물러나기 기록 방식마다 95 % 이상.
  B: P1ir 칸의 접근 속도 중앙값(가장자리 1.0–0.6 m) 0.40 m/s 이상, 접근 각도 중앙값 5° 안. 감속 단계의 실제 감속(제동 시험)이 목표 ± 15 %.
- 장치 결함: find_faults.py의 규칙에 '카메라 방식인데 처리한 깊이 영상 0프레임'을 더했다 (결과와 무관, 고정 전 11/05).
"""
import glob
import json
import math
import os
import socket
import statistics
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from analyze_v2 import fisher, holm, mcnemar, newcombe, safety_fail, verdict  # noqa: E402
from analyze_v3 import approach_speed, arrived, paired_ci, table2, track, voids  # noqa: E402
from analyze_v4 import first_stop, signed_edge  # noqa: E402
from find_faults import faults  # noqa: E402
import v5_plan as V  # noqa: E402

PUSH_SHARE, LOC_SHARE, BACKUP_SHARE, HAZARD_SHARE = 0.95, 0.8, 0.95, 0.5
LOC_FAR, LOC_BACK = 0.5, 0.2      # 위치 오차 조작: 이만큼 벌어졌다가 [m], 이 아래로 돌아옴 [m]
GATE_V, GATE_ANGLE = 0.40, 5.0    # B 접근 속도 [m/s], 접근 각도 [deg]
DECEL_TOL = 0.15
BAND = (0.6, 1.0)                 # 접근 각도를 재는 가장자리 거리 구간 [m] (접근 속도와 같은 구간)
METRICS = {"safety": safety_fail, "arrival": arrived}


def camera_frames(run_dir):
    """cliff_detector가 처리한 깊이 영상 수. 카메라를 쓰지 않는 방식(cliff.jsonl 없음)은 None."""
    p = os.path.join(run_dir, "cliff.jsonl")
    if not os.path.exists(p):
        return None
    n = 0
    for ln in open(p):
        if '"n"' in ln:
            n = max(n, json.loads(ln).get("n") or 0)
    return n


def real_faults(d):
    """find_faults의 결함(가짜 도착 제외)에 '카메라 영상 0프레임'을 더한다 (v5, 11/05: 화면 변수 없이 띄우면 Gazebo 카메라가 안 뜬다).
    둘 다 로봇의 판단과 무관한 장치 결함이다."""
    out = [(x, why) for x, why in faults(d) if not why.startswith("가짜 도착")]
    bad = {os.path.abspath(x) for x, _ in out}
    for dirpath, _, filenames in os.walk(d):
        if "result.json" in filenames and os.path.abspath(dirpath) not in bad and camera_frames(dirpath) == 0:
            out.append((dirpath, "카메라 영상 0프레임 (장치 결함)"))
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


# ---------- 조작 확인 · 접근 ----------
def loc_shift(tr):
    """AMCL 추정(est)과 참값(gt)의 거리가 LOC_FAR 이상 벌어졌다가 LOC_BACK 아래로 돌아왔나. (최대 거리, 돌아왔나)"""
    errs = [(p["t"], math.hypot(p["est"][0] - p["gt"][0], p["est"][1] - p["gt"][1])) for p in tr if p.get("est")]
    i = next((k for k, (_, e) in enumerate(errs) if e >= LOC_FAR), None)
    top = max((e for _, e in errs), default=0.0)
    return top, i is not None and any(e < LOC_BACK for _, e in errs[i:])


def nearest_point(x, y, rects):
    best = None
    for a, b, c, d in rects:
        px, py = min(max(x, a), c), min(max(y, b), d)
        dd = math.hypot(px - x, py - y)
        if best is None or dd < best[0]:
            best = (dd, px, py)
    return best


def approach_angle(tr, rects):
    """가장자리 1.0–0.6 m 구간에서 진행 방향과 '가장 가까운 가장자리 점 쪽' 방향의 각도 중앙값 [deg]. 구간에 안 들어가면 None."""
    out = []
    for p in tr:
        x, y, yaw = p["gt"][0], p["gt"][1], p["gt"][2]
        e = signed_edge(x, y, rects)
        if BAND[0] <= e < BAND[1]:
            _, px, py = nearest_point(x, y, rects)
            d = math.atan2(py - y, px - x) - yaw
            out.append(abs(math.degrees(math.atan2(math.sin(d), math.cos(d)))))
    return statistics.median(out) if out else None


def events(r, kind):
    return [e for e in r["events"] if e["kind"] == kind]


# ---------- 제동 시험 ----------
def brake_table(root):
    """사전 시험 제동 결과: 바퀴 설정값 → (감속 중앙값, 지연 중앙값, 회수). 없으면 빈 dict."""
    rows = {}
    for f in glob.glob(os.path.join(os.path.dirname(os.path.abspath(root)), "v5_pilot", "brake", "acc_*.jsonl")):
        for ln in open(f):
            r = json.loads(ln)
            if r.get("decel"):
                rows.setdefault(round(r["wheel_acc"], 3), []).append(r)
    return {k: (statistics.median(x["decel"] for x in v), statistics.median(x["latency"] for x in v if x.get("latency") is not None)
                if any(x.get("latency") is not None for x in v) else None, len(v)) for k, v in rows.items()}


def measured_decel(brake, key):
    target, acc = V.DECEL[key]
    m = brake.get(round(acc, 3))
    return (m[0] if m else None), target


def predicted_stop(brake, ir_r, key, delay):
    """기준 v5 5장의 식. 제동 시험 값이 없으면 목표 감속(기본 단계는 2.5)으로 계산한다."""
    a_base = measured_decel(brake, "base")[0] or 2.5
    a, target = measured_decel(brake, key)
    a = a or target or a_base
    v = V.V_B
    return V.FIRST_STOP_F[ir_r] - v * delay - v * v / 2 * (1 / a - 1 / a_base)


def gate(get, root, group, scens, cond):
    """시험 성립 조건 (기준 v5 4장). 못 넘은 항목을 글로 돌려준다."""
    c, data, out = V.GROUPS[group], get(group), []
    rs = lambda s, p: [r for (ss, cc, _), r in data.items() if ss == s and cc == p]  # noqa: E731
    if group == "WS3":
        if "S3" in scens:
            r3 = rs("S3", cond)
            n = sum(bool(events(r, "push")) for r in r3)
            if r3 and n < PUSH_SHARE * len(r3):
                out.append(f"밀림 S3 {cond} {n}/{len(r3)}")
        for s in scens:
            rr = rs(s, cond)
            n = sum(loc_shift(track(r))[1] for r in rr)
            if rr and n < LOC_SHARE * len(rr):
                out.append(f"위치 오차 조작 {s} {cond} {n}/{len(rr)}")
    if group == "WS8":
        ctl = rs("S8", "P1r")
        k = sum(map(safety_fail, ctl))
        if not ctl or k < HAZARD_SHARE * len(ctl):
            out.append(f"위험 확인 S8 P1r {k}/{len(ctl)}")
        rr = rs("S8", cond)
        n = sum(bool(events(r, "backup")) for r in rr)
        if rr and n < BACKUP_SHARE * len(rr):
            out.append(f"물러나기 S8 {cond} {n}/{len(rr)}")
    if c.get("decel") is not None:
        rects = voids(V.WORLD)
        if cond == "P1ir":
            rr = rs("S9", cond)
            trs = [track(r) for r in rr]
            vs = [v for v in (approach_speed(t, rects) for t in trs) if v is not None]
            ags = [a for a in (approach_angle(t, rects) for t in trs) if a is not None]
            if not vs or statistics.median(vs) < GATE_V:
                out.append(f"접근 속도 {cond} {statistics.median(vs) if vs else 0:.2f} m/s")
            if not ags or statistics.median(ags) > GATE_ANGLE:
                out.append(f"접근 각도 {cond} {statistics.median(ags) if ags else 99:.1f}°")
        a, target = measured_decel(brake_table(root), c["decel"])
        if target is not None and (a is None or abs(a - target) > DECEL_TOL * target):
            out.append(f"감속 {c['decel']} {'없음' if a is None else f'{a:.2f}'} (목표 {target})")
    return out


def label(group, cond):
    ir_r = V.GROUPS[group].get("ir_r")
    return f"{cond}({ir_r:.2f})" if cond == "P1ir" and ir_r is not None else cond


def cond_label(group):
    c = V.GROUPS[group]
    if c.get("decel") is None:
        return "월드 2"
    t = V.DECEL[c["decel"]][0]
    return f"월드 2, 감속 {'기본' if t is None else t}, 지연 +{c.get('delay', 0.0):.1f} s"


def med(vs, fmt="{:.2f}"):
    vs = [v for v in vs if v is not None]
    return fmt.format(statistics.median(vs)) if vs else "–"


# ---------- 출력 ----------
def print_primary(get, root):
    print("| # | 시나리오 | 조건 | 앞 대 뒤 | 지표 | 앞 | 뒤 | 차이 앞−뒤 (95 % CI) | 검정 | p | 보정 p | 도착 앞 / 뒤 (차이 CI) | 성립 | 판정 |")
    print("|---|---|---|---|---|---:|---:|---|---|---:|---:|---|---|---|")
    ms, gates = [], []
    for name, ga, gb, sc, a, b, met in V.PRIMARY:
        ms.append(compare(get(ga), get(gb), sc, a, b, met))
        gates.append(sorted(set(gate(get, root, ga, sc, a) + gate(get, root, gb, sc, b))))
    adj = holm([m["p"] if "p" in m and not g else 1.0 for m, g in zip(ms, gates)])
    for (name, ga, gb, sc, a, b, met), m, g, pa in zip(V.PRIMARY, ms, gates, adj):
        head = f"| {name} | {'·'.join(sc)} | {cond_label(ga)} | {label(ga, a)} 대 {label(gb, b)} | {'안전 실패' if met == 'safety' else '도착'}"
        if "p" not in m:
            print(f"{head} | – | – | – | – | – | – | – | – | 데이터 없음 |")
            continue
        if name in V.NO_ARRIVAL_RULE:
            m = dict(m, arr_ci=(0.0, 0.0))          # 도착 조건을 보지 않는다 (목표가 구멍 안)
        ci, (aa, an, ab, bn) = m["ci"], m["arr"]
        arr_txt = "– (보지 않음)" if name in V.NO_ARRIVAL_RULE else \
            f"{aa}/{an} · {ab}/{bn} ({100 * m['arr_ci'][0]:+.0f} ~ {100 * m['arr_ci'][1]:+.0f})"
        print(f"{head} | {m['ka']}/{m['na']} | {m['kb']}/{m['nb']} | "
              f"{100 * (m['ka'] / m['na'] - m['kb'] / m['nb']):+.0f} %p ({100 * ci[0]:+.0f} ~ {100 * ci[1]:+.0f}) | "
              f"{m['test']} | {m['p']:.3g} | {pa:.3g} | {arr_txt} | "
              f"{'탈락: ' + ', '.join(g) if g else '통과'} | {'시험 안 됨' if g else verdict(m, met, pa)} |")
    print()


def print_manipulation(get):
    print("| 묶음 | 시나리오 | 방식 | 회차 | 밀림 기록 | 위치 오차 조작 (0.5 → 0.2 m) | 최대 위치 오차 중앙값 | 물러나기 기록 | 안전 실패 | 도착 |")
    print("|---|---|---|---:|---:|---:|---:|---:|---:|---:|")
    for g in ("WS3", "WS3r", "WS8"):
        c = V.GROUPS[g]
        for s in c["scens"]:
            for p in c["conds"]:
                rs = [r for (ss, cc, _), r in get(g).items() if ss == s and cc == p]
                if not rs:
                    continue
                shifts = [loc_shift(track(r)) for r in rs]
                print(f"| {g} | {s} | {p} | {len(rs)} | {sum(bool(events(r, 'push')) for r in rs)} | {sum(ok for _, ok in shifts)} | "
                      f"{med(top for top, _ in shifts)} m | {sum(bool(events(r, 'backup')) for r in rs)} | "
                      f"{sum(map(safety_fail, rs))} | {sum(map(arrived, rs))} |")
    print()


def print_grid(get, root):
    brake = brake_table(root)
    rects = voids(V.WORLD)
    print("| 묶음 | 감속 단계 (실제 감속) | 지연 | 방식 | 회차 | 안전 실패 | 추락 | 첫 정지 여유 중앙값 (최소) | 예측 첫 정지 | 제동 거리 중앙값 | 접근 속도 | 접근 각도 | 성립 |")
    print("|---|---|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---|")
    for g, c in V.GROUPS.items():
        if c.get("decel") is None:
            continue
        a, target = measured_decel(brake, c["decel"])
        for p in c["conds"]:
            rs = [r for (_, cc, _), r in get(g).items() if cc == p]
            if not rs:
                continue
            trs = [track(r) for r in rs]
            stops = [first_stop(r, t, rects) for r, t in zip(rs, trs)]
            margins = [mm for mm, _ in stops if mm is not None]
            pred = f"{predicted_stop(brake, c['ir_r'], c['decel'], c.get('delay', 0.0)):.3f} m" if p == "P1ir" else "–"
            bad = gate(get, root, g, c["scens"], p)
            print(f"| {g} | {c['decel']} ({'–' if a is None else f'{a:.2f}'}{'' if target is None else f', 목표 {target}'}) | "
                  f"+{c.get('delay', 0.0):.1f} s | {label(g, p)} | {len(rs)} | {sum(map(safety_fail, rs))} | "
                  f"{sum(r['outcome'] == 'fall' for r in rs)} | "
                  f"{f'{statistics.median(margins):.3f} ({min(margins):.3f}) m' if margins else '–'} | {pred} | "
                  f"{med((b for _, b in stops), '{:.3f} m')} | {med(approach_speed(t, rects) for t in trs)} m/s | "
                  f"{med((approach_angle(t, rects) for t in trs), '{:.1f}°')} | {'탈락: ' + ', '.join(bad) if bad else '통과'} |")
    print()


def print_brake(root):
    brake = brake_table(root)
    if not brake:
        print("제동 시험 결과 없음 (사전 시험 `v5_pilot/brake/`).\n")
        return
    print("| 바퀴 설정값 | 실제 감속 중앙값 | 명령 → 감속 지연 중앙값 | 회수 |")
    print("|---:|---:|---:|---:|")
    for k in sorted(brake, reverse=True):
        a, lat, n = brake[k]
        print(f"| {k} | {a:.2f} m/s² | {'–' if lat is None else f'{lat:.3f} s'} | {n} |")
    print()


def print_faults(root):
    base = os.path.join(os.path.dirname(os.path.abspath(root)), "v5_faults")
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
    root = args[0] if args else os.path.join(HERE, "..", f"results_{socket.gethostname()}", "v5")
    cache = {}
    get = lambda g: cache.setdefault(g, load(root, g))  # noqa: E731
    print(f"# 판정 기준 v5 — 확인 실험 판정\n\n결과 폴더: `{os.path.relpath(root)}`\n")
    print("## 주 가설 (Holm 보정 5개)\n")
    print_primary(get, root)
    print("## 조작 확인 · 참고 (W 묶음)\n")
    print_manipulation(get)
    print("## 부 가설 G: 0.45 m/s 정면 접근 격자 (판정 없음)\n")
    print_grid(get, root)
    print("## 제동 시험 (사전 시험)\n")
    print_brake(root)
    print("## 장치 결함\n")
    print_faults(root)
    left = sum(len(real_faults(os.path.join(root, g))) for g in V.GROUPS if os.path.isdir(os.path.join(root, g)))
    print(f"남아 있는 장치 결함 회차 (분석에서 뺌): {left}")


if __name__ == "__main__":
    main()
