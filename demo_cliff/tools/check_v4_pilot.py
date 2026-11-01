"""판정 기준 v4 사전 시험 채점 — 8장 표의 통과 기준을 그대로 코드로 옮겼다.

    python3 tools/check_v4_pilot.py [사전 시험 폴더]      # 기본 results_<호스트>/v4_pilot
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
from analyze_v3 import approach_speed, arrived, track, voids  # noqa: E402
from analyze_v4 import ir_starts, real_faults, signed_edge, speeds  # noqa: E402

GATE_V = 0.56          # 0.8 m/s의 70 %
EDGE_V = 0.72          # 0.8 m/s의 90 %


def load(root, name):
    out = []
    for d in sorted(glob.glob(os.path.join(root, name, "*"))):
        rj = os.path.join(d, "result.json")
        if os.path.isdir(d) and os.path.exists(rj):
            r = json.load(open(rj))
            r["dir"] = d
            out.append(r)
    return out


def tf_lines(r):
    return sum("Transform data too old" in ln for ln in open(os.path.join(r["dir"], "launch.log"), errors="ignore"))


def common(root, name, rs, n, need_arrive=True):
    """모든 항목의 공통 기준. (통과 여부, 설명)"""
    bad = real_faults(os.path.join(root, name)) if os.path.isdir(os.path.join(root, name)) else []
    fall = sum(r["outcome"] == "fall" for r in rs)
    arr = sum(map(arrived, rs))
    ok = len(rs) == n and not bad
    txt = f"회차 {len(rs)}/{n}, 결함 {len(bad)}, TF 경고 최대 {max(map(tf_lines, rs), default=0)}줄"
    if need_arrive:
        ok = ok and arr == n and fall == 0
        txt += f", 도착 {arr}/{n}, 넘어짐 {fall}"
    return ok, txt


def corridor_speed(tr):
    """복도 x 5–7 m 구간의 평균 속도."""
    vs = [v for p, v in zip(tr, speeds(tr)) if v is not None and 5.0 <= p["gt"][0] <= 7.0]
    return sum(vs) / len(vs) if vs else 0.0


def max_cap(r):
    caps = [json.loads(ln).get("cap") for ln in open(os.path.join(r["dir"], "cliff.jsonl"))]
    caps = [c for c in caps if c is not None]
    return max(caps) if caps else 0.0


def speed_after_kill(r, tr):
    """종료 1 s 뒤부터 1 s 구간 평균 속도의 최대. track은 약 10 Hz이고 기록 간격이 흔들려 구간 하나로는 속도가 부풀려진다
    (0.8 m/s 회차의 구간 속도가 1.18 m/s로 찍힌다)."""
    kill = next(e for e in r["events"] if e["kind"] == "camera_kill")
    pts = [p for p in tr if p["t"] >= kill["t"] + 1.0]
    out = 0.0
    for i, p in enumerate(pts):
        q = next((x for x in pts[i:] if x["t"] >= p["t"] + 1.0), None)
        if q:
            out = max(out, math.hypot(q["gt"][0] - p["gt"][0], q["gt"][1] - p["gt"][1]) / (q["t"] - p["t"]))
    return out


def cliff_lines_after_kill(r):
    kill = next(e for e in r["events"] if e["kind"] == "camera_kill")
    t = (r.get("t0_sim") or 0) + kill["t"] + 1.0
    return sum(json.loads(ln).get("t", 0) > t for ln in open(os.path.join(r["dir"], "cliff.jsonl")))


def approach_angle(tr, rects):
    """가장자리 0.5 m 안에 처음 들 때 진행 방향과 x축(계단 A 서쪽 가장자리의 수직선)의 각도 [deg]. 못 들면 None."""
    for p in tr:
        if signed_edge(p["gt"][0], p["gt"][1], rects) < 0.5:
            return abs(math.degrees(math.atan2(math.sin(p["gt"][2]), math.cos(p["gt"][2]))))
    return None


def edge_speed(tr, rects, dist=0.25):
    """가장자리 dist 안에 처음 들 때의 속도: 직전 0.3 s 구간 평균 (기록 간격의 흔들림을 줄인다). 못 들면 None."""
    for i, p in enumerate(tr):
        if signed_edge(p["gt"][0], p["gt"][1], rects) < dist:
            q = next((x for x in reversed(tr[:i]) if x["t"] <= p["t"] - 0.3), None)
            return math.hypot(p["gt"][0] - q["gt"][0], p["gt"][1] - q["gt"][1]) / (p["t"] - q["t"]) if q else None
    return None


def recomputed_person(tr):
    best = 9e9
    for q in tr:
        for px, py in q.get("ped", []):
            best = min(best, math.hypot(q["gt"][0] - px, q["gt"][1] - py))
    return best


def main():
    args = [x for x in sys.argv[1:] if not x.startswith("--")]
    root = args[0] if args else os.path.join(HERE, "..", f"results_{socket.gethostname()}", "v4_pilot")
    rects = voids(1)
    rows = []

    # 1. P3p가 0.8 m/s에서 동작하는가
    rs = load(root, "1_p3p")
    ok, txt = common(root, "1_p3p", rs, 3)
    v = statistics.median(corridor_speed(track(r)) for r in rs) if rs else 0.0
    cap = max(map(max_cap, rs), default=0.0)
    rows.append(("1 P3p 0.8 m/s", ok and v >= GATE_V and cap > 0.45, f"{txt}, x 5–7 m 속도 {v:.2f} m/s, 상한 최대 {cap:.2f} m/s"))

    # 2. P3p + 결측 100 %
    rs = load(root, "2_p3p_void")
    ok, txt = common(root, "2_p3p_void", rs, 3)
    rows.append(("2 P3p VOID=1.0", ok and all(r["void_dropout"] == 1.0 for r in rs), f"{txt}, void_dropout {sorted({r['void_dropout'] for r in rs})}"))

    # 3. 카메라 종료 주입
    rs = load(root, "3_camkill")
    ok, txt = common(root, "3_camkill", rs, 6)
    kills = [next((e for e in r["events"] if e["kind"] == "camera_kill"), None) for r in rs]
    dead = sum(bool(k and k["dead"]) for k in kills)
    p3p = [r for r, k in zip(rs, kills) if r["condition"] == "P3p" and k]
    vmax = max((speed_after_kill(r, track(r)) for r in p3p), default=9.0)
    lines = max((cliff_lines_after_kill(r) for r, k in zip(rs, kills) if k), default=0)
    xs = [k["x"] for k in kills if k]
    rows.append(("3 카메라 종료", ok and dead == 6 and vmax <= 0.22 and lines == 0,
                 f"{txt}, 종료 확인 {dead}/6, 종료 x {min(xs) if xs else 0:.2f}–{max(xs) if xs else 0:.2f}, P3p 종료 뒤 속도 최대 {vmax:.3f} m/s, 종료 뒤 cliff 기록 {lines}줄"))

    # 4. IR 0.30 m
    rs = load(root, "4_ir30")
    ok, txt = common(root, "4_ir30", rs, 3)
    off_mat = sum(not s["on_mat"] for r in rs for s in ir_starts(r))
    rows.append(("4 IR 0.30 m", ok and off_mat == 0 and all(r["ir_r"] == 0.30 for r in rs), f"{txt}, 매트 밖 IR 정지 {off_mat}, ir_r {sorted({r['ir_r'] for r in rs})}"))

    # 5. 직진 접근
    rs = load(root, "5_straight")
    ok, txt = common(root, "5_straight", rs, 3, need_arrive=False)
    vs = [approach_speed(track(r), rects) for r in rs]
    vs = [x for x in vs if x is not None]
    v = statistics.median(vs) if vs else 0.0
    ang = [approach_angle(track(r), rects) for r in rs]
    worst = max((a for a in ang if a is not None), default=999.0)
    ev = [edge_speed(track(r), rects) for r in rs]
    slow = min((x for x in ev if x is not None), default=0.0)
    rows.append(("5 직진 접근", ok and v >= GATE_V and worst <= 10.0 and None not in ang and None not in ev and slow >= EDGE_V,
                 f"{txt}, 접근 속도 {v:.2f} m/s, 수직선에서 최대 {worst:.1f}°, 가장자리 0.25 m 속도 최소 {slow:.2f} m/s, 결과 {[r['outcome'] for r in rs]}"))

    # 6. 직진 + IR 기록
    rs = load(root, "6_straight_ir")
    ok, txt = common(root, "6_straight_ir", rs, 3, need_arrive=False)
    logged = sum(any(not s["on_mat"] and s["gt"] and signed_edge(s["gt"][0], s["gt"][1], rects) < 1.0 for s in ir_starts(r)) for r in rs)
    ends = [r["time"] for r in rs]
    rows.append(("6 직진 + IR 기록", ok and logged == 3 and max(ends, default=99) <= 32, f"{txt}, IR 정지 시작 기록 {logged}/3, 끝난 시각 최대 {max(ends, default=0):.1f} s"))

    # 7. P2i 0.8 m/s
    rs = load(root, "7_p2i")
    ok, txt = common(root, "7_p2i", rs, 3)
    rows.append(("7 P2i 0.8 m/s", ok, txt))

    # 8. A 속도 조건 미리 보기
    rs = load(root, "8_a_speed")
    ok, txt = common(root, "8_a_speed", rs, 3, need_arrive=False)
    vs = [approach_speed(track(r), rects) for r in rs]
    vs = [x for x in vs if x is not None]
    v = statistics.median(vs) if vs else 0.0
    rows.append(("8 A 속도 미리 보기", ok and v >= GATE_V, f"{txt}, 접근 속도 {v:.2f} m/s (회차 {[round(x, 2) for x in vs]})"))

    # 9. 접촉 기록
    rs = load(root, "9_person")
    ok, txt = common(root, "9_person", rs, 3, need_arrive=False)
    diffs = [abs(r["min_person"] - recomputed_person(track(r))) for r in rs if r.get("min_person") is not None]
    rows.append(("9 접촉 기록", ok and len(diffs) == 3 and max(diffs, default=9.0) <= 0.05,
                 f"{txt}, min_person {[r.get('min_person') for r in rs]}, 다시 계산과 차이 최대 {max(diffs, default=9.0):.3f} m, 접촉 {[r.get('contact_n') for r in rs]}"))

    print("# 판정 기준 v4 사전 시험 채점\n")
    print("| 항목 | 통과 | 값 |\n|---|---|---|")
    for name, ok, txt in rows:
        print(f"| {name} | {'통과' if ok else '**탈락**'} | {txt} |")
    print(f"\n통과 {sum(ok for _, ok, _ in rows)}/{len(rows)}")


if __name__ == "__main__":
    main()
