"""판정 기준 v5 사전 시험 채점 — 10장 표의 통과 기준을 그대로 코드로 옮겼다. 비교 방식은 돌리지 않았다 (P1r 회차, 제동 시험, 지연 주입만).

    python3 tools/check_v5_pilot.py [사전 시험 폴더]      # 기본 results_<호스트>/v5_pilot

마지막에 제동 시험 값으로 다시 계산한 5장 예측표와, 목표 감속에 맞는 바퀴 설정값을 찍는다 (고정 전에 기준 문서와 v5_plan.py에 옮긴다).
"""
import glob
import json
import os
import socket
import statistics
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from analyze_v2 import safety_fail  # noqa: E402
from analyze_v3 import approach_speed, track, voids  # noqa: E402
from analyze_v5 import (DECEL_TOL, GATE_ANGLE, GATE_V, approach_angle, events, loc_shift, real_faults,  # noqa: E402
                        brake_table)
import v5_plan as V  # noqa: E402


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


def common(root, name, rs, n):
    bad = real_faults(os.path.join(root, name)) if os.path.isdir(os.path.join(root, name)) else []
    return len(rs) == n and not bad, f"회차 {len(rs)}/{n}, 결함 {len(bad)}, TF 경고 최대 {max(map(tf_lines, rs), default=0)}줄"


def main():
    root = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, "..", f"results_{socket.gethostname()}", "v5_pilot")
    rects = voids(V.WORLD)
    rows = []

    p = os.path.join(root, "delay.txt")
    txt = open(p).read().strip() if os.path.exists(p) else ""
    rows.append(("지연 주입 (오프라인)", bool(txt) and "미달" not in txt and txt.count("통과") == 2, txt.replace("\n", " / ") or "결과 없음"))

    brake = brake_table(os.path.join(root, "..", "v5"))
    for key, (target, acc) in V.DECEL.items():
        m = brake.get(round(acc, 3))
        if target is None:
            rows.append((f"제동 {key} (설정 {acc})", m is not None, f"실제 감속 {m[0]:.2f} m/s² ({m[2]}회)" if m else "결과 없음"))
        else:
            ok = m is not None and abs(m[0] - target) <= DECEL_TOL * target
            rows.append((f"제동 {key} (설정 {acc}, 목표 {target})", ok, f"실제 감속 {m[0]:.2f} m/s² ({m[2]}회)" if m else "결과 없음"))

    for s, n in (("S3", 2), ("S3j", 2)):
        rs = [r for r in load(root, "1_s3") if r["scenario"] == s]
        ok, txt = common(root, "1_s3", rs, n)
        shifts = [loc_shift(track(r)) for r in rs]
        push = sum(bool(events(r, "push")) for r in rs)
        good = ok and all(c for _, c in shifts) and (s != "S3" or push == len(rs))
        rows.append((f"{s} P1r", good, f"{txt}, 밀림 기록 {push}, 위치 오차 조작 {sum(c for _, c in shifts)}/{len(rs)} "
                                         f"(최대 {', '.join(f'{t:.2f}' for t, _ in shifts)} m), 안전 실패 {sum(map(safety_fail, rs))}"))

    rs = load(root, "2_s8")
    ok, txt = common(root, "2_s8", rs, 3)
    back = sum(bool(events(r, "backup")) for r in rs)
    fail = sum(map(safety_fail, rs))
    rows.append(("S8 P1r", ok and back == len(rs) and fail >= 2, f"{txt}, 물러나기 기록 {back}, 안전 실패 {fail}"))

    rs = load(root, "3_s9")
    ok, txt = common(root, "3_s9", rs, 2)
    trs = [track(r) for r in rs]
    vs = [approach_speed(t, rects) for t in trs]
    ags = [approach_angle(t, rects) for t in trs]
    good = ok and all(v is not None and v >= GATE_V for v in vs) and all(a is not None and a <= GATE_ANGLE for a in ags)
    rows.append(("S9w2 P1r", good, f"{txt}, 접근 속도 {', '.join('–' if v is None else f'{v:.2f}' for v in vs)} m/s, "
                                   f"접근 각도 {', '.join('–' if a is None else f'{a:.1f}' for a in ags)}°, "
                                   f"결과 {', '.join(r['outcome'] for r in rs)}"))

    print("# 판정 기준 v5 — 사전 시험 채점\n")
    print("| 항목 | 통과 | 내용 |")
    print("|---|---|---|")
    for name, ok, txt in rows:
        print(f"| {name} | {'통과' if ok else '**미달**'} | {txt} |")
    print(f"\n전체: {'모두 통과' if all(ok for _, ok, _ in rows) else '미달 있음'}\n")

    if brake:
        print("## 제동 시험 원자료 (바퀴 설정값별)\n")
        print("| 바퀴 설정값 | 실제 감속 중앙값 | 명령 → 감속 지연 중앙값 | 회수 |")
        print("|---:|---:|---:|---:|")
        for k in sorted(brake, reverse=True):
            a, lat, n = brake[k]
            print(f"| {k} | {a:.2f} m/s² | {'–' if lat is None else f'{lat:.3f} s'} | {n} |")
        print("\n## 목표 감속에 가장 가까운 설정값\n")
        for key, (target, _) in V.DECEL.items():
            if target is None:
                continue
            k = min(brake, key=lambda x: abs(brake[x][0] - target))
            print(f"- {key} (목표 {target} m/s²): 설정 {k} → 실제 {brake[k][0]:.2f} m/s² "
                  f"({'± 15 % 안' if abs(brake[k][0] - target) <= DECEL_TOL * target else '± 15 % 밖'})")
        a_base = (brake.get(round(V.DECEL['base'][1], 3)) or (None,))[0]
        if a_base:
            print("\n## 5장 예측표 (제동 시험 값으로 다시 계산, 첫 정지 위치 [m], 0.15 m 아래면 안전 실패 예측)\n")
            print("| 감속 단계 | 실제 감속 | IR 0.24, 기본 지연 | IR 0.24, +0.1 s | IR 0.30, 기본 지연 | IR 0.30, +0.1 s |")
            print("|---|---:|---:|---:|---:|---:|")
            v = V.V_B
            for key, (target, acc) in V.DECEL.items():
                m = brake.get(round(acc, 3))
                if not m:
                    continue
                a = m[0]
                cells = [V.FIRST_STOP_F[ir] - v * dl - v * v / 2 * (1 / a - 1 / a_base) for ir in (V.IR_R0, V.IR_R1) for dl in V.DELAYS]
                print(f"| {key} | {a:.2f} | " + " | ".join(f"{c:.3f}" for c in cells) + " |")
            print()


if __name__ == "__main__":
    main()
