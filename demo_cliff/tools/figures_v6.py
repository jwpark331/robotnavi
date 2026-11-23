"""판정 기준 v6 발표용 그림 (11-21, 판정 뒤 — 판정에 쓰지 않는다, 고정 파일 아님) → results_<호스트>/figures_v6/

    PYTHONNOUSERSITE=1 python3 tools/figures_v6.py       # matplotlib은 시스템 numpy 1.x로 돈다

1. v6_backup_paths.png — 같은 배치(월드 1 · 0.55 m · 시드 SEED)에서 다섯 로봇이 뒤로 비켜 준 궤적(참값).
   ① 로봇 칸에는 후진 요청 때 기억하던 낭떠러지 칸(cliff.jsonl, odom → 월드 변환)을 겹친다.
2. v6_results.png — 후진 묶음 4칸 × 다섯 로봇: 안전 실패 수 · 비켜 준 수 (20회 중). analysis.md와 같은 정의.
영상(① · ③ 나란히)은 tools/video_v6.py (OpenCV가 사용자 numpy 2.x에 묶여 있어 따로 돈다).
"""
import json
import math
import os
import socket
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Rectangle  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from figures import setup, SURF, INK, INK2, MUTED  # noqa: E402
from analyze_v2 import safety_fail  # noqa: E402
from analyze_v3 import voids  # noqa: E402
from analyze_v6 import load, meas, yielded, backup_margin  # noqa: E402

ROOT = os.path.join(HERE, "..", f"results_{socket.gethostname()}")
RUNS = os.path.join(ROOT, "v6")
OUT = os.path.join(ROOT, "figures_v6")
GROUP, SEED = "Y-w1-d055", 513
GROUPS = [("Y-w1-d055", "월드 1\n가장자리 0.55 m"), ("Y-w2-d055", "월드 2\n가장자리 0.55 m"),
          ("Y-w1-d110", "월드 1\n가장자리 1.10 m"), ("Y-w2-d110", "월드 2\n가장자리 1.10 m")]
ROBOTS = [("P3y", "① 기억을 보고\n안전한 만큼만 물러남", "#2a78d6"),
          ("P3", "② 후진을\nNav2에 맡김", "#eda100"),
          ("P3n", "③ 기억 없음\n(요청대로 물러남)", "#eb6834"),
          ("P1irb", "④ 앞·뒤 IR", "#1baf7a"),
          ("P2nav", "⑤ Nav2 깊이 레이어", "#e87ba4")]
VOID, MEM = "#3d3c39", "#5aa0ec"


def ev(r, kind):
    return next((e for e in r["events"] if e["kind"] == kind), None)


def memory_cells(r):
    """후진 요청 시각에 기억하던 금지(lethal) 칸들의 월드 좌표 사각형. odom 원점 = 시작 참값 자세."""
    p = os.path.join(r["dir"], "cliff.jsonl")
    plan = r.get("yield_plan") or {}
    if not os.path.exists(p) or "t" not in plan:
        return [], 0.1
    meta, last = None, None
    for line in open(p):
        rec = json.loads(line)
        if rec.get("meta"):
            meta = rec
        elif rec["t"] <= plan["t"]:
            last = rec
    if meta is None or last is None:
        return [], 0.1
    tr = json.load(open(os.path.join(r["dir"], "track.json")))
    g0, o0 = tr[0]["gt"], tr[0]["odom"]
    th = g0[2] - o0[2]
    c, s = math.cos(th), math.sin(th)
    tx, ty = g0[0] - (c * o0[0] - s * o0[1]), g0[1] - (s * o0[0] + c * o0[1])
    res = meta["res"]
    out = []
    for code in last["lethal"]:
        ix, iy = divmod(code, 1000)
        ox, oy = meta["ox"] + (ix + 0.5) * res, meta["oy"] + (iy + 0.5) * res
        out.append((tx + c * ox - s * oy, ty + s * ox + c * oy))
    return out, res


def fig_paths(data):
    rects = voids(1)
    fig, axes = plt.subplots(1, 5, figsize=(16, 4.9), sharex=True, sharey=True)
    for ax, (cond, label, col) in zip(axes, ROBOTS):
        r = data[(GROUP, "S8", cond, SEED)]
        bk, be = ev(r, "backup"), ev(r, "backup_end")
        t0, t1 = bk["t"] - 1.0, (be["t"] if be else r["time"]) + 0.5
        tr = json.load(open(os.path.join(r["dir"], "track.json")))
        seg = [p["gt"] for p in tr if t0 <= p["t"] <= t1]
        for x0, y0, x1, y1 in rects:
            ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, color=VOID, zorder=1))
        if cond == "P3y":
            cells, res = memory_cells(r)
            for cx, cy in cells:
                ax.add_patch(Rectangle((cx - res / 2, cy - res / 2), res, res, fc=MEM, alpha=0.75, ec=SURF, lw=0.4, zorder=2))
        ax.axhline(5.4 + 0.30, color=MUTED, lw=0.8, ls=(0, (4, 3)), zorder=2)
        xs, ys = [p[0] for p in seg], [p[1] for p in seg]
        ax.plot(xs, ys, color=col, lw=2.6, zorder=4)
        ax.plot(xs[0], ys[0], "o", ms=9, color=col, mec=INK, mew=0.8, zorder=5)
        ax.plot(xs[-1], ys[-1], "s", ms=8, color=col, mec=INK, mew=0.8, zorder=5)
        moved = meas(r)["moved"]
        m = backup_margin(r)
        tag = "가장자리 넘음" if m < 0 else ("위험 (< 0.15 m)" if m < 0.15 else "")
        ax.set_title(label, fontsize=11.5, loc="left")
        ax.text(0.03, 0.97, f"물러난 거리 {moved:.2f} m\n가장자리까지 최소 {m:.2f} m" + (f"\n{tag}" if tag else ""),
                transform=ax.transAxes, va="top", fontsize=10, color=INK,
                bbox=dict(boxstyle="round,pad=0.3", fc=SURF, ec="none", alpha=0.9), zorder=6)
        ax.set_aspect("equal")
        ax.set_xlim(7.7, 9.5)
        ax.set_ylim(4.95, 6.75)
        ax.grid(False)
    axes[1].text(8.27, 5.1, "계단 (지도에 없음)", color="white", fontsize=10, zorder=3)
    axes[0].set_ylabel("y [m]")
    for ax in axes:
        ax.set_xlabel("x [m]")
    fig.suptitle("같은 자리에서 '뒤로 비켜 달라'는 요청을 받았을 때 (월드 1 · 가장자리에서 0.55 m · 시드 %d)" % SEED,
                 x=0.01, ha="left", fontsize=14, fontweight="bold")
    fig.text(0.01, 0.015, "선 = 후진 구간 참값 궤적 (● 시작, ■ 끝) · 점선 = 가장자리에서 0.30 m · "
             "① 칸의 파란 칸 = 요청 때 기억하던 낭떠러지 칸 (① 로봇은 이 칸에서 약 0.30 m 앞까지만 간다) · 로봇 중심 기준", fontsize=9.5, color=MUTED)
    fig.tight_layout(rect=(0, 0.04, 1, 0.94))
    p = os.path.join(OUT, "v6_backup_paths.png")
    fig.savefig(p, dpi=200)
    plt.close(fig)
    return p


def fig_results(data):
    fig, axes = plt.subplots(1, 2, figsize=(15, 6.0), sharey=True)
    w = 0.8 / len(ROBOTS)
    for ax, (title, fn) in zip(axes, [("안전 실패 (추락 또는 가장자리 0.15 m 안)", safety_fail),
                                      ("비켜 줌 (실제로 0.20 m 이상 물러남)", yielded)]):
        for ri, (cond, label, col) in enumerate(ROBOTS):
            for gi, (g, _) in enumerate(GROUPS):
                rs = [r for (gg, s, c, k), r in data.items() if gg == g and c == cond]
                k = sum(bool(fn(r)) for r in rs)
                x = gi + (ri - (len(ROBOTS) - 1) / 2) * w
                ax.bar(x, max(k, 0.15), w * 0.88, color=col, zorder=3, label=label.replace("\n", " ") if gi == 0 else None)
                ax.text(x, k + 0.3, str(k), ha="center", va="bottom", fontsize=8.5, color=INK)
        ax.set_xticks(range(len(GROUPS)), [l for _, l in GROUPS])
        ax.set_ylim(0, 22.5)
        ax.set_title(title, loc="left")
        ax.grid(axis="x", visible=False)
    axes[0].set_ylabel("회차 (칸마다 20회)")
    h, l = axes[0].get_legend_handles_labels()
    fig.legend(h, l, ncol=5, loc="lower center", bbox_to_anchor=(0.5, 0.045), fontsize=10.5)
    fig.suptitle("뒤로 비켜 주기 — 다섯 로봇 × 네 배치 (확인 실험 400회)", x=0.01, ha="left", fontsize=14, fontweight="bold")
    fig.text(0.01, 0.01, "같은 시드 20개를 다섯 로봇이 나눠 탄다 · 판정: 가설 1 (①대③ 안전 실패) · 가설 2 (①대② 비켜 줌), "
             "0.55 m 두 칸 · 1.10 m 두 칸 합산 (analysis.md)", fontsize=9.5, color=MUTED)
    fig.tight_layout(rect=(0, 0.11, 1, 0.94))
    p = os.path.join(OUT, "v6_results.png")
    fig.savefig(p, dpi=200)
    plt.close(fig)
    return p


def main():
    setup()
    os.makedirs(OUT, exist_ok=True)
    data = {}
    for g, _ in GROUPS:
        data.update(load(RUNS, g))
    print(fig_paths(data))
    print(fig_results(data))


if __name__ == "__main__":
    main()
