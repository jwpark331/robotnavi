"""발표용 그림 (PNG) — results/figures/.

    PYTHONNOUSERSITE=1 python3 tools/figures.py

장치 결함 회차는 analyze.load()가 이미 뺀다 (find_faults 기준).
색: 조건 = 범주형 슬롯 1–6 고정 순서(검증: 밝은 모드 CVD ΔE 9.1, 정상 시각 19.6 — 대비 3:1 미만 3색은 값 라벨로 보완).
결과 유형 = 상태색(좋음·경고·위험) + 회색. 경고↔심각은 정상 시각 ΔE 13.6로 구별이 약해 '다른 곳 멈춤'은 회색.
"""
import os
import statistics as st
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib import font_manager  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import analyze as A  # noqa: E402
from analyze import G  # noqa: E402

OUT = os.path.join(A.ROOT, "figures")
SURF, INK, INK2, MUTED, GRID, AXIS = "#fcfcfb", "#0b0b0b", "#52514e", "#898781", "#e1e0d9", "#c3c2b7"
COND_COLOR = {"P0": "#2a78d6", "P1": "#eb6834", "P1i": "#1baf7a", "P2": "#eda100", "P3m": "#e87ba4", "P3": "#008300"}
OUTCOME = [("arrived", "도착", "#0ca30c"), ("edge", "가장자리 앞 갇힘", "#fab219"),
           ("else", "다른 곳 멈춤", "#b5b3ab"), ("fall", "추락", "#d03b3b")]
SHORT = {"P0": "P0 지도만", "P1": "P1 keepout", "P1i": "P1i 하향 IR", "P2": "P2 단일 프레임",
         "P3m": "P3m 베이즈(map)", "P3": "P3 베이즈(odom)"}


def setup():
    for name in ("Noto Sans CJK KR", "Noto Sans CJK JP"):
        if any(f.name == name for f in font_manager.fontManager.ttflist):
            plt.rcParams["font.family"] = name
            break
    plt.rcParams.update({
        "figure.facecolor": SURF, "axes.facecolor": SURF, "savefig.facecolor": SURF,
        "axes.edgecolor": AXIS, "axes.labelcolor": INK2, "xtick.color": INK2, "ytick.color": INK2,
        "text.color": INK, "axes.spines.top": False, "axes.spines.right": False,
        "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.8, "axes.axisbelow": True,
        "font.size": 11, "axes.titlesize": 13, "axes.titleweight": "bold", "legend.frameon": False,
    })


def kind(r):
    if r["outcome"] == "fall":
        return "fall"
    if r["outcome"] == "arrived":
        return "arrived"
    return "edge" if r["min_edge"] < 0.6 else "else"


def fig_fall_rate(runs, conds, scens):
    fig, ax = plt.subplots(figsize=(12, 5.2))
    w = 0.8 / len(conds)
    for ci, c in enumerate(conds):
        for si, s in enumerate(scens):
            rs = [r for r in runs if r["scenario"] == s and r["condition"] == c]
            if not rs:
                continue
            n, k = len(rs), sum(r["outcome"] == "fall" for r in rs)
            p = 100 * k / n
            lo, hi = A.wilson(k, n)
            x = si + (ci - (len(conds) - 1) / 2) * w
            ax.bar(x, max(p, 0.8), w * 0.88, color=COND_COLOR[c], label=SHORT[c] if si == 0 else None, zorder=3)
            ax.plot([x, x], [100 * lo, 100 * hi], color=INK2, lw=1.2, zorder=4)
            ax.plot([x - w * 0.18, x + w * 0.18], [100 * hi] * 2, color=INK2, lw=1.2, zorder=4)
            if k:
                ax.text(x, 100 * hi + 2, f"{k}/{n}", ha="center", va="bottom", fontsize=9, color=INK)
    ax.set_xticks(range(len(scens)), [f"{s}\n{G.SCENARIOS[s]['label']}" for s in scens])
    ax.set_ylim(0, 110)
    ax.set_ylabel("추락률 [%]  (막대 끝 선 = 95% 신뢰구간)")
    ax.set_title("시나리오 × 조건별 추락률", loc="left")
    ax.grid(axis="x", visible=False)
    ax.legend(ncol=6, loc="upper center", bbox_to_anchor=(0.5, -0.14), fontsize=10)
    n_seeds = len({r["seed"] for r in runs})
    fig.text(0.01, 0.01, f"칸마다 시드 최대 {n_seeds}회 (장치 결함 회차 제외) · 값 라벨은 추락이 있는 칸만", fontsize=9, color=MUTED)
    fig.tight_layout(rect=(0, 0.03, 1, 1))
    fig.savefig(os.path.join(OUT, "fig1_fall_rate.png"), dpi=170)
    plt.close(fig)


def fig_outcomes(runs, conds, scens):
    fig, axes = plt.subplots(1, len(scens), figsize=(14, 4.6), sharey=True)
    for ax, s in zip(axes, scens):
        for ri, c in enumerate(conds):
            rs = [r for r in runs if r["scenario"] == s and r["condition"] == c]
            n = len(rs)
            left = 0.0
            for key, _, col in OUTCOME:
                k = sum(kind(r) == key for r in rs)
                if not n or not k:
                    continue
                wdt = 100 * k / n
                ax.barh(ri, wdt - 0.6, 0.62, left=left + 0.3, color=col, zorder=3)
                if wdt >= 12:
                    ax.text(left + wdt / 2, ri, str(k), ha="center", va="center", fontsize=9,
                            color=INK if key in ("edge", "else") else "#ffffff")
                left += wdt
            if n:
                ax.text(101, ri, f"n={n}", va="center", fontsize=8, color=MUTED)
        ax.set_title(f"{s} {G.SCENARIOS[s]['label']}", loc="left", fontsize=11)
        ax.set_xlim(0, 112)
        ax.set_xticks([0, 50, 100], ["0", "50", "100 %"])
        ax.grid(axis="y", visible=False)
    axes[0].set_yticks(range(len(conds)), [SHORT[c] for c in conds])
    axes[0].invert_yaxis()          # y축 공유 — 한 번만 뒤집는다 (P0이 맨 위)
    handles = [plt.Rectangle((0, 0), 1, 1, color=col) for _, _, col in OUTCOME]
    fig.legend(handles, [lab for _, lab, _ in OUTCOME], ncol=4, loc="lower center", fontsize=10)
    fig.suptitle("시나리오 × 조건별 결과 구성 (도착 / 가장자리 앞 갇힘 / 다른 곳 멈춤 / 추락)", x=0.01, ha="left", fontweight="bold")
    fig.tight_layout(rect=(0, 0.07, 1, 0.95))
    fig.savefig(os.path.join(OUT, "fig2_outcomes.png"), dpi=170)
    plt.close(fig)


def load_sweeps():
    out = {"push": {}, "dropout": {}}
    sw = os.path.join(A.ROOT, "sweep")
    if not os.path.isdir(sw):
        return out
    for name in sorted(os.listdir(sw)):
        k, _, v = name.partition("_")
        if k in out:
            out[k][float(v)] = A.load(os.path.join(sw, name))
    return out


def fig_push(push):
    if not push:
        return
    fig, ax = plt.subplots(figsize=(8.5, 4.8))
    xs = sorted(push)
    for c in ("P1", "P1i", "P3"):
        pts = []
        for v in xs:
            rs = [r for r in push[v] if r["condition"] == c]
            for r in rs:
                y = -0.15 if r["outcome"] == "fall" else r["min_edge"]
                if r["outcome"] == "fall":
                    ax.plot(v, y, marker="x", ms=9, mew=2.2, color="#d03b3b", zorder=5)
                elif y > 1.0:          # 계단과 먼 곳에서 멈춘 회차 — 축을 늘리지 않고 위쪽 끝에 ▲
                    ax.plot(v, 0.97, "^", ms=7, color=COND_COLOR[c], zorder=5)
                else:
                    ax.plot(v, y, "o", ms=5, color=COND_COLOR[c], alpha=0.55, zorder=4)
            if rs:
                pts.append((v, st.median(-0.15 if r["outcome"] == "fall" else min(r["min_edge"], 1.0) for r in rs)))
        if pts:
            ax.plot(*zip(*pts), color=COND_COLOR[c], lw=2.2, label=SHORT[c], zorder=4)
            ax.text(pts[-1][0] + 0.03, pts[-1][1], c, va="center", fontsize=10, color=INK)
    ax.axhline(0, color="#d03b3b", lw=1.2)
    ax.text(xs[0], 0.02, "계단 가장자리", color="#d03b3b", fontsize=9)
    ax.set_xlabel("밀림 거리 [m]  (바퀴 인코더가 모르는 이동 = 위치 착각의 크기)")
    ax.set_ylabel("실제 가장자리까지 최소 거리 [m]")
    ax.set_title("밀림 거리(위치 착각 크기)에 따른 가장자리 최소 거리 — S3", loc="left")
    ax.set_xlim(xs[0] - 0.05, xs[-1] + 0.2)
    ax.set_ylim(-0.3, 1.02)
    ax.legend(loc="upper right", fontsize=10)
    fig.text(0.01, 0.01, "선 = 회차 중앙값, 점 = 회차, 빨간 × = 추락, ▲ = 1 m 넘게 떨어진 곳에서 멈춘 회차", fontsize=9, color=MUTED)
    fig.tight_layout(rect=(0, 0.03, 1, 1))
    fig.savefig(os.path.join(OUT, "fig3_push_sweep.png"), dpi=170)
    plt.close(fig)


def fig_dropout(drop):
    if not drop:
        return
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.4))
    xs = sorted(drop)
    for c in ("P2", "P3"):
        fp, tm = [], []
        for v in xs:
            rs = [r for r in drop[v] if r["condition"] == c and r["conf"]]
            if rs:
                fp.append((100 * v, st.mean(max(1, r["conf"]["FP"]) for r in rs)))
            ar = [r for r in drop[v] if r["condition"] == c and r["outcome"] == "arrived"]
            if ar:
                tm.append((100 * v, st.mean(r["time"] for r in ar)))
        if fp:
            axes[0].plot(*zip(*fp), "-o", color=COND_COLOR[c], lw=2.2, ms=7, label=SHORT[c])
        if tm:
            axes[1].plot(*zip(*tm), "-o", color=COND_COLOR[c], lw=2.2, ms=7, label=SHORT[c])
    axes[0].set_yscale("log")
    axes[0].set_title("가짜 낭떠러지 (셀-프레임, 로그)", loc="left", fontsize=12)
    axes[1].set_title("도착 시간 [s]", loc="left", fontsize=12)
    for ax in axes:
        ax.set_xlabel("검은 매트 깊이 결측률 [%]")
        ax.legend(fontsize=10)
    fig.suptitle("검은 매트 결측률에 따른 가짜 낭떠러지와 도착 시간 — S1",
                 x=0.01, ha="left", fontweight="bold")
    fig.tight_layout(rect=(0, 0, 1, 0.93))
    fig.savefig(os.path.join(OUT, "fig4_dropout_sweep.png"), dpi=170)
    plt.close(fig)


def fig_time(runs, conds):
    fig, ax = plt.subplots(figsize=(9, 4.2))
    for ci, c in enumerate(conds):
        ts = [r["time"] for r in runs if r["scenario"] == "S1" and r["condition"] == c and r["outcome"] == "arrived"]
        if not ts:
            continue
        ax.plot(ts, [ci] * len(ts), "o", ms=7, color=COND_COLOR[c], alpha=0.7)
        m = st.median(ts)
        ax.plot([m, m], [ci - 0.3, ci + 0.3], color=INK, lw=2)
        ax.text(max(ts) + 0.4, ci, f"중앙값 {m:.1f} s", va="center", fontsize=9, color=INK2)
    ax.set_yticks(range(len(conds)), [SHORT[c] for c in conds])
    ax.invert_yaxis()
    ax.set_xlabel("도착 시간 [s] (S1 평상시)")
    ax.set_title("평상시(S1) 도착 시간 — 조건별 비용", loc="left")
    ax.grid(axis="y", visible=False)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "fig5_time_s1.png"), dpi=170)
    plt.close(fig)


def main():
    os.makedirs(OUT, exist_ok=True)
    setup()
    runs = A.load(os.path.join(A.ROOT, "runs"))
    conds = [c for c in G.CONDITIONS if any(r["condition"] == c for r in runs)]
    scens = [s for s in G.SCENARIOS if any(r["scenario"] == s for r in runs)]
    fig_fall_rate(runs, conds, scens)
    fig_outcomes(runs, conds, scens)
    sw = load_sweeps()
    fig_push(sw["push"])
    fig_dropout(sw["dropout"])
    fig_time(runs, conds)
    print("→", OUT, sorted(os.listdir(OUT)))


if __name__ == "__main__":
    main()
