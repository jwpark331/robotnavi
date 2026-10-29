"""results/runs/* → results/report.html (영상은 runs/<회차>/video.mp4를 상대 경로로 참조).

    python3 tools/make_report.py results/runs results/report.html
"""
import base64
import io
import json
import os
import subprocess
import sys
import time

from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "ws", "src", "cliff_demo"))
from cliff_demo import geometry as G  # noqa: E402

SCEN_DESC = {
    "S1": "사람 없음. 복도를 지나 모퉁이를 돌아 사무실로",
    "S2": "모퉁이 안쪽(가지 복도 입구)에 4명이 서서 대화 중, 45초 뒤 흩어짐",
    "S3": "사람 없음. 복도 중간에서 로봇이 앞으로 0.8–1.0 m 밀림 (바퀴 인코더는 모름 → AMCL이 위치를 착각)",
    "S4": "사무실 문 앞을 4명이 막고 있음(45초 뒤 흩어짐) → 계단 B 쪽으로 돌아 들어가야 함. 운영자가 계단 B keepout을 안 그림",
    "S5": "5명이 무작위 위치·시각에 나타나 복도·모퉁이·사무실 문 사이를 걸어 다님 (로봇 앞에선 멈춰 비켜 줌)",
    "S6": "S5와 같은 움직이는 사람들 + 운영자가 계단 B keepout을 안 그림",
}


def map_png(maps_dir):
    info = {}
    for ln in open(os.path.join(maps_dir, "city_hall.yaml")):
        if ":" in ln:
            k, v = ln.split(":", 1)
            info[k.strip()] = v.strip()
    im = Image.open(os.path.join(maps_dir, info["image"])).convert("L")
    buf = io.BytesIO()
    im.save(buf, "PNG")
    ox, oy = [float(v) for v in info["origin"].strip("[]").split(",")[:2]]
    return dict(png="data:image/png;base64," + base64.b64encode(buf.getvalue()).decode(), ox=ox, oy=oy,
                res=float(info["resolution"]), w=im.width, h=im.height)


def ir_summary(d):
    """ir.jsonl → IR 급정지 횟수·시간."""
    f = os.path.join(d, "ir.jsonl")
    if not os.path.exists(f):
        return None
    stops = []
    for ln in open(f):
        if ln.strip():
            stops += json.loads(ln)["stops"]
    return dict(n=len(stops), time=round(sum(b - a for a, b, _ in stops), 1), on_mat=sum(1 for *_, m in stops if m))


def load_run(d, rel, write_replay=True):
    """요약은 리포트 HTML에 넣고, 궤적·낭떠러지 기록은 <회차>/replay.json으로 따로 쓴다 (회차가 많아도 HTML이 가볍게)."""
    r = json.load(open(os.path.join(d, "result.json")))
    tr = json.load(open(os.path.join(d, "track.json")))
    rows = []
    sel = tr[::2] + ([tr[-1]] if tr and len(tr) % 2 == 0 else [])  # 5 Hz + 마지막
    for e in sel:
        g = e["gt"]
        est = e["est"] or [None, None, None]
        cov = e["cov"] or [None, None, None]
        od = e.get("odom") or [None] * 3
        rows.append([e["t"], g[0], g[1], g[2], g[3], est[0], est[1], est[2], cov[0], cov[2], e["rec"], od[0], od[1], od[2]])
    amcl_max = max([((e["gt"][0] - e["est"][0]) ** 2 + (e["gt"][1] - e["est"][1]) ** 2) ** 0.5
                    for e in tr if e["est"] and e["gt"][3] > -0.1] or [0.0])
    cliff, conf = [], None
    grid = dict(frame="map", ox=0.0, oy=1.5, res=0.1)          # 프레임 정보가 없는 옛 로그는 map 프레임
    cj = os.path.join(d, "cliff.jsonl")
    if os.path.exists(cj):
        lines = [json.loads(ln) for ln in open(cj) if ln.strip()]
        for ln in lines:
            if ln.get("meta"):
                grid = dict(frame=ln["frame"], ox=ln["ox"], oy=ln["oy"], res=ln["res"])
        lines = [ln for ln in lines if not ln.get("meta")]
        if lines:
            off = r.get("t0_sim", lines[0]["t"])           # cliff 로그는 sim 절대시각 → 목표 전송 후 경과로
            for ln in lines:
                cliff.append(dict(t=round(ln["t"] - off, 2), lethal=ln["lethal"], caution=ln["caution"]))
            conf = lines[-1]["conf"]
    cond = G.CONDITIONS[r["condition"]]
    for k in ("people",):
        r.setdefault(k, [])
    r.update(conf=conf, cliffGrid=grid, amcl_max=round(amcl_max, 3), ir=ir_summary(d),
             keepout=list(G.SCENARIOS[r["scenario"]]["keepout"]) if cond["keepout"] else [],
             video=(rel + "/video.mp4") if os.path.exists(os.path.join(d, "video.mp4")) else None,
             replay=rel + "/replay.json")
    if write_replay:
        with open(os.path.join(d, "replay.json"), "w") as f:
            json.dump(dict(track=rows, cliff=cliff), f, separators=(",", ":"))
    return r


EXCLUDED = []   # 실험 장치 결함(AMCL·컨트롤러 TF 멈춤, 시뮬 멈춤, 가짜 도착)으로 통계에서 뺀 회차
sys.path.insert(0, HERE)
from find_faults import faults  # noqa: E402


def load_dir(runs_dir, out_dir, write_replay=True):
    runs = []
    if not os.path.isdir(runs_dir):
        return runs
    bad_dirs = {os.path.abspath(x) for x, _ in faults(runs_dir)}
    for name in sorted(os.listdir(runs_dir)):
        d = os.path.join(runs_dir, name)
        if os.path.exists(os.path.join(d, "result.json")):
            r = load_run(d, os.path.relpath(d, out_dir), write_replay)
            if r["outcome"] in ("infra_fault", "sim_stalled") or os.path.abspath(d) in bad_dirs:
                EXCLUDED.append(name)
                continue
            runs.append(r)
    return runs


def load_sweeps(sweep_dir, out_dir):
    """results/sweep/{push,dropout}_<값>/<회차> → 견고성 그래프용 요약."""
    out = dict(push=[], dropout=[])
    if not os.path.isdir(sweep_dir):
        return out
    for name in sorted(os.listdir(sweep_dir)):
        kind, _, val = name.partition("_")
        if kind not in out:
            continue
        for r in load_dir(os.path.join(sweep_dir, name), out_dir):
            out[kind].append(dict(v=float(val), condition=r["condition"], seed=r["seed"], outcome=r["outcome"],
                                  time=r["time"], min_edge=r["min_edge"], amcl_max=r["amcl_max"],
                                  fp=(r["conf"] or {}).get("FP"), tp=(r["conf"] or {}).get("TP"),
                                  recoveries=r["recoveries"], ir=r["ir"], replay=r["replay"], video=r["video"],
                                  scenario=r["scenario"]))
    return out


def main(runs_dir, out_html):
    out_dir = os.path.dirname(os.path.abspath(out_html))
    runs = load_dir(runs_dir, out_dir)
    sweeps = load_sweeps(os.path.join(os.path.dirname(os.path.abspath(runs_dir)), "sweep"), out_dir)
    conf = {}
    for r in runs:
        if r["conf"]:
            c = conf.setdefault(r["condition"], dict(TP=0, FP=0, FN=0, TN=0))
            for k in c:
                c[k] += r["conf"][k]
    try:
        gz = subprocess.run(["gzserver", "--version"], capture_output=True, text=True).stdout.split("version")[1].split()[0]
    except Exception:
        gz = "11"
    maps_dir = os.path.join(HERE, "..", "ws", "src", "cliff_demo", "maps")
    data = dict(
        generated=time.strftime("%Y-%m-%d %H:%M"), gazebo=gz,
        scenarios=[s for s in G.SCENARIOS if any(r["scenario"] == s for r in runs)], conditions=list(G.CONDITIONS),
        scenLabels={k: v["label"] for k, v in G.SCENARIOS.items()}, scenDesc=SCEN_DESC,
        condLabels={k: v["label"] for k, v in G.CONDITIONS.items()},
        geom=dict(voids={k: list(v[:4]) for k, v in G.VOIDS.items()}, keepoutMargin=G.KEEPOUT_MARGIN,
                  start=list(G.START), goal=list(G.GOAL), overhead=G.OVERHEAD, robotR=G.ROBOT_R, personR=G.PERSON_R),
        map=map_png(maps_dir), runs=runs, conf=conf, sweeps=sweeps, excluded=EXCLUDED)
    tpl = open(os.path.join(HERE, "report_template.html")).read()
    html = tpl.replace("/*__DATA__*/null", json.dumps(data, ensure_ascii=False, separators=(",", ":")))
    open(out_html, "w").write(html)
    print(f"→ {out_html}  (회차 {len(runs)}개, sweep {len(sweeps['push'])}+{len(sweeps['dropout'])}개)")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "results/runs", sys.argv[2] if len(sys.argv) > 2 else "results/report.html")
