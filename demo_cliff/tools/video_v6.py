"""판정 기준 v6 발표용 영상 (11-21, 판정 뒤 — 판정에 쓰지 않는다, 고정 파일 아님) → results_<호스트>/figures_v6/

    python3 tools/video_v6.py        # OpenCV · Pillow (사용자 site의 numpy 2.x로 돈다)

같은 배치(월드 1 · 0.55 m · 시드 513)에서 ① 기억을 보고 안전한 만큼만 물러나는 로봇과 ③ 기억 없는 로봇의
위에서 본 영상(회차마다 저장된 video.mp4)을 같은 시뮬레이션 시각끼리 맞춰 나란히 붙인다.
구간: ③ 로봇 후진 요청 3 s 전 ~ ③ 로봇 추락 2 s 뒤. 먼저 끝난 쪽은 마지막 장면을 유지한다.
"""
import json
import os
import socket

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, "..", f"results_{socket.gethostname()}")
GROUP, SEED = "Y-w1-d055", 513
SIDES = [("P3y", "① 기억을 보고 안전한 만큼만 물러남"), ("P3n", "③ 기억 없음 — 요청대로 물러남")]
FONT = "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc"
STEP, FPS = 0.3, 8          # 원본 영상이 0.3 s마다 한 장, 8 fps로 저장돼 있다 → 약 2.4배속


def load(cond):
    d = os.path.join(ROOT, "v6", GROUP, f"S8_{cond}_{SEED}")
    r = json.load(open(os.path.join(d, "result.json")))
    cap = cv2.VideoCapture(os.path.join(d, "video.mp4"))
    frames = []
    while True:
        ok, f = cap.read()
        if not ok:
            break
        frames.append(f)
    return r, frames


def frame_at(r, frames, t):
    ft = r["frame_t"][:len(frames)]
    i = int(np.argmin([abs(x - t) for x in ft])) if t <= ft[-1] else len(frames) - 1
    return frames[i]


def status(r, t):
    ev = {e["kind"]: e["t"] for e in r["events"]}
    if "fall" in ev and t >= ev["fall"]:
        return "추락", (208, 59, 59)
    if "backup" in ev and ev["backup"] <= t <= ev.get("backup_end", 1e9):
        return "뒤로 비켜 주는 중", (42, 120, 214)
    if "return" in ev and ev["return"] <= t <= ev.get("return_end", 1e9):
        return "물러난 만큼 되돌아옴", (42, 120, 214)
    if r["outcome"] == "arrived" and t >= r["time"]:
        return "목적지 도착", (12, 163, 12)
    if "return" in ev and ev.get("backup_end", 1e9) < t < ev["return"]:
        return "물러난 자리에서 사람이 지나가길 기다림", (82, 81, 78)
    if "backup_end" in ev and t > ev["backup_end"]:
        return "다음 목적지로 이동", (82, 81, 78)
    return "목적지(계단 옆)에 서 있음", (82, 81, 78)


def banner(w, title, sub, col, font, small):
    im = Image.new("RGB", (w, 78), (252, 252, 251))
    dr = ImageDraw.Draw(im)
    dr.text((14, 8), title, font=font, fill=(11, 11, 11))
    dr.text((14, 44), sub, font=small, fill=col)
    return cv2.cvtColor(np.array(im), cv2.COLOR_RGB2BGR)


def main():
    (ra, fa), (rb, fb) = load(SIDES[0][0]), load(SIDES[1][0])
    evb = {e["kind"]: e["t"] for e in rb["events"]}
    t0, t1 = evb["backup"] - 3.0, evb.get("fall", rb["time"]) + 2.0
    font, small = ImageFont.truetype(FONT, 26), ImageFont.truetype(FONT, 22)
    h, w = fa[0].shape[:2]
    out = os.path.join(ROOT, "figures_v6", f"v6_side_by_side_{GROUP}_{SEED}.mp4")
    vw = cv2.VideoWriter(out, cv2.VideoWriter_fourcc(*"avc1"), FPS, (2 * w + 12, h + 78 + 40))
    if not vw.isOpened():
        vw = cv2.VideoWriter(out, cv2.VideoWriter_fourcc(*"mp4v"), FPS, (2 * w + 12, h + 78 + 40))
    t, n = t0, 0
    while t <= t1:
        cols = []
        for (cond, label), r, fr in ((SIDES[0], ra, fa), (SIDES[1], rb, fb)):
            s, col = status(r, t)
            cols.append(np.vstack([banner(w, label, s, col, font, small), frame_at(r, fr, t)]))
        gap = np.full((h + 78, 12, 3), 252, np.uint8)
        body = np.hstack([cols[0], gap, cols[1]])
        foot = Image.new("RGB", (body.shape[1], 40), (252, 252, 251))
        ImageDraw.Draw(foot).text((14, 8), f"월드 1 · 지도에 없는 계단 가장자리에서 0.55 m · 시드 {SEED} · 시뮬레이션 {t:5.1f} s (약 2.4배속)",
                                  font=ImageFont.truetype(FONT, 18), fill=(137, 135, 129))
        vw.write(np.vstack([body, cv2.cvtColor(np.array(foot), cv2.COLOR_RGB2BGR)]))
        if n == int((evb["backup"] + 6 - t0) / STEP):
            cv2.imwrite(out.replace(".mp4", "_still.png"), np.vstack([body, cv2.cvtColor(np.array(foot), cv2.COLOR_RGB2BGR)]))
        t += STEP
        n += 1
    vw.release()
    print(out, n, "장", f"{n / FPS:.1f} s")


if __name__ == "__main__":
    main()
