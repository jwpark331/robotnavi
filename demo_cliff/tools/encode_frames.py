"""ffmpeg가 없는 컴퓨터용: 폴더 아래 frames/*.jpg를 video.mp4(8 fps, run_all.sh와 같음, mp4v 코덱)로 묶고 frames를 지운다.

    python3 tools/encode_frames.py <폴더>
"""
import glob
import os
import shutil
import sys

import cv2

root = sys.argv[1]
for fdir in sorted(glob.glob(os.path.join(root, "**", "frames"), recursive=True)):
    jpgs = sorted(glob.glob(os.path.join(fdir, "*.jpg")))
    if not jpgs:
        continue
    first = cv2.imread(jpgs[0])
    h, w = first.shape[:2]
    out = os.path.join(os.path.dirname(fdir), "video.mp4")
    vw = cv2.VideoWriter(out, cv2.VideoWriter_fourcc(*"mp4v"), 8, (w, h))
    for j in jpgs:
        img = cv2.imread(j)
        if img is not None:
            vw.write(img)
    vw.release()
    if os.path.getsize(out) > 0:
        shutil.rmtree(fdir)
    print(out, len(jpgs), "frames")
