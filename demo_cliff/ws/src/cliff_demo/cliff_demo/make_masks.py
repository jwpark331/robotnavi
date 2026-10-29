"""SLAM 지도(maps/city_hall.yaml)와 같은 격자로 keepout 마스크를 만든다 — 운영자가 계단에 금지 구역을 그리는 일.

    python3 -m cliff_demo.make_masks <maps 폴더>

keepout_AB: 계단 A, B 모두 (구멍 + 30 cm).  keepout_A: 계단 B를 빠뜨린 지도 (S4 "keepout 안 그린 계단").  keepout_B: 계단 A를 빠뜨린 지도 (S7·S8).
"""
import os
import sys

from cliff_demo import geometry as G


def read_yaml(path):
    info = {}
    for ln in open(path):
        if ":" in ln:
            k, v = ln.split(":", 1)
            info[k.strip()] = v.strip()
    return info


def pgm_size(path):
    with open(path, "rb") as f:
        tokens = []
        while len(tokens) < 4:
            ln = f.readline()
            if ln.startswith(b"#"):
                continue
            tokens += ln.split()
    return int(tokens[1]), int(tokens[2])


def make(maps_dir):
    info = read_yaml(os.path.join(maps_dir, "city_hall.yaml"))
    res = float(info["resolution"])
    ox, oy = [float(v) for v in info["origin"].strip("[]").split(",")[:2]]
    w, h = pgm_size(os.path.join(maps_dir, info["image"]))
    m = G.KEEPOUT_MARGIN
    for name, keys in (("keepout_AB", ("A", "B")), ("keepout_A", ("A",)), ("keepout_B", ("B",))):
        img = bytearray([254]) * (w * h)
        for k in keys:
            x0, y0, x1, y1, _ = G.VOIDS[k]
            for row in range(h):
                y = oy + (h - 1 - row + 0.5) * res
                if not (y0 - m <= y <= y1 + m):
                    continue
                for col in range(w):
                    x = ox + (col + 0.5) * res
                    if x0 - m <= x <= x1 + m:
                        img[row * w + col] = 0
        with open(os.path.join(maps_dir, name + ".pgm"), "wb") as f:
            f.write(f"P5\n{w} {h}\n255\n".encode() + bytes(img))
        with open(os.path.join(maps_dir, name + ".yaml"), "w") as f:
            f.write(f"image: {name}.pgm\nmode: scale\nresolution: {res}\norigin: [{ox}, {oy}, 0.0]\n"
                    "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n")
        print(name, w, h)


if __name__ == "__main__":
    make(sys.argv[1] if len(sys.argv) > 1 else "maps")
