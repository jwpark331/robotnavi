"""판정 기준 v6 사전 시험 1번 (8장): ① 로봇의 '갈 수 있는 거리' 계산 단위 시험. 시뮬레이션 없이 돈다.

    python3 tools/test_v6_backup.py

cliff_detector와 같은 방식으로 금지 칸(낭떠러지 칸 + 0.30 m)을 그린 마스크를 만들고, 손으로 잰 값과 ±0.05 m 안인지 본다.
  - 낭떠러지 칸: y < 5.4 (월드 1 계단 A 북쪽 가장자리, 0.1 m 격자)
  - 로봇: 북쪽을 보고 서서 가장자리를 등진다. 중심이 가장자리에서 d면, 금지 칸은 가장자리 + 0.30 m까지라 손 계산 a ≈ d − 0.30
"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "ws", "src", "cliff_demo"))
from cliff_demo.safe_backup import allowed_backup, rear_lethal_cells  # noqa: E402

RES, OX, OY = 0.1, 6.0, 2.0
NX, NY = 60, 80
EDGE_Y, INFLATE = 5.4, 0.30


def mask():
    cliff = np.zeros((NY, NX), bool)
    for iy in range(NY):
        if OY + (iy + 0.5) * RES < EDGE_Y:
            cliff[iy, 20:50] = True                     # x 8.0–11.0
    grid = np.zeros((NY, NX), np.int16)
    r = int(math.ceil(INFLATE / RES))
    for i in range(-r, r + 1):
        for j in range(-r, r + 1):
            if math.hypot(i, j) * RES <= INFLATE + 1e-9:
                grid[np.roll(np.roll(cliff, i, 0), j, 1)] = 100   # cliff_detector.publish_mask와 같은 모양
    return grid


def main():
    g = mask()
    info = (OX, OY, RES)
    ok = True
    print("| 가장자리 거리 d | 요청 r | 계산 a | 손 계산 | 차이 | 결과 |")
    print("|---:|---:|---:|---:|---:|---|")
    for d in (0.35, 0.55, 0.80, 1.10, 1.60):
        for r in (0.6, 0.9):
            y = EDGE_Y + d
            a = allowed_backup(g, info, 9.0, y, math.pi / 2, r + 0.05)
            hand = min(max(d - INFLATE, 0.0), r + 0.05)
            # 격자 칸 경계 때문에 금지 띠 끝이 칸 단위(0.1 m)로 잘린다 → ±0.05 m 안이면 통과
            good = abs(a - hand) <= 0.05 + 1e-9
            ok &= good
            print(f"| {d:.2f} | {r:.1f} | {a:.2f} | {hand:.2f} | {a - hand:+.2f} | {'통과' if good else '실패'} |")
    # 기억이 없으면(빈 마스크) 격자 끝까지 갈 수 있다 → 요청 거리만큼
    a0 = allowed_backup(np.zeros_like(g), info, 9.0, 5.95, math.pi / 2, 0.95)
    print(f"\n빈 마스크: a = {a0:.2f} (요청 상한 0.95와 같아야 함) → {'통과' if abs(a0 - 0.95) < 1e-6 else '실패'}")
    ok &= abs(a0 - 0.95) < 1e-6
    n = rear_lethal_cells(g, info, 9.0, 5.95, math.pi / 2)
    print(f"등 뒤 1.5 m 반원의 금지 칸: {n}개 (0보다 커야 함) → {'통과' if n > 0 else '실패'}")
    ok &= n > 0
    print("\n전체:", "통과" if ok else "실패")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
