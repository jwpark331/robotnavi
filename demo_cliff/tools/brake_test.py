"""제동 시험 (판정 기준 v5 10장) — 낭떠러지 없이 직진하다 정지 명령을 줘서 실제 감속과 지연을 잰다.

    (sim.launch.py가 떠 있는 상태에서) python3 tools/brake_test.py --out <jsonl> [--speed 0.45] [--reps 3]

- /cmd_vel에 20 Hz로 speed를 보내 cruise초 달린 뒤, 0을 보내 멈춘다. 모든 명령은 ir_guard(enabled=false면 그대로 넘김)를 거쳐 바퀴로 간다.
- 시각은 모두 메시지 머리의 시각(sim)이다. 받은 시각은 /clock 콜백이 몰리면 늦어져 쓰지 않는다 (11/05 첫 시도에서 감속이 틀리게 나옴).
- 감속: 바퀴 odom 속도(50 Hz)가 정지 직전 속도의 80 %에서 20 %로 떨어지는 구간의 기울기.
- 정지 거리: 0을 보낸 순간의 위치(앞뒤 odom 표본으로 보간)부터 멈춘 자리까지. 참값(20 Hz)으로도 같이 잰다.
  지연 L = (정지 거리 − v²/2a) / v (명령 → 감속 시작, ir_guard 중계 포함).
- 비교 방식의 결과(낭떠러지 앞 정지)를 보지 않고 바퀴 설정만 확인하려는 시험이다.
"""
import argparse
import json
import math
import os
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "ws", "src", "cliff_demo"))
from cliff_demo import geometry as G  # noqa: E402


class Brake(Node):
    def __init__(self):
        super().__init__("brake_test", parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.gt, self.odom = [], []
        self.create_subscription(Odometry, "/ground_truth", lambda m: self.gt.append(self.sample(m)), 50)
        self.create_subscription(Odometry, "/odom", lambda m: self.odom.append(self.sample(m)), 50)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    @staticmethod
    def sample(m):
        p, v, st = m.pose.pose.position, m.twist.twist.linear, m.header.stamp
        return (st.sec + st.nanosec * 1e-9, p.x, p.y, math.hypot(v.x, v.y))

    def drive(self, v, dur):
        """dur초(sim) 동안 20 Hz로 v를 보낸다."""
        t_end = self.now() + dur
        while self.now() < t_end:
            msg = Twist()
            msg.linear.x = v
            self.pub.publish(msg)
            t_next = self.now() + 0.05
            while self.now() < t_next:
                rclpy.spin_once(self, timeout_sec=0.01)


def at(samples, t):
    """t 시각의 위치를 앞뒤 표본으로 선형 보간한다."""
    prev = [q for q in samples if q[0] <= t]
    nxt = [q for q in samples if q[0] > t]
    if not prev or not nxt:
        q = (prev or nxt)[-1 if prev else 0]
        return q[1], q[2]
    a, b = prev[-1], nxt[0]
    w = (t - a[0]) / (b[0] - a[0]) if b[0] > a[0] else 0.0
    return a[1] + w * (b[1] - a[1]), a[2] + w * (b[2] - a[2])


def stop_dist(samples, t0):
    x0, y0 = at(samples, t0)
    rest = [q for q in samples if q[0] > t0 and q[3] < 0.005]
    q = rest[0] if rest else samples[-1]
    return math.hypot(q[1] - x0, q[2] - y0)


def analyse(gt, odom, t0):
    odom, gt = sorted(odom), sorted(gt)
    before = [q for q in odom if t0 - 0.3 <= q[0] < t0]
    after = [q for q in odom if q[0] >= t0]
    if not before or len(after) < 5 or len(gt) < 5:
        return None
    v0 = sum(q[3] for q in before) / len(before)
    seg = [q for q in after if 0.2 * v0 <= q[3] <= 0.8 * v0]
    a = None
    if len(seg) >= 2:
        n = len(seg)
        mt, mv = sum(q[0] for q in seg) / n, sum(q[3] for q in seg) / n
        den = sum((q[0] - mt) ** 2 for q in seg)
        a = -sum((q[0] - mt) * (q[3] - mv) for q in seg) / den if den > 0 else None
    d, d_gt = stop_dist(odom, t0), stop_dist(gt, t0)
    lat = (d - v0 * v0 / (2 * a)) / v0 if a and a > 0 and v0 > 0 else None
    return dict(v0=round(v0, 4), decel=round(a, 3) if a else None, n_seg=len(seg), stop_dist=round(d, 4),
                stop_dist_gt=round(d_gt, 4), latency=round(lat, 4) if lat is not None else None)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--speed", type=float, default=0.45)
    ap.add_argument("--reps", type=int, default=3)
    ap.add_argument("--cruise", type=float, default=2.0)
    ap.add_argument("--settle", type=float, default=1.5)
    args = ap.parse_args()
    rclpy.init()
    n = Brake()
    w = time.monotonic()
    while (not n.gt or not n.odom) and time.monotonic() - w < 60:      # 로봇이 뜨고 시뮬 시계가 돌 때까지
        rclpy.spin_once(n, timeout_sec=0.1)
    if not n.gt or not n.odom:
        print("참값 · odom을 받지 못했다", file=sys.stderr)
        sys.exit(1)
    n.drive(0.0, 1.0)
    with open(args.out, "a") as f:
        for rep in range(args.reps):
            n.gt.clear()
            n.odom.clear()
            n.drive(args.speed, args.speed / min(G.WHEEL_ACC, 3.0) + args.cruise)
            t0 = n.now()
            n.drive(0.0, args.settle)
            r = analyse(n.gt, n.odom, t0)
            row = dict(wheel_acc=G.WHEEL_ACC, world=G.WORLD, speed=args.speed, rep=rep, **(r or dict(error="자료 부족")))
            f.write(json.dumps(row, ensure_ascii=False) + "\n")
            f.flush()
            print(row)
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
