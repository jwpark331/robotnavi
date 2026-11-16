"""기억 기반 후진 (판정 기준 v6 ①, 11/16): 후진 요청을 받으면 기억한 낭떠러지 금지 칸 앞까지만 물러난다.

/yield_request(Float32, 요청 거리 r [m]) → 갈 수 있는 거리 a를 계산 → min(r, a)만큼 0.10 m/s로 후진 → /yield_status(String, JSON).
  a: 로봇 중심이 뒤쪽 직선 위에서 /cliff_filter_mask의 금지 칸(값 100)에 닿기 전까지의 거리.
     cliff_detector의 금지 칸은 기억한 낭떠러지 칸에서 0.30 m까지 퍼져 있으므로 "기억한 가장자리에서 0.30 m 앞까지"다.
     격자 밖은 기억이 없는 곳이라 그 앞에서 멈춘다 (보수적으로).
  min(r, a) < 0.05 m면 움직이지 않는다 (거부).
Nav2 후진 기능(BackUp)은 금지 띠에 걸려 아예 거부하므로 쓰지 않고, 바퀴 odom을 보며 /cmd_vel을 직접 낸다 (ir_guard를 거쳐 바퀴로).

보는 것은 기억 마스크와 바퀴 odom뿐이다. 참값(/ground_truth)이나 계단 좌표(geometry.VOIDS)를 쓰지 않는다 —
구독 목록을 시작할 때 log_path에 남겨 성립 조건(기준 v6 4장 '참값을 안 썼는가')을 확인한다.
"""
import json
import math

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32, String

SPEED = 0.10        # 후진 속도 [m/s] — 시나리오의 Nav2 후진(geometry.YIELD_SPEED)과 같다
MIN_MOVE = 0.05     # 이보다 짧으면 움직이지 않는다 [m]
STEP = 0.01         # 뒤쪽 직선을 훑는 간격 [m]
LETHAL = 100
REAR_LOOK = 1.5     # 기록용: 요청 때 등 뒤 반원(이 거리 안)에 기억된 금지 칸 수


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))


def allowed_backup(grid, info, x, y, yaw, limit):
    """(x, y, yaw)에서 뒤로 갈 수 있는 거리 [m]. grid는 (높이, 폭) 배열, info는 (원점 x, 원점 y, 해상도).
    limit까지만 훑는다. 금지 칸이나 격자 밖을 만나면 그 직전 거리를 돌려준다."""
    ox, oy, res = info
    h, w = grid.shape
    c, s = math.cos(yaw), math.sin(yaw)
    n = int(math.ceil(limit / STEP))
    for k in range(n + 1):
        d = k * STEP
        px, py = x - d * c, y - d * s
        ix, iy = int(math.floor((px - ox) / res)), int(math.floor((py - oy) / res))
        if not (0 <= ix < w and 0 <= iy < h) or grid[iy, ix] >= LETHAL:
            return max(0.0, d - STEP)
    return limit


def rear_lethal_cells(grid, info, x, y, yaw, look=REAR_LOOK):
    """등 뒤 반원(거리 look 안)의 금지 칸 수 — 성립 조건 '기억이 있었는가'의 기록용 (판단에 쓰지 않는다)."""
    ox, oy, res = info
    iy, ix = np.nonzero(grid >= LETHAL)
    cx, cy = ox + (ix + 0.5) * res, oy + (iy + 0.5) * res
    dx, dy = cx - x, cy - y
    behind = (dx * math.cos(yaw) + dy * math.sin(yaw)) < 0
    return int((behind & (np.hypot(dx, dy) <= look)).sum())


class SafeBackup(Node):
    def __init__(self):
        super().__init__("safe_backup")
        self.log_path = self.declare_parameter("log_path", "").value
        self.grid = self.info = None
        self.pose = None
        self.job = None
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, "/cliff_filter_mask", self.on_mask, qos)
        self.create_subscription(Odometry, "/odom", self.on_odom, 20)
        self.create_subscription(Float32, "/yield_request", self.on_request, 10)
        self.cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self.status = self.create_publisher(String, "/yield_status", 10)
        self.create_timer(0.05, self.tick)
        self.write(dict(kind="start", subscriptions=sorted(s.topic_name for s in self.subscriptions)))

    def write(self, rec):
        rec = dict(t=round(self.get_clock().now().nanoseconds * 1e-9, 2), **rec)
        if self.log_path:
            with open(self.log_path, "a") as f:
                f.write(json.dumps(rec, ensure_ascii=False) + "\n")
        return rec

    def on_mask(self, m):
        self.grid = np.array(m.data, dtype=np.int16).reshape(m.info.height, m.info.width)
        self.info = (m.info.origin.position.x, m.info.origin.position.y, m.info.resolution)

    def on_odom(self, m):
        p = m.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_of(p.orientation))

    def on_request(self, m):
        r = float(m.data)
        if self.pose is None or self.grid is None:
            a, n_rear = 0.0, None
        else:
            a = allowed_backup(self.grid, self.info, *self.pose, r + MIN_MOVE)
            n_rear = rear_lethal_cells(self.grid, self.info, *self.pose)
        target = min(r, a)
        rec = self.write(dict(kind="request", r=round(r, 3), a=round(a, 3), target=round(target, 3), rear_lethal=n_rear,
                              pose=[round(v, 3) for v in self.pose] if self.pose else None, mask=self.grid is not None))
        self.status.publish(String(data=json.dumps(dict(event="plan", **rec))))
        if target < MIN_MOVE:
            self.finish("refused", 0.0)
            return
        self.job = dict(target=target, x0=self.pose[0], y0=self.pose[1],
                        t0=self.get_clock().now().nanoseconds * 1e-9, limit=target / SPEED + 5.0)

    def tick(self):
        if self.job is None or self.pose is None:
            return
        j = self.job
        moved = math.hypot(self.pose[0] - j["x0"], self.pose[1] - j["y0"])
        now = self.get_clock().now().nanoseconds * 1e-9
        if moved >= j["target"] - 0.005:
            self.finish("succeeded", moved)
        elif now - j["t0"] > j["limit"]:
            self.finish("timeout", moved)
        else:
            tw = Twist()
            tw.linear.x = -SPEED
            self.cmd.publish(tw)

    def finish(self, result, moved):
        self.job = None
        self.cmd.publish(Twist())
        rec = self.write(dict(kind="end", result=result, moved_odom=round(moved, 3)))
        self.status.publish(String(data=json.dumps(dict(event="end", **rec))))


def main():
    rclpy.init()
    n = SafeBackup()
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        n.destroy_node()
        rclpy.try_shutdown()
