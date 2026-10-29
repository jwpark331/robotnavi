"""하향 IR 낭떠러지 센서 + 급정지 (룸바식 비교군, 조건 P1i).

/cmd_vel(Nav2 출력) → /cmd_vel_hw(바퀴). enabled=false면 그대로 넘기기만 한다(모든 조건에서 같은 경로를 거치게).
명령 감시: /cmd_vel이 CMD_TIMEOUT(0.3 s) 동안 안 오면 0을 보낸다. Gazebo 차동구동 플러그인은 마지막 명령을 끝없이 유지해서,
위치추정이 잠깐 멈춰 Nav2가 명령을 못 내면 로봇이 눈을 감고 직진한다. 실제 모터 제어기에는 이 감시가 기본으로 있다.
enabled=true면 앞쪽 IR 4개 중 하나라도 바닥이 IR_CLIFF보다 멀거나 반사가 없으면 전진(+x)만 막는다. 회전·후진은 허용.
2회 연속(50–100 ms) 같은 판정일 때만 상태를 바꾼다 (실제 센서의 디바운스).
plus=true (P3+, 피드백 1 대응): ① cliff_detector가 보낸 속도 상한(/cliff_speed_cap)으로 전진 속도를 자른다
  ② IR이 낭떠러지를 보면 그 지점을 /ir_cliff로 알려 기억에 CLIFF로 남기고 ③ REVERSE_DIST(0.4 m)만큼 뒤로 물러난 뒤 Nav2에 맡긴다.

Gazebo 광선은 검은 매트에서도 반사된다. artifacts=true면 참값 위치로 매트 위 빔을 골라 MAT_DROPOUT 확률로 '반사 없음'으로 바꾼다
(룸바가 검은 카펫을 낭떠러지로 보는 것과 같은 현상). 판단에는 참값을 쓰지 않는다.
"""
import json
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Range
from std_msgs.msg import Float32

from cliff_demo import geometry as G

NAMES = {"r60": -60, "r20": -20, "l20": 20, "l60": 60}
CMD_TIMEOUT = 0.3
# 후진 거리: IR(중심 앞 0.24 m)이 기록한 낭떠러지 칸 둘레 0.3 m는 금지이고, Nav2 컨트롤러는 몸 전체(반지름 0.25 m)가 그 밖에 있어야
# 움직인다 → 중심이 0.3 + 0.25 = 0.55 m 넘게 떨어져야 한다 → 최소 0.55 − 0.24 = 0.31 m, 여유를 둬 0.4 m.
# (피드백 원안 0.2 m로는 금지 띠 안에 남아 '유효한 경로 없음'으로 갇혔다 — 10/21 시험)
REVERSE_DIST, REVERSE_SPEED = 0.40, 0.10
CAP_STALE, CAP_DEFAULT = 0.5, 0.20   # 속도 상한이 0.5 s 넘게 안 오면 보수적으로 0.2 m/s


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))


class IrGuard(Node):
    def __init__(self):
        super().__init__("ir_guard")
        self.enabled = self.declare_parameter("enabled", False).value
        self.artifacts = self.declare_parameter("artifacts", True).value
        self.dropout = self.declare_parameter("mat_dropout", G.MAT_DROPOUT).value
        self.log_path = self.declare_parameter("log_path", "").value
        self.plus = self.declare_parameter("plus", False).value
        self.cap, self.cap_t, self.n_capped = None, None, 0
        self.rev_until = None
        self.n_reverse = 0
        self.ir_pub = self.create_publisher(PointStamped, "/ir_cliff", 10)
        if self.plus:
            self.create_subscription(Float32, "/cliff_speed_cap", self.on_cap, 10)
        self.rng = np.random.default_rng(self.declare_parameter("seed", 1).value + 101)
        self.raw = {n: False for n in NAMES}       # 이번 판독이 낭떠러지인가
        self.prev = {n: False for n in NAMES}
        self.cliff = False
        self.gt = None
        self.stop_since = None
        self.stops = []                             # [시작 sim 시각, 끝, 매트 위였나]
        self.n_block = 0
        self.pub = self.create_publisher(Twist, "/cmd_vel_hw", 10)
        self.last_cmd = None
        self.timed_out = 0
        self.create_timer(0.05, self.watchdog)
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd, 10)
        self.create_subscription(Odometry, "/ground_truth", self.on_gt, 10)
        for n in NAMES:
            self.create_subscription(Range, f"/ir/{n}", lambda m, n=n: self.on_ir(n, m), qos_profile_sensor_data)
        self.log = open(self.log_path, "a") if self.log_path else None
        self.get_logger().info(f"ir_guard enabled={self.enabled}")

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_gt(self, m):
        p = m.pose.pose
        self.gt = (p.position.x, p.position.y, yaw_of(p.orientation))

    def on_ir(self, name, m):
        r = m.range
        bad = (not math.isfinite(r)) or r >= m.max_range - 1e-3 or r > G.IR_CLIFF
        on_mat = False
        if self.artifacts and self.gt is not None:
            x, y, th = self.gt
            a = th + math.radians(NAMES[name])
            wx, wy = x + G.IR_RADIUS * math.cos(a), y + G.IR_RADIUS * math.sin(a)
            on_mat = G.in_rect(wx, wy, G.DARK_MAT)
            if on_mat and self.rng.random() < self.dropout:
                bad = True
        cur = bad and self.prev[name]               # 2회 연속
        self.prev[name] = bad
        self.raw[name] = cur
        was = self.cliff
        self.cliff = any(self.raw.values())
        if cur and self.plus and self.enabled:                  # IR이 본 낭떠러지 지점을 기억으로 보낸다
            p = PointStamped()
            p.header.frame_id = "base_footprint"
            p.header.stamp = self.get_clock().now().to_msg()
            a = math.radians(NAMES[name])
            p.point.x, p.point.y = G.IR_RADIUS * math.cos(a), G.IR_RADIUS * math.sin(a)
            self.ir_pub.publish(p)
        if self.cliff and not was:
            self.stop_since = (self.now(), on_mat)
            if self.plus and self.enabled:                      # 물러났다가 (기억된 낭떠러지를 피해) 다시 짠 경로로
                self.rev_until = self.now() + REVERSE_DIST / REVERSE_SPEED
                self.n_reverse += 1
        elif was and not self.cliff and self.stop_since:
            self.stops.append([round(self.stop_since[0], 2), round(self.now(), 2), self.stop_since[1]])
            self.stop_since = None
            self.write_log()

    def on_cap(self, m):
        self.cap, self.cap_t = m.data, self.now()

    def reversing(self):
        if self.rev_until is not None and self.now() < self.rev_until:
            out = Twist()
            out.linear.x = -REVERSE_SPEED
            self.pub.publish(out)
            return True
        self.rev_until = None
        return False

    def watchdog(self):
        if self.reversing():
            return
        if self.last_cmd is not None and self.now() - self.last_cmd > CMD_TIMEOUT:
            self.pub.publish(Twist())
            self.last_cmd = None
            self.timed_out += 1

    def on_cmd(self, m):
        self.last_cmd = self.now()
        if self.reversing():
            return
        out = Twist()
        out.linear.x, out.angular.z = m.linear.x, m.angular.z
        if self.plus:
            cap = self.cap if self.cap_t is not None and self.now() - self.cap_t < CAP_STALE else CAP_DEFAULT
            if out.linear.x > cap:
                out.linear.x = cap
                self.n_capped += 1
        if self.enabled and self.cliff and out.linear.x > 0.0:
            out.linear.x = 0.0
            self.n_block += 1
        self.pub.publish(out)

    def write_log(self):
        if self.log:
            self.log.write(json.dumps(dict(t=round(self.now(), 2), stops=self.stops[-1:], n_block=self.n_block,
                                           n_capped=self.n_capped, n_reverse=self.n_reverse)) + "\n")
            self.log.flush()


def main():
    rclpy.init()
    n = IrGuard()
    try:
        # 로봇이 가장자리에 걸려 넘어질 때 깨진 메시지가 들어오면 rclpy가 RuntimeError로 노드를 죽인다(기존 77회, 모두 가장자리 6 cm 안에서).
        # 이 노드는 모든 조건의 명령 중계·감시를 맡으므로 그 메시지만 버리고 계속 돈다 (10/21).
        while rclpy.ok():
            try:
                rclpy.spin_once(n, timeout_sec=0.1)
            except RuntimeError as e:
                n.get_logger().warn(f"메시지 변환 실패, 건너뜀: {e}")
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if n.log:
            n.log.close()
        n.destroy_node()
        rclpy.try_shutdown()
