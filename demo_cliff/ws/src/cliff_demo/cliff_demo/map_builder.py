"""매핑 주행: slam_toolbox가 도는 동안 정해진 경로를 돌고(RTR: 돌고-가고-돌기, L7), 끝나면 지도를 저장한다.

참값(/ground_truth)으로 운전한다 — 매핑이 목적이라 내비게이션은 쓰지 않는다. 계단 가장자리에서는 0.5 m 이상 떨어진다 (A 앞에서는 0.9 m).
저장한 지도의 원점은 slam_toolbox map 프레임(= 출발 위치)이므로, yaml origin에 출발 위치를 더해 map = world로 맞춘다.
"""
import math
import os
import subprocess

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

from cliff_demo import geometry as G

PATH = G.MAP_PATH   # (x, y, 도착 후 바라볼 방향 or None) — 월드마다 geometry에 있다


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class Builder(Node):
    def __init__(self):
        super().__init__("map_builder")
        self.declare_parameter("map_path", "")
        self.pose = None
        self.create_subscription(Odometry, "/ground_truth", self.on_gt, 10)
        self.cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self.k = 0
        self.phase = "go"
        self.t_wait = None
        self.timer = self.create_timer(0.05, self.step)
        self.done = False

    def on_gt(self, m):
        p = m.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_of(p.orientation))

    def step(self):
        if self.pose is None or self.done:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.t_wait is None:
            self.t_wait = now + 4.0          # slam_toolbox가 첫 스캔을 잡도록 잠깐 기다림
        if now < self.t_wait:
            return
        x, y, th = self.pose
        tx, ty, face = PATH[self.k]
        tw = Twist()
        if self.phase == "go":
            d = math.hypot(tx - x, ty - y)
            err = wrap(math.atan2(ty - y, tx - x) - th)
            if d < 0.08:
                self.phase = "face" if face is not None else "next"
            elif abs(err) > 0.25:
                tw.angular.z = 0.7 * math.copysign(1, err)
            else:
                tw.linear.x = min(0.35, 0.8 * d)
                tw.angular.z = 1.5 * err
        if self.phase == "face":
            err = wrap(face - th)
            if abs(err) < 0.05:
                self.phase = "next"
            else:
                tw.angular.z = max(-0.6, min(0.6, 1.5 * err))
        if self.phase == "next":
            self.k += 1
            self.phase = "go"
            if self.k == len(PATH):
                self.cmd.publish(Twist())
                self.finish()
                return
        self.cmd.publish(tw)

    def finish(self):
        self.done = True
        path = self.get_parameter("map_path").value
        self.get_logger().info("경로 끝. 지도 저장 → " + path)
        os.makedirs(os.path.dirname(path), exist_ok=True)
        subprocess.run(["ros2", "run", "nav2_map_server", "map_saver_cli", "-f", path,
                        "--ros-args", "-p", "use_sim_time:=true"], check=False)
        yml = path + ".yaml"
        if os.path.exists(yml):
            lines = open(yml).read().splitlines()
            out = []
            for ln in lines:
                if ln.startswith("origin:"):
                    ox, oy, oth = [float(v) for v in ln.split("[")[1].split("]")[0].split(",")]
                    ln = f"origin: [{ox + G.START[0]:.4f}, {oy + G.START[1]:.4f}, {oth}]"
                out.append(ln)
            open(yml, "w").write("\n".join(out) + "\n")
            self.get_logger().info("지도 원점을 world에 맞춤: " + yml)
        raise SystemExit


def main():
    rclpy.init()
    n = Builder()
    try:
        rclpy.spin(n)
    except SystemExit:
        pass
    n.destroy_node()
    rclpy.try_shutdown()
