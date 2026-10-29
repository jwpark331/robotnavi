"""시뮬 점검: ground truth 몇 번 찍고, 위 카메라·깊이 이미지를 PNG로 저장."""
import math, sys, time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from PIL import Image as PImage

out = sys.argv[1] if len(sys.argv) > 1 else "."
class P(Node):
    def __init__(s):
        super().__init__("probe"); s.gt=[]; s.img=None; s.dep=None
        s.create_subscription(Odometry, "/ground_truth", lambda m: s.gt.append(m), 10)
        s.create_subscription(Image, "/overhead/overhead/image_raw", lambda m: setattr(s, "img", m), 1)
        s.create_subscription(Image, "/depth_cam/depth_cam/image_raw", lambda m: setattr(s, "dep", m), 1)
rclpy.init(); n = P(); t0 = time.time()
while time.time() - t0 < 8: rclpy.spin_once(n, timeout_sec=0.1)
for m in n.gt[::40]:
    p, o = m.pose.pose.position, m.pose.pose.orientation
    yaw = math.atan2(2*(o.w*o.z+o.x*o.y), 1-2*(o.y**2+o.z**2))
    print(f"gt t={m.header.stamp.sec}.{m.header.stamp.nanosec//10**8} x={p.x:.3f} y={p.y:.3f} z={p.z:.4f} yaw={math.degrees(yaw):.1f}")
if n.img: PImage.frombytes("RGB", (n.img.width, n.img.height), bytes(n.img.data)).save(f"{out}/overhead.png"); print("overhead saved")
if n.dep: PImage.frombytes("RGB", (n.dep.width, n.dep.height), bytes(n.dep.data)).save(f"{out}/depth_rgb.png"); print("depth rgb saved")
