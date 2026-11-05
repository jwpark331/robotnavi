"""인지 지연 주입 확인 (판정 기준 v5 10장) — Gazebo 없이 ir_guard와 cliff_detector의 지연만 시험한다.

    source env.sh && python3 tools/test_v5_delay.py [지연, 기본 0.1]

- ir_guard: 가짜 IR 판독 2회(디바운스)로 낭떠러지를 만든 뒤, 명령 차단 판정(blocked)이 켜질 때까지 걸린 시간을 잰다.
- cliff_detector: 깊이 영상 하나를 넣은 뒤 처리 함수가 불릴 때까지 걸린 시간을 잰다 (처리 함수는 시각만 적게 바꿔 끼운다).
- 지연 0이면 둘 다 바로 반영되는지(기존 동작)도 본다. 통과 기준: 지연 ± 0.02 s.
- 벽시계로 잰다 (use_sim_time = false). 실제 회차에서는 sim 시각으로 같은 코드가 돈다.
"""
import sys
import time

import rclpy
from sensor_msgs.msg import Image, Range

from cliff_demo.cliff_detector import CliffDetector
from cliff_demo.ir_guard import IrGuard

TOL = 0.02


def make(cls, **over):
    """노드의 declare_parameter 기본값을 바꿔 끼워 만든다 (노드 코드는 그대로)."""
    over = {"use_sim_time": False, **over}

    class N(cls):
        def declare_parameter(self, name, value=None, *a, **kw):
            return super().declare_parameter(name, over.get(name, value), *a, **kw)
    return N()


def bad_range():
    m = Range()
    m.range, m.max_range = 0.20, 1.0      # IR_CLIFF(0.07 m)보다 멀다 = 바닥 없음
    return m


def ir_latency(delay):
    node = make(IrGuard, sense_delay=float(delay), enabled=True)
    node.on_ir("r20", bad_range())
    node.on_ir("r20", bad_range())        # 2회 연속이라 지금부터 낭떠러지
    t0 = time.monotonic()
    while time.monotonic() - t0 < 1.0 and not node.blocked()[0]:
        time.sleep(0.001)
    dt = time.monotonic() - t0
    node.destroy_node()
    return dt


def cam_latency(delay):
    node = make(CliffDetector, sense_delay=float(delay))
    seen = []
    node.process_depth = lambda msg: seen.append(time.monotonic())
    t0 = time.monotonic()
    node.on_depth(Image())
    while time.monotonic() - t0 < 1.0 and not seen:
        rclpy.spin_once(node, timeout_sec=0.002)
    node.destroy_node()
    return (seen[0] - t0) if seen else None


def main():
    d = float(sys.argv[1]) if len(sys.argv) > 1 else 0.1
    rclpy.init()
    ok = True
    for delay in (0.0, d):
        ir, cam = ir_latency(delay), cam_latency(delay)
        good = all(x is not None and abs(x - delay) <= TOL for x in (ir, cam))
        ok &= good
        print(f"지연 {delay:.2f} s: ir_guard 차단까지 {ir:.3f} s, cliff_detector 처리까지 "
              f"{'–' if cam is None else f'{cam:.3f} s'} → {'통과' if good else '미달'}")
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
