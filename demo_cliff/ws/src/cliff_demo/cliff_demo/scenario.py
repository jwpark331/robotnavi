"""시나리오 실행: 사람 배치 → Nav2에 목표(사무실) 전달 → 참값으로 추락·도착 판정 → 기록.

로봇은 참값을 모른다. 참값(/ground_truth)은 판정·기록과 S3의 '밀림' 주입에만 쓴다.
  S3 밀림: 로봇이 x = PUSH_TRIGGER_X를 지나면 Gazebo에서 로봇을 앞으로 0.8–1.0 m 옮긴다. 바퀴는 돌지 않았으니
  인코더 odom은 모른다 → AMCL이 스캔으로 알아채지 못하면 지도 위 자기 위치를 그만큼 뒤로 착각한다.
"""
import json
import math
import os
import time

import numpy as np
import rclpy
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import DeleteEntity, SetEntityState, SpawnEntity
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import Odometry
from PIL import Image as PImage
from rclpy.time import Time
from rcl_interfaces.msg import Log
from sensor_msgs.msg import Image
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener

from cliff_demo import geometry as G
from cliff_demo import walkers as W

PERSON_SDF = """<sdf version="1.6"><model name="{name}"><static>true</static><pose>{x} {y} 0.85 0 0 0</pose>
<link name="l"><collision name="c"><geometry><cylinder><radius>{r}</radius><length>1.7</length></cylinder></geometry></collision>
<visual name="v"><geometry><cylinder><radius>{r}</radius><length>1.7</length></cylinder></geometry>
<material><ambient>0.16 0.36 0.78 1</ambient><diffuse>0.16 0.36 0.78 1</diffuse></material></visual></link></model></sdf>"""


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))


def tilt_of(q):
    roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x ** 2 + q.y ** 2))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
    return max(abs(roll), abs(pitch))


def pose_stamped(nav, x, y, yaw):
    p = PoseStamped()
    p.header.frame_id = "map"
    p.header.stamp = nav.get_clock().now().to_msg()
    p.pose.position.x, p.pose.position.y = float(x), float(y)
    p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return p


class Runner:
    def __init__(self):
        self.nav = BasicNavigator()
        n = self.nav
        self.scen = n.declare_parameter("scenario", "S1").value
        self.cond = n.declare_parameter("condition", "P0").value
        self.seed = n.declare_parameter("seed", 1).value
        self.out = n.declare_parameter("out_dir", "/tmp/cliff_run").value
        self.timeout = n.declare_parameter("timeout", 150.0).value
        self.frames = n.declare_parameter("record_frames", True).value
        self.push_fixed = n.declare_parameter("push_dx", -1.0).value
        self.mat_dropout = n.declare_parameter("mat_dropout", G.MAT_DROPOUT).value
        self.void_dropout = n.declare_parameter("void_dropout", G.VOID_DROPOUT).value
        self.floor_mu = n.declare_parameter("floor_mu", G.FLOOR_MU).value
        self.pitch_k = n.declare_parameter("pitch_k", 0.0).value
        os.makedirs(os.path.join(self.out, "frames"), exist_ok=True)
        self.gt = self.gt_q = None
        self.amcl = None
        self.img = None
        n.create_subscription(Odometry, "/ground_truth", self.on_gt, 10)
        n.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self.on_amcl, 10)
        self.odom = None
        n.create_subscription(Odometry, "/odom", self.on_odom, 10)
        n.create_subscription(Image, "/overhead/overhead/image_raw", lambda m: setattr(self, "img", m), 1)
        self.spawn = n.create_client(SpawnEntity, "/spawn_entity")
        self.delete = n.create_client(DeleteEntity, "/delete_entity")
        self.setstate = n.create_client(SetEntityState, "/set_entity_state")
        self.people = []
        self.walkers = []
        self.walk_fut = {}
        # Nav2 컨트롤러가 TF를 못 받는지 감시: 컨트롤러는 TF가 오래되면 20 Hz로 'Transform data too old'를 남긴다
        self.too_old = []
        n.create_subscription(Log, "/rosout", self.on_log, 100)
        # 계측: /tf를 직접 받아 AMCL이 보낸 map→odom의 마지막 시각 (TF 버퍼를 거치지 않은 원본)
        self.raw_mo = None
        self.raw_mo_n = 0
        n.create_subscription(TFMessage, "/tf", self.on_tf, 100)
        self.tf = Buffer()
        self.tfl = TransformListener(self.tf, n)

    def on_gt(self, m):
        p = m.pose.pose
        self.gt = (p.position.x, p.position.y, yaw_of(p.orientation), p.position.z, tilt_of(p.orientation))
        self.gt_q = p.orientation

    def on_tf(self, m):
        for t in m.transforms:
            if t.header.frame_id == "map" and t.child_frame_id == "odom":
                self.raw_mo = t.header.stamp.sec + t.header.stamp.nanosec * 1e-9
                self.raw_mo_n += 1

    def on_log(self, m):
        if "Transform data too old" in m.msg:
            self.too_old.append(time.monotonic())

    def on_odom(self, m):
        p = m.pose.pose
        self.odom = (p.position.x, p.position.y, yaw_of(p.orientation))

    def on_amcl(self, m):
        p = m.pose.pose
        c = m.pose.covariance
        self.amcl = (p.position.x, p.position.y, yaw_of(p.orientation), c[0], c[1], c[7])

    def now(self):
        return self.nav.get_clock().now().nanoseconds * 1e-9

    def call(self, client, req, timeout=5.0):
        client.wait_for_service(timeout_sec=30.0)
        fut = client.call_async(req)
        rclpy.spin_until_future_complete(self.nav, fut, timeout_sec=timeout)
        return fut.result()

    def place_people(self):
        rng = np.random.default_rng(self.seed)
        k = 0
        for g in G.SCENARIOS[self.scen]["groups"]:
            for x, y in G.GROUPS[g]:
                x += rng.uniform(-G.GROUP_JITTER, G.GROUP_JITTER)
                y += rng.uniform(-G.GROUP_JITTER, G.GROUP_JITTER)
                req = SpawnEntity.Request()
                req.name = f"person{k}"
                req.xml = PERSON_SDF.format(name=req.name, x=x, y=y, r=G.PERSON_R)
                req.reference_frame = "world"
                self.call(self.spawn, req)
                self.people.append((req.name, round(x, 3), round(y, 3)))
                k += 1
        if G.SCENARIOS[self.scen].get("walkers"):
            # 움직이는 사람: 건물 밖 대기 자리에 만들어 두고, 나타날 때 옮긴다
            for k in range(W.N_WALKERS):
                w = W.Walker(k, self.seed)
                req = SpawnEntity.Request()
                req.name = w.name
                req.xml = PERSON_SDF.format(name=w.name, x=w.park[0], y=w.park[1], r=G.PERSON_R)
                req.reference_frame = "world"
                self.call(self.spawn, req)
                self.walkers.append(w)

    def move_walkers(self, t, dt):
        robot = self.gt[:2] if self.gt is not None else None
        for w in self.walkers:
            before = (w.pos, w.visible)
            others = [o.pos for o in self.walkers if o is not w and o.visible]
            w.step(t, dt, robot, others)
            if (w.pos, w.visible) == before:
                continue
            f = self.walk_fut.get(w.name)
            if f is not None and not f.done():
                continue                       # 앞 요청이 아직 처리 중이면 이번 갱신은 건너뛴다 (다음에 최신 위치로)
            st = EntityState()
            st.name = w.name
            st.reference_frame = "world"
            st.pose.position.x, st.pose.position.y, st.pose.position.z = float(w.pos[0]), float(w.pos[1]), 0.85
            st.pose.orientation.w = 1.0
            req = SetEntityState.Request()
            req.state = st
            self.walk_fut[w.name] = self.setstate.call_async(req)

    def push(self, dx):
        x, y, yaw, z, _ = self.gt
        st = EntityState()
        st.name = "servi"
        st.reference_frame = "world"
        st.pose.position.x, st.pose.position.y, st.pose.position.z = x + dx, y, 0.005
        st.pose.orientation = self.gt_q
        req = SetEntityState.Request()
        req.state = st
        self.call(self.setstate, req)

    def tf_stamp(self):
        try:
            st = self.tf.lookup_transform("map", "odom", Time()).header.stamp
        except Exception:
            return None
        return st.sec + st.nanosec * 1e-9

    def est_tf(self):
        try:
            tr = self.tf.lookup_transform("map", "base_footprint", Time()).transform
        except Exception:
            return None
        return (tr.translation.x, tr.translation.y, yaw_of(tr.rotation))

    def wait_fresh_tf(self, need=2.0, limit=60.0):
        """map→base_footprint가 need 초 동안 계속 최신일 때까지 기다린다.
        Nav2 Humble은 출발 순간 map→odom이 오래되면 목표를 (0,0)으로 변환해 '바로 도착'으로 끝내는 경합이 있다."""
        t_start = self.now()
        ok_since = None
        while self.now() - t_start < limit:
            rclpy.spin_once(self.nav, timeout_sec=0.1)
            try:
                st = self.tf.lookup_transform("map", "base_footprint", Time()).header.stamp
                fresh = self.now() - (st.sec + st.nanosec * 1e-9) < 0.5
            except Exception:
                fresh = False
            if fresh:
                ok_since = ok_since if ok_since is not None else self.now()
                if self.now() - ok_since >= need:
                    return True
            else:
                ok_since = None
        return False

    def wait_ready(self, limit=120.0):
        """Nav2 준비 대기. BasicNavigator.waitUntilNav2Active()는 lifecycle get_state 응답을 시간 제한 없이 기다려
        응답이 한 번 유실되면 영원히 멈춘다 → amcl_pose 수신 + navigate_to_pose 액션 서버만 확인한다."""
        nav = self.nav
        w0 = time.monotonic()
        last_pub, n_pub = 0.0, 0
        while time.monotonic() - w0 < limit:
            # 초기 위치는 3초에 한 번, 최대 5번만 — 한꺼번에 여러 번 받으면 AMCL이 스캔 처리를 멈춘 적이 있다
            if self.amcl is None and n_pub < 5 and time.monotonic() - last_pub > 3.0:
                nav._setInitialPose()
                last_pub, n_pub = time.monotonic(), n_pub + 1
            rclpy.spin_once(nav, timeout_sec=0.5)
            if self.amcl is not None and nav.nav_to_pose_client.wait_for_server(timeout_sec=0.5):
                return True
        return False

    def run(self):
        nav = self.nav
        self.place_people()
        rng = np.random.default_rng(self.seed + 1)
        push_dx = rng.uniform(*G.PUSH_DX) if self.push_fixed < 0 else self.push_fixed
        # 목적지: 보통은 사무실. S7·S8은 계단 옆 지점(시드마다 ±5 cm), S8은 그다음 비켜 주기 → 사무실 (두 구간)
        sc = G.SCENARIOS[self.scen]
        g = sc.get("goal")
        goal = list(getattr(G, g) if isinstance(g, str) else (g or G.GOAL))
        rg = np.random.default_rng(self.seed + 2)            # S7·S8에서만 뽑는다 → 다른 시나리오는 이전과 같다
        if sc.get("goal_jitter"):
            goal[0] += rg.uniform(-sc["goal_jitter"], sc["goal_jitter"])
            goal[1] += rg.uniform(-sc["goal_jitter"], sc["goal_jitter"])
        legs = [tuple(goal)] + ([G.GOAL] if sc.get("yield_back") else [])
        back_dist = rg.uniform(*G.YIELD_BACK) if sc.get("yield_back") else None
        leg, phase, wait_until, blockers = 0, "go", None, []
        nav.setInitialPose(pose_stamped(nav, *G.START))
        if not self.wait_ready():
            nav.get_logger().error("Nav2가 준비되지 않음")
        self.wait_fresh_tf()
        t0 = self.now()
        nav.goToPose(pose_stamped(nav, *legs[leg]))
        resent, retried, t_sent = 0, 0, 0.0
        log, events = [], []
        outcome, fell_at, pushed = None, None, False
        next_log = next_frame = 0.0
        fi = 0
        frame_t = []
        left = False
        recov = 0
        min_edge = 9.0
        w_start = time.monotonic()
        w_fell = None
        last_t, w_last_t = -1.0, time.monotonic()
        stale_since = None
        walk_t = 0.0
        while True:
            done = nav.isTaskComplete()
            t = self.now() - t0
            # 벽시계 안전장치: 로봇이 계단을 구르면 물리 엔진이 크게 느려지거나 멈출 수 있다
            if t > last_t + 1e-6:
                last_t, w_last_t = t, time.monotonic()
            elif time.monotonic() - w_last_t > 15.0:
                # 중심이 계단 구멍 위에 걸린 채 물리 엔진이 멈췄으면 장치 결함이 아니라 추락이다 (넘어지는 순간 접촉 계산이 폭주)
                if fell_at is None and self.gt is not None and G.void_depth(self.gt[0], self.gt[1]) > 0:
                    fell_at, outcome = t, "fall"
                    events.append(dict(t=round(t, 2), kind="fall", x=round(self.gt[0], 3), y=round(self.gt[1], 3),
                                       why="sim_stalled_over_void"))
                outcome = outcome or "sim_stalled"
                events.append(dict(t=round(t, 2), kind="sim_stalled"))
                break
            # 인프라 결함 감시: AMCL의 map→odom이 3초 넘게 멈춘 상태가 5초 이어지면 로봇 판단이 아니라 실험 장치 문제
            st = self.tf_stamp()
            if st is not None and self.now() - st > 3.0 and fell_at is None:
                stale_since = stale_since if stale_since is not None else self.now()
                if self.now() - stale_since > 5.0:
                    outcome = "infra_fault"
                    events.append(dict(t=round(t, 2), kind="infra_fault"))
                    nav.cancelTask()
                    break
            else:
                stale_since = None
            # 컨트롤러 쪽 TF 정지: 최근 8초(벽시계)에 경고가 100개 넘음 = 5초 넘게 위치를 못 받는 중
            self.too_old = [x for x in self.too_old if time.monotonic() - x < 8.0]
            if len(self.too_old) > 100 and fell_at is None:
                outcome = "infra_fault"
                events.append(dict(t=round(t, 2), kind="infra_fault", why="controller_tf"))
                nav.cancelTask()
                break
            if time.monotonic() - w_start > self.timeout * 3 + 60:
                nav.cancelTask()
                outcome = outcome or "timeout"
                break
            fb = nav.getFeedback()
            if fb is not None and hasattr(fb, "number_of_recoveries"):      # 물러나기(BackUp) 중 피드백은 형식이 다르다
                if fb.number_of_recoveries > recov:
                    events.append(dict(t=round(t, 2), kind="recovery", n=fb.number_of_recoveries))
                recov = fb.number_of_recoveries
            if self.walkers and fell_at is None and t - walk_t >= W.UPDATE_DT:
                self.move_walkers(t, max(0.0, min(0.5, t - walk_t)))
                walk_t = t
            if self.gt is not None:
                x, y, yaw, z, tilt = self.gt
                if fell_at is None:
                    min_edge = min(min_edge, -G.void_depth(x, y))
                if fell_at is None and (z < -0.10 or tilt > 0.6):
                    fell_at = t
                    w_fell = time.monotonic()
                    outcome = "fall"
                    events.append(dict(t=round(t, 2), kind="fall", x=round(x, 3), y=round(y, 3)))
                    nav.cancelTask()
                if (G.SCENARIOS[self.scen]["push"] and not pushed and fell_at is None
                        and x >= G.PUSH_TRIGGER_X and abs(y - 4.2) < 0.9 and t > 2.0):
                    self.push(push_dx)
                    pushed = True
                    events.append(dict(t=round(t, 2), kind="push", dx=round(push_dx, 3)))
                if t >= next_log:
                    next_log = t + 0.1
                    a = self.amcl
                    # 로봇이 지금 믿는 위치 = TF map→base_footprint (AMCL map→odom ∘ 바퀴 odom).
                    # /amcl_pose는 AMCL이 갱신할 때(0.25 m마다)만 나와서 움직이는 중 비교하면 오차가 부풀려진다.
                    est = self.est_tf()
                    if est is None and a:
                        est = a[:3]
                    log.append(dict(t=round(t, 2), gt=[round(x, 3), round(y, 3), round(yaw, 3), round(z, 3)],
                                    est=[round(v, 3) for v in est] if est else None,
                                    cov=[round(v, 5) for v in a[3:]] if a else None, rec=recov,
                                    odom=[round(v, 3) for v in self.odom] if self.odom else None,
                                    mo=round(self.raw_mo, 2) if self.raw_mo else None, mo_n=self.raw_mo_n,
                                    old=len(self.too_old),
                                    **({"ped": [[round(w.pos[0], 2), round(w.pos[1], 2)] for w in self.walkers if w.visible]}
                                       if self.walkers else {})))
            if self.frames and self.img is not None and t >= next_frame:
                next_frame = t + 0.25
                m = self.img
                PImage.frombytes("RGB", (m.width, m.height), bytes(m.data)).save(
                    os.path.join(self.out, "frames", f"{fi:05d}.jpg"), quality=82)
                frame_t.append(round(t, 2))
                fi += 1
            if self.people and not left and t >= G.GROUP_LEAVE_T:
                for name, _, _ in self.people:
                    req = DeleteEntity.Request()
                    req.name = name
                    self.call(self.delete, req)
                left = True
                events.append(dict(t=round(t, 2), kind="leave"))
            if fell_at is not None and (t > fell_at + 4.0 or time.monotonic() - w_fell > 12.0):
                break
            if phase == "backup" and done and fell_at is None:
                res_b = nav.getResult()
                events.append(dict(t=round(t, 2), kind="backup_end",
                                   result="succeeded" if res_b == TaskResult.SUCCEEDED else "stopped"))
                phase, wait_until = "wait", t + G.YIELD_REST
            if phase == "wait":
                rclpy.spin_once(nav, timeout_sec=0.1)           # 끝난 작업만 있으면 isTaskComplete가 돌지 않아 /clock이 멈춘 것처럼 보인다
                if fell_at is None and t >= wait_until:
                    for name in blockers:                       # 사람들이 지나갔다 → 사무실로
                        req = DeleteEntity.Request()
                        req.name = name
                        self.call(self.delete, req)
                    blockers = []
                    leg += 1
                    retried = 0
                    events.append(dict(t=round(t, 2), kind="leg", n=leg + 1))
                    nav.goToPose(pose_stamped(nav, *legs[leg]))
                    t_sent = self.now() - t0
                    phase = "go"
                continue
            if done and fell_at is None:
                est = self.est_tf()
                far = est is not None and math.hypot(est[0] - legs[leg][0], est[1] - legs[leg][1]) > 0.6
                if nav.getResult() == TaskResult.SUCCEEDED and far and resent >= 6:
                    outcome = "infra_fault"
                    events.append(dict(t=round(t, 2), kind="infra_fault", why="spurious_success"))
                    break
                if nav.getResult() == TaskResult.SUCCEEDED and far:
                    # '성공'인데 로봇이 믿는 위치조차 목표에서 멀다 — 컨트롤러가 오래된 TF로 목표를 (0,0)에 변환한 경합.
                    # 같은 목표를 다시 보낸다. (S3처럼 위치를 착각한 채 '도착'한 것은 믿는 위치가 목표 근처라 해당 없음)
                    resent += 1
                    events.append(dict(t=round(t, 2), kind="resend"))
                    self.wait_fresh_tf(need=1.0, limit=10.0)
                    # 컨트롤러 쪽 TF도 신선해질 때까지: 'Transform data too old' 경고가 2초 동안 없을 때 다시 보낸다
                    w = time.monotonic()
                    while time.monotonic() - w < 20.0:
                        rclpy.spin_once(nav, timeout_sec=0.1)
                        if not self.too_old or time.monotonic() - self.too_old[-1] > 2.0:
                            break
                    nav.goToPose(pose_stamped(nav, *legs[leg]))
                    t_sent = self.now() - t0
                    continue
                if nav.getResult() == TaskResult.SUCCEEDED and leg + 1 < len(legs) and phase == "go":
                    # S8: 계단 옆 지점 도착 → 앞(북쪽)에 사람들이 나타남 → 길을 비켜 주려고 뒤로 물러난다 (등 뒤가 계단)
                    events.append(dict(t=round(t, 2), kind="reached", n=leg + 1))
                    for k, (px, py) in enumerate(G.YIELD_PEOPLE):
                        req = SpawnEntity.Request()
                        req.name = f"yield{k}"
                        req.xml = PERSON_SDF.format(name=req.name, x=px, y=py, r=G.PERSON_R)
                        req.reference_frame = "world"
                        self.call(self.spawn, req)
                        blockers.append(req.name)
                    nav.backup(backup_dist=float(back_dist), backup_speed=G.YIELD_SPEED,
                               time_allowance=int(back_dist / G.YIELD_SPEED) + 10)
                    events.append(dict(t=round(t, 2), kind="backup", dist=round(back_dist, 3)))
                    phase = "backup"
                    continue
                if nav.getResult() == TaskResult.FAILED and retried < 4 and t < self.timeout - 10:
                    # Nav2가 복구 끝에 포기 → 실제 배송 로봇처럼 관제 쪽에서 같은 목표를 다시 보낸다
                    retried += 1
                    events.append(dict(t=round(t, 2), kind="retry", n=retried))
                    nav.goToPose(pose_stamped(nav, *legs[leg]))
                    t_sent = self.now() - t0
                    continue
                break
            if t > self.timeout:
                nav.cancelTask()
                outcome = "timeout"
                break
        res = nav.getResult()
        if outcome is None:
            outcome = {TaskResult.SUCCEEDED: "arrived", TaskResult.FAILED: "failed",
                       TaskResult.CANCELED: "canceled"}.get(res, "unknown")
        gx, gy = (self.gt[0], self.gt[1]) if self.gt else (None, None)
        result = dict(scenario=self.scen, condition=self.cond, seed=self.seed, outcome=outcome, t0_sim=round(t0, 2),
                      time=round(fell_at if fell_at is not None else self.now() - t0, 2),
                      recoveries=recov, min_edge=round(min_edge, 3), pushed=pushed,
                      push_dx=round(push_dx, 3) if pushed else None, people=self.people, events=events,
                      final_gt=[round(gx, 3), round(gy, 3)] if gx is not None else None,
                      goal_err=round(math.hypot(gx - legs[-1][0], gy - legs[-1][1]), 3) if gx is not None else None,
                      frames=fi, frame_t=frame_t, resent=resent, retried=retried,
                      push_param=self.push_fixed, mat_dropout=self.mat_dropout,
                      void_dropout=self.void_dropout, floor_mu=self.floor_mu,
                      walkers=[dict(name=w.name, appear=w.appear_n, yield_n=w.block_n) for w in self.walkers],
                      goals=[[round(v, 3) for v in q] for q in legs], leg_reached=leg + (outcome == "arrived"),
                      back_dist=round(back_dist, 3) if back_dist else None, pitch_k=self.pitch_k)
        with open(os.path.join(self.out, "result.json"), "w") as f:
            json.dump(result, f, ensure_ascii=False, indent=1)
        with open(os.path.join(self.out, "track.json"), "w") as f:
            json.dump(log, f, separators=(",", ":"))
        nav.get_logger().info(f"결과: {outcome}  t={result['time']}s  가장자리 최소 {min_edge:.2f} m  복구 {recov}회")


def main():
    rclpy.init()
    r = Runner()
    try:
        r.run()
    finally:
        r.nav.destroy_node()
        rclpy.try_shutdown()
