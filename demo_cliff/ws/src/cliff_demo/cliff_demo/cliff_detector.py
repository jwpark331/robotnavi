"""깊이카메라 → 낭떠러지 층 → Nav2 KeepoutFilter 마스크.

픽셀마다 광선을 바닥면(z = 0)과 만나는 점까지 따라가서 그 바닥 셀에 표를 하나 준다.
  - 바닥보다 8 cm 넘게 아래에서 반사 → '바닥 없음'(drop) — 광선이 바닥 높이를 그냥 지나갔다
  - 바닥 높이에서 반사 → '바닥'
  - 반사 없음(결측) → '모름에 가까운 의심'(nan) — 검은 매트도, 깊은 낭떠러지도 이렇게 보인다
  - 바닥보다 위 → 장애물 (LiDAR 몫이라 무시)
한 셀에 픽셀이 여럿 들어오므로 셀마다 비율로 한 표를 정한다.

mode=bayes (제안): 셀마다 이진 베이즈 필터(log-odds). drop +0.9, 바닥 −0.7, nan은 +0.35이되 1.2(주의)까지만 —
    결측만으로는 절대 '낭떠러지 확정'(L > 2.0)이 되지 않는다. 격자에 남으므로 시야에서 벗어나도 기억한다.
    frame=odom(제안)이면 기억을 odom 프레임에 둔다 — AMCL이 위치를 고쳐도(map→odom이 바뀌어도) 기억이 로봇과 같이 옮겨진다.
    frame=map이면 위치를 착각한 채 본 가장자리가 그 틀린 자리에 박제된다 (비교용).
mode=frame (비교): 이번 프레임만 본다. drop이나 nan 셀이 이웃에 2개 이상 뭉치면 낭떠러지 (룸바식 보수적 판정). 기억 없음.

Gazebo는 검은 바닥의 적외선 흡수나 햇빛 간섭을 렌더링하지 못한다. artifacts=true면 참값 위치를 써서
DARK_MAT 구역 픽셀은 MAT_DROPOUT(기본 75 %) 결측, SUN_PATCH 구역 픽셀은 SUN_OUTLIER(15 %) 이상치로 바꾼다 (센서 결함 주입 — 판단에는 참값을 안 쓴다).
void_dropout > 0이면 계단 구멍 안으로 들어간 광선도 그 확률로 결측이 된다 ('검은 계단' 실험: 어두운 계단·깊은 낭떠러지).
"""
import json
import math
import time

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float32
from tf2_ros import Buffer, TransformListener

from cliff_demo import geometry as G

H_DROP = -0.08
H_OBST = 0.08
NEAR, FAR = 0.30, 3.0          # 이 거리 안의 바닥만 쓴다 (몸체·먼 바닥 제외)
L_DROP, L_FLOOR, L_NAN, L_NAN_CAP = 0.9, -0.7, 0.35, 1.2
L_LETHAL, L_CAUTION, L_MIN, L_MAX = 2.0, 0.7, -2.5, 3.5


def vmax(d, t_r, a):
    """거리 d 안에 멈출 수 있는 최대 속도 (반응 지연 t_r, 감속 a): v = a(−t_r + √(t_r² + 2d/a))."""
    if d <= 0:
        return 0.0
    return a * (-t_r + math.sqrt(t_r * t_r + 2.0 * d / a))


def quat_to_R(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))


class CliffDetector(Node):
    def __init__(self):
        super().__init__("cliff_detector")
        self.mode = self.declare_parameter("mode", "bayes").value
        self.artifacts = self.declare_parameter("artifacts", True).value
        self.res = self.declare_parameter("resolution", 0.1).value
        self.inflate = self.declare_parameter("inflate", G.KEEPOUT_MARGIN).value
        self.log_path = self.declare_parameter("log_path", "").value
        self.decim = self.declare_parameter("decimation", 2).value
        self.frame = self.declare_parameter("frame", "odom").value
        self.ox, self.oy, self.nx, self.ny = G.CLIFF_GRID[self.frame]
        self.L = np.zeros((self.ny, self.nx), np.float32)
        self.lethal = np.zeros((self.ny, self.nx), bool)
        self.caution = np.zeros((self.ny, self.nx), bool)
        # 마스크 모양: 낭떠러지 셀에서 inflate(0.30 m) 안은 금지(100), 그 밖 ring(0.70 m)까지는 90 → 15로 줄어드는 비용.
        # Nav2 costmap filter는 인플레이션 층을 거치지 않으므로, 경로가 금지 구역 경계에 딱 붙지 않게 마스크에 직접 넣는다.
        # (KeepoutFilter는 0–100을 비용 0–254로 선형 변환하고 기존 비용보다 클 때만 덮어쓴다 → 중간 값 = 비싸지만 지나갈 수 있음)
        self.ring = self.declare_parameter("ring", 0.70).value
        r = int(math.ceil(self.ring / self.res))
        self.offsets = []
        for i in range(-r, r + 1):
            for j in range(-r, r + 1):
                dist = math.hypot(i, j) * self.res
                if dist <= self.inflate + 1e-9:
                    self.offsets.append((i, j, 100))
                elif dist <= self.ring + 1e-9:
                    f = (dist - self.inflate) / (self.ring - self.inflate)
                    self.offsets.append((i, j, int(round(90 - 75 * f))))
        # P3h: 주의 칸(+ inflate 0.3 m)을 caution_cost(0–100)로 마스크에 넣는다. 0이면 끈다 (본래 P3는 주의 칸을 안 보낸다)
        self.caution_cost = int(self.declare_parameter("caution_cost", 0).value)
        rc = int(math.ceil(self.inflate / self.res))
        self.caution_offsets = [(i, j) for i in range(-rc, rc + 1) for j in range(-rc, rc + 1)
                                if math.hypot(i, j) * self.res <= self.inflate + 1e-9]
        self.rng = np.random.default_rng(self.declare_parameter("seed", 1).value + 7)
        self.mat_dropout = self.declare_parameter("mat_dropout", G.MAT_DROPOUT).value
        self.sun_outlier = self.declare_parameter("sun_outlier", G.SUN_OUTLIER).value
        self.void_dropout = self.declare_parameter("void_dropout", G.VOID_DROPOUT).value
        # ---- 피드백 1 대응 (10/21). 모두 기본값이면 기존 동작과 같다 ----
        # memory: bayes는 keep(P3) / none(P3n: 이번에 안 본 칸은 잊는다), frame은 none(P2) / keep(P2o: 칸마다 마지막 판정을 기억)
        self.memory = self.declare_parameter("memory", "keep" if self.mode == "bayes" else "none").value
        self.nan_neutral = self.declare_parameter("nan_neutral", False).value     # 값 없음은 낭떠러지 증거도 바닥 증거도 아니다
        self.view_gate = self.declare_parameter("view_gate", False).value         # 5 cm 또는 3° 움직였을 때만 증거를 센다
        self.ground_plane = self.declare_parameter("ground_plane", False).value   # 매 프레임 바닥 평면 RANSAC → 높이를 평면 기준으로
        self.sigma_phi = math.radians(self.declare_parameter("sigma_phi_deg", 0.0).value)   # 거리별 임계 τ(d) = max(8 cm, 3·d·σφ)
        self.three_state = self.declare_parameter("three_state", False).value     # CLIFF / FLOOR / UNKNOWN + 정지거리 속도 제한
        self.ir_memory = self.declare_parameter("ir_memory", False).value         # IR이 본 낭떠러지를 기억에 CLIFF로 고정
        self.pitch_k = math.radians(self.declare_parameter("pitch_k_deg", 0.0).value)       # 실험용: 가감속 연동 기울기 오차 [rad/(m/s²)]
        # 속도 제한 상수 (Nav2 설정값): 감속 a, IR 반응 지연 t_IR, IR이 볼 때 남는 거리 d_IR(IR 0.24 m − 걸침 한계 0.15 m)
        self.a_dec = self.declare_parameter("a_dec", 2.0).value
        self.t_ir = self.declare_parameter("t_ir", 0.15).value
        self.d_ir = self.declare_parameter("d_ir", 0.09).value
        self.t_lat = self.declare_parameter("t_lat", 0.20).value
        self.v_nav = self.declare_parameter("v_nav", 0.45).value
        self.v_ir = vmax(self.d_ir, self.t_ir, self.a_dec)
        self.seen_floor = np.zeros((self.ny, self.nx), bool)
        self.ir_conf = np.zeros((self.ny, self.nx), bool)
        self.mem = np.zeros((self.ny, self.nx), bool)
        self.last_upd = None
        self.acc, self.v_prev, self.t_prev = 0.0, None, None
        self.rng_plane = np.random.default_rng(self.get_parameter("seed").value + 11)
        self.plane_ok = self.plane_n = 0
        self.plane_corr = 0.0
        self.plane_s = np.zeros(3)          # 시간 평활한 평면 (a, b, c). 맞추지 못하면 기본 평면(0)으로 서서히 돌아간다
        self.cap = self.v_nav
        self.cap_low = 0
        # IR 기억 칸의 마스크 모양: 그 칸만 금지(100), 0.4 m까지 60 → 15 비용. IR은 발밑 한 점이라 위치가 cm 단위로 정확해서
        # 카메라 칸처럼 0.3 m 금지 띠를 둘 필요가 없다. (띠를 두면 좁은 복도에서 계획기는 띠 옆으로 길을 내고 컨트롤러는 몸 전체가
        # 띠 밖이어야 움직여서 갇혔다 — 10/21 시험, 검은 매트 옆 복도)
        ri = int(math.ceil(0.4 / self.res))
        self.ir_offsets = [(i, j, 100 if (i, j) == (0, 0) else int(round(60 - 45 * (math.hypot(i, j) * self.res - self.res) / (0.4 - self.res))))
                           for i in range(-ri, ri + 1) for j in range(-ri, ri + 1) if math.hypot(i, j) * self.res <= 0.4 + 1e-9]
        rf = int(math.ceil(G.ROBOT_R / self.res))
        self.foot_offsets = [(i, j) for i in range(-rf, rf + 1) for j in range(-rf, rf + 1) if math.hypot(i, j) * self.res <= G.ROBOT_R]
        self.K = None
        self.R_bc = self.t_bc = None
        self.gt = None
        self.conf = dict(TP=0, FP=0, FN=0, TN=0)
        self.n_frames = 0
        self.last_pub = 0.0
        self.last_log = 0.0
        self.log = open(self.log_path, "a") if self.log_path else None
        if self.log:
            self.log.write(json.dumps(dict(meta=True, frame=self.frame, ox=self.ox, oy=self.oy, res=self.res,
                                           mode=self.mode)) + "\n")

        self.tf = Buffer(cache_time=Duration(seconds=10))
        self.tfl = TransformListener(self.tf, self)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(OccupancyGrid, "/cliff_filter_mask", qos)
        self.create_subscription(CameraInfo, "/depth_cam/depth_cam/depth/camera_info", self.on_info, 1)
        self.create_subscription(Image, "/depth_cam/depth_cam/depth/image_raw", self.on_depth, 2)
        self.create_subscription(Odometry, "/ground_truth", self.on_gt, 10)
        if self.pitch_k:
            self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        if self.ir_memory:
            self.create_subscription(PointStamped, "/ir_cliff", self.on_ir_cliff, 10)
        self.cap_pub = self.create_publisher(Float32, "/cliff_speed_cap", 10) if self.three_state else None
        self.publish_mask(force=True)
        self.get_logger().info(f"cliff_detector mode={self.mode} artifacts={self.artifacts}")

    # ---------- 입력 ----------
    def on_info(self, m):
        if self.K is None:
            self.K = np.array(m.k, float).reshape(3, 3)

    def on_gt(self, m):
        p = m.pose.pose
        self.gt = (p.position.x, p.position.y, yaw_of(p.orientation))

    def on_odom(self, m):
        """가감속 추정 (기울기 오차 주입용): 바퀴 odom 속도의 변화율, 0.15 s 지수 평활."""
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        v = m.twist.twist.linear.x
        if self.t_prev is not None and t > self.t_prev:
            a = (v - self.v_prev) / (t - self.t_prev)
            k = min(1.0, (t - self.t_prev) / 0.15)
            self.acc += k * (a - self.acc)
        self.v_prev, self.t_prev = v, t

    def on_ir_cliff(self, m):
        """IR이 바닥 없음을 본 지점 → 그 한 칸을 CLIFF로 고정. 카메라 관측으로 지워지지 않는다.
        (처음엔 3×3칸으로 넓혀 기록했더니 검은 매트 헛감지 몇 번에 복도 절반이 막혔다 — 10/21 피드백 원안대로 한 칸)"""
        try:
            tf = self.tf.lookup_transform(self.frame, m.header.frame_id, Time())
        except Exception:
            return
        R = quat_to_R(tf.transform.rotation)[:2, :2]
        p = R @ np.array([m.point.x, m.point.y]) + np.array([tf.transform.translation.x, tf.transform.translation.y])
        ix = int(math.floor((p[0] - self.ox) / self.res))
        iy = int(math.floor((p[1] - self.oy) / self.res))
        if 0 <= ix < self.nx and 0 <= iy < self.ny:
            self.ir_conf[iy, ix] = True
            self.L[self.ir_conf] = L_MAX

    def fit_plane(self, Q):
        """0.3–1.2 m 앞 점들에 바닥 평면 z = a x + b y + c 를 RANSAC으로 맞춘다. 기울기 5° 안, inlier 60 % 이상이어야 채택."""
        n = len(Q)
        if n < 150 or np.ptp(Q[:, 0]) < 0.5 or np.ptp(Q[:, 1]) < 0.3:   # 벽 앞 등 바닥이 좁게만 보이면 맞추지 않는다
            return None
        best, best_cnt = None, 0
        for _ in range(40):
            p0, p1, p2 = Q[self.rng_plane.choice(n, 3, replace=False)]
            nv = np.cross(p1 - p0, p2 - p0)
            nn = np.linalg.norm(nv)
            if nn < 1e-9:
                continue
            nv = nv / nn
            if nv[2] < 0:
                nv = -nv
            if nv[2] < math.cos(math.radians(5.0)):
                continue
            inl = np.abs((Q - p0) @ nv) < 0.015
            cnt = int(inl.sum())
            if cnt > best_cnt:
                best, best_cnt = inl, cnt
        if best is None or best_cnt < 0.6 * n:
            return None
        A = np.c_[Q[best, 0], Q[best, 1], np.ones(best_cnt)]
        coef, *_ = np.linalg.lstsq(A, Q[best, 2], rcond=None)
        return coef

    def static_cam(self):
        if self.R_bc is not None:
            return True
        try:
            t = self.tf.lookup_transform("base_footprint", "camera_optical", Time())
        except Exception:
            return False
        self.R_bc = quat_to_R(t.transform.rotation)
        tr = t.transform.translation
        self.t_bc = np.array([tr.x, tr.y, tr.z])
        return True

    # ---------- 한 프레임 ----------
    def on_depth(self, msg):
        if self.mode == "off" or self.K is None or not self.static_cam():
            return
        try:
            tm = self.tf.lookup_transform(self.frame, "base_footprint", Time.from_msg(msg.header.stamp),
                                          timeout=Duration(seconds=0.05))
        except Exception:
            try:
                tm = self.tf.lookup_transform(self.frame, "base_footprint", Time())
            except Exception:
                return
        d = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
        s = self.decim
        d = d[::s, ::s].astype(np.float64)
        vv, uu = np.mgrid[0:msg.height:s, 0:msg.width:s]
        fx, fy, cx, cy = self.K[0, 0], self.K[1, 1], self.K[0, 2], self.K[1, 2]
        ray = np.stack([(uu - cx) / fx, (vv - cy) / fy, np.ones_like(d)], -1).reshape(-1, 3)
        D = ray @ self.R_bc.T                     # 몸체 좌표의 광선 방향 (광학 깊이 1당) — 참 방향
        D_true = D
        t = self.t_bc
        d = d.ravel()
        down = D[:, 2] < -1e-3
        s0 = np.where(down, -t[2] / np.where(down, D[:, 2], -1), np.inf)   # 바닥면과 만나는 깊이
        C = t[:2] + s0[:, None] * D[:, :2]                                   # 그 바닥 점 (몸체 좌표)
        rC = np.hypot(C[:, 0], C[:, 1])
        valid = np.isfinite(d) & (d > 0.15) & (d < 3.99)

        if self.artifacts and self.gt is not None:
            gx, gy, gth = self.gt
            c, sn = math.cos(gth), math.sin(gth)
            wx = gx + c * C[:, 0] - sn * C[:, 1]
            wy = gy + sn * C[:, 0] + c * C[:, 1]
            reach = down & (~valid | (d >= s0 - 0.05))           # 광선이 그 바닥 점까지 실제로 간 픽셀
            m0 = G.DARK_MAT
            on_mat = reach & (wx >= m0[0]) & (wx <= m0[2]) & (wy >= m0[1]) & (wy <= m0[3])
            sp = G.SUN_PATCH
            on_sun = reach & (wx >= sp[0]) & (wx <= sp[2]) & (wy >= sp[1]) & (wy <= sp[3])
            drop_px = on_mat & (self.rng.random(d.size) < self.mat_dropout)
            out_px = on_sun & (self.rng.random(d.size) < self.sun_outlier)
            void_px = None
            if self.void_dropout > 0:                            # 0이면 난수를 더 뽑지 않는다 → 본 실험과 같은 결과
                in_void = np.zeros(d.size, bool)
                for x0, y0, x1, y1, _ in G.VOIDS.values():
                    in_void |= (wx >= x0) & (wx <= x1) & (wy >= y0) & (wy <= y1)
                void_px = reach & in_void & (self.rng.random(d.size) < self.void_dropout)
            d = d.copy()
            d[drop_px] = np.nan
            d[out_px] = self.rng.uniform(0.3, 3.9, out_px.sum())
            if void_px is not None:
                d[void_px] = np.nan
            valid = np.isfinite(d) & (d > 0.15) & (d < 3.99)

        if self.pitch_k:
            # 실험: 가속하면 차체가 들리는데 소프트웨어는 고정 외부 파라미터를 믿는다 → 광선이 δ만큼 더 아래를 향한다고 착각
            dl = max(-math.radians(3.0), min(math.radians(3.0), self.pitch_k * self.acc))
            cd, sd = math.cos(dl), math.sin(dl)
            D = D_true @ np.array([[cd, 0.0, sd], [0.0, 1.0, 0.0], [-sd, 0.0, cd]]).T
            down = D[:, 2] < -1e-3
            s0 = np.where(down, -t[2] / np.where(down, D[:, 2], -1), np.inf)
            C = t[:2] + s0[:, None] * D[:, :2]
            rC = np.hypot(C[:, 0], C[:, 1])
        P = t + np.where(valid, d, 0.0)[:, None] * D
        h = P[:, 2]
        if self.ground_plane:
            rho = np.hypot(P[:, 0], P[:, 1])
            q = valid & down & (rho > 0.3) & (rho < 1.2) & (np.abs(h) < 0.15)
            coef = self.fit_plane(P[q])
            self.plane_n += 1
            lim = np.array([math.tan(math.radians(3.0)), math.tan(math.radians(3.0)), 0.03])
            target = np.clip(coef, -lim, lim) if coef is not None else np.zeros(3)
            self.plane_ok += coef is not None
            self.plane_s += 0.3 * (target - self.plane_s)                 # 약 0.3 s 평활 (가감속 기울기는 0.5–1 s 이어진다)
            a_, b_, c_ = self.plane_s
            self.plane_corr = math.degrees(math.atan(a_))
            # 0.3°·5 mm 안이면 평평한 바닥으로 보고 보정하지 않는다 (먼 거리 외삽 잡음으로 표가 가장자리 앞으로 당겨지는 것을 막는다)
            if abs(a_) > math.tan(math.radians(0.3)) or abs(b_) > math.tan(math.radians(0.3)) or abs(c_) > 0.005:
                h = P[:, 2] - (a_ * P[:, 0] + b_ * P[:, 1] + c_)            # 추정 평면 기준 높이
                den = D[:, 2] - a_ * D[:, 0] - b_ * D[:, 1]
                dn = down & (den < -1e-3)
                s0 = np.where(dn, (a_ * t[0] + b_ * t[1] + c_ - t[2]) / np.where(dn, den, -1), np.inf)
                C = t[:2] + s0[:, None] * D[:, :2]
                rC = np.hypot(C[:, 0], C[:, 1])
        use = down & (rC > NEAR) & (rC < FAR)
        tau = np.maximum(-H_DROP, 3.0 * rC * self.sigma_phi) if self.sigma_phi else -H_DROP
        k_drop = use & valid & (h < -tau)
        k_floor = use & valid & (np.abs(h) <= H_OBST) & (np.hypot(P[:, 0], P[:, 1]) > NEAR)
        k_nan = use & ~valid
        xy = np.where(k_floor[:, None], P[:, :2], C)                       # 바닥 표는 반사점, 나머지는 바닥면 교점
        sel = k_drop | k_floor | k_nan
        if not sel.any():
            return
        kind = np.where(k_drop, 0, np.where(k_floor, 1, 2))[sel]
        xy = xy[sel]

        # 몸체 → 지도
        Rm = quat_to_R(tm.transform.rotation)[:2, :2]
        tmv = tm.transform.translation
        mxy = xy @ Rm.T + np.array([tmv.x, tmv.y])
        ix = np.floor((mxy[:, 0] - self.ox) / self.res).astype(int)
        iy = np.floor((mxy[:, 1] - self.oy) / self.res).astype(int)
        ok = (ix >= 0) & (ix < self.nx) & (iy >= 0) & (iy < self.ny)
        ix, iy, kind, xy = ix[ok], iy[ok], kind[ok], xy[ok]
        flat = iy * self.nx + ix
        n = self.nx * self.ny
        cnt = np.stack([np.bincount(flat[kind == k], minlength=n) for k in range(3)])   # drop, floor, nan
        tot = cnt.sum(0)
        seen = tot > 0
        fd = np.where(seen, cnt[0] / np.maximum(tot, 1), 0)
        ff = np.where(seen, cnt[1] / np.maximum(tot, 1), 0)
        fn = np.where(seen, cnt[2] / np.maximum(tot, 1), 0)
        v_drop = seen & (fd > 0.3)
        v_floor = seen & ~v_drop & (ff > 0.7)
        v_nan = seen & ~v_drop & ~v_floor & (fn > 0.5)
        v_drop, v_floor, v_nan = (a.reshape(self.ny, self.nx) for a in (v_drop, v_floor, v_nan))

        seen2 = seen.reshape(self.ny, self.nx)
        rx, ry = tmv.x, tmv.y
        ryaw = math.atan2(Rm[1, 0], Rm[0, 0])
        gate = True
        if self.view_gate and self.last_upd is not None:
            lx, ly, lyaw = self.last_upd
            gate = math.hypot(rx - lx, ry - ly) >= 0.05 or abs(math.atan2(math.sin(ryaw - lyaw), math.cos(ryaw - lyaw))) >= math.radians(3.0)
        if self.mode == "bayes":
            if gate:
                self.last_upd = (rx, ry, ryaw)
                L = self.L + L_DROP * v_drop + L_FLOOR * v_floor
                if not self.nan_neutral:
                    L = np.where(v_nan, np.minimum(L + L_NAN, np.maximum(L, L_NAN_CAP)), L)
                self.L = np.clip(L, L_MIN, L_MAX).astype(np.float32)
                self.seen_floor |= v_floor
                if self.memory == "none":                        # P3n: 이번에 안 본 칸은 잊는다
                    self.L[~seen2] = 0.0
            self.L[self.ir_conf] = L_MAX
            self.lethal = (self.L > L_LETHAL) | self.ir_conf
            self.caution = (self.L > L_CAUTION) & ~self.lethal
        else:
            bad = (v_drop | v_nan).astype(np.int32)
            nb = sum(np.roll(np.roll(bad, i, 0), j, 1) for i in (-1, 0, 1) for j in (-1, 0, 1))
            now = bad.astype(bool) & (nb >= 2)
            if self.memory == "keep":                            # P2o: 칸마다 마지막 판정을 기억 (본 칸만 새로 덮어씀)
                self.mem = np.where(seen2, now, self.mem)
                now = self.mem
            self.lethal = now | self.ir_conf
            self.caution = np.zeros_like(self.lethal)
            self.seen_floor |= v_floor
        if self.three_state:
            self.speed_cap(rx, ry, ryaw)

        # 평가 (참값): 이번에 본 셀이 정말 계단 구멍인가 vs 지금 낭떠러지로 판정했나
        if self.gt is not None:
            gx, gy, gth = self.gt
            c, sn = math.cos(gth), math.sin(gth)
            uniq, first = np.unique(iy * self.nx + ix, return_index=True)
            bx, by = xy[first, 0], xy[first, 1]
            wx, wy = gx + c * bx - sn * by, gy + sn * bx + c * by
            truth = np.zeros(len(uniq), bool)
            for x0, y0, x1, y1, _ in G.VOIDS.values():
                truth |= (wx > x0 + 0.05) & (wx < x1 - 0.05) & (wy > y0 + 0.05) & (wy < y1 - 0.05)
            edge = np.zeros(len(uniq), bool)                     # 가장자리 ±5 cm 셀은 채점에서 뺀다
            for x0, y0, x1, y1, _ in G.VOIDS.values():
                inside_pad = (wx > x0 - 0.05) & (wx < x1 + 0.05) & (wy > y0 - 0.05) & (wy < y1 + 0.05)
                edge |= inside_pad & ~((wx > x0 + 0.05) & (wx < x1 - 0.05) & (wy > y0 + 0.05) & (wy < y1 - 0.05))
            pred = self.lethal.ravel()[uniq]
            m = ~edge
            self.conf["TP"] += int((pred & truth & m).sum())
            self.conf["FP"] += int((pred & ~truth & m).sum())
            self.conf["FN"] += int((~pred & truth & m).sum())
            self.conf["TN"] += int((~pred & ~truth & m).sum())
        self.n_frames += 1
        self.publish_mask()

    def speed_cap(self, rx, ry, ryaw):
        """CLIFF / FLOOR / UNKNOWN으로 나누고, 로봇 몸이 지나갈 앞쪽 통로(폭 ±0.25 m = 반지름, 3 m)에서 처음 만나는
        CLIFF·UNKNOWN 칸까지 거리로 속도 상한. (처음엔 ±0.3 m로 잡아 경로 옆 낭떠러지 칸까지 '앞'으로 보고 멈췄다 — 10/21 수정)
        v ≤ min( v_max(d_cliff − 0.3; t_lat),  sqrt(v_IR² + 2 a d_unk) ),  d_unk는 IR(앞 0.24 m)이 그 칸에 닿기까지 남은 거리."""
        iy0, ix0 = int((ry - self.oy) / self.res), int((rx - self.ox) / self.res)
        for i, j in self.foot_offsets:                           # 로봇이 서 있는 칸은 바닥이다
            if 0 <= iy0 + i < self.ny and 0 <= ix0 + j < self.nx:
                self.seen_floor[iy0 + i, ix0 + j] = True
        floor = self.seen_floor & (self.L <= 0.0) & ~self.lethal
        s = np.arange(G.ROBOT_R, 3.0, 0.05)
        lat = np.array([-G.ROBOT_R, -G.ROBOT_R / 2, 0.0, G.ROBOT_R / 2, G.ROBOT_R])
        c, sn = math.cos(ryaw), math.sin(ryaw)
        X = rx + s[:, None] * c - lat[None, :] * sn
        Y = ry + s[:, None] * sn + lat[None, :] * c
        ix = np.floor((X - self.ox) / self.res).astype(int)
        iy = np.floor((Y - self.oy) / self.res).astype(int)
        inside = (ix >= 0) & (ix < self.nx) & (iy >= 0) & (iy < self.ny)
        ixc, iyc = np.clip(ix, 0, self.nx - 1), np.clip(iy, 0, self.ny - 1)
        cliff = inside & self.lethal[iyc, ixc]
        unk = ~inside | (~floor[iyc, ixc] & ~cliff)
        cap = self.v_nav
        rows = np.nonzero(cliff.any(1))[0]
        if len(rows):
            cap = min(cap, vmax(s[rows[0]] - 0.30, self.t_lat, self.a_dec))
        rows = np.nonzero(unk.any(1))[0]
        if len(rows):
            d_unk = max(0.0, s[rows[0]] - G.IR_RADIUS)
            cap = min(cap, math.sqrt(self.v_ir ** 2 + 2 * self.a_dec * d_unk))
        self.cap = cap
        self.cap_low += cap < 0.40                               # 의미 있게 느려진 프레임 (0.37 m/s 근처까지)
        self.cap_pub.publish(Float32(data=float(cap)))

    # ---------- 출력 ----------
    def publish_mask(self, force=False):
        now = time.monotonic()
        if not force and now - self.last_pub < 0.2:
            return
        self.last_pub = now
        val = np.zeros(self.lethal.shape, np.int8)
        cam = self.lethal & ~self.ir_conf                     # 카메라가 판정한 낭떠러지: 0.3 m 금지 + 0.7 m까지 비용
        if cam.any():
            for i, j, v in self.offsets:
                sh = np.roll(np.roll(cam, i, 0), j, 1)
                np.maximum(val, np.where(sh, v, 0).astype(np.int8), out=val)
        if self.ir_conf.any():                                # IR이 확인한 낭떠러지: 그 칸 금지 + 0.4 m까지 비용
            for i, j, v in self.ir_offsets:
                sh = np.roll(np.roll(self.ir_conf, i, 0), j, 1)
                np.maximum(val, np.where(sh, v, 0).astype(np.int8), out=val)
        if self.caution_cost > 0 and self.caution.any():
            for i, j in self.caution_offsets:
                sh = np.roll(np.roll(self.caution, i, 0), j, 1)
                np.maximum(val, np.where(sh, self.caution_cost, 0).astype(np.int8), out=val)
        m = OccupancyGrid()
        m.header.frame_id = self.frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.info.resolution = float(self.res)
        m.info.width, m.info.height = self.nx, self.ny
        m.info.origin.position.x, m.info.origin.position.y = self.ox, self.oy
        m.info.origin.orientation.w = 1.0
        m.data = val.ravel().tolist()
        self.pub.publish(m)
        if self.log and now - self.last_log > 0.5:
            self.last_log = now
            st = self.get_clock().now().nanoseconds * 1e-9
            ly, lx = np.nonzero(self.lethal)
            cy, cx = np.nonzero(self.caution)
            extra = {}
            if self.three_state:
                extra.update(cap=round(self.cap, 3), cap_low=int(self.cap_low))
            if self.ground_plane:
                extra.update(plane_ok=self.plane_ok, plane_n=self.plane_n, plane_deg=round(self.plane_corr, 2))
            if self.pitch_k:
                extra.update(acc=round(self.acc, 3))
            if self.ir_memory:
                extra.update(ir_cells=int(self.ir_conf.sum()))
            self.log.write(json.dumps(dict(t=round(st, 2), lethal=(lx * 1000 + ly).tolist(),
                                           caution=(cx * 1000 + cy).tolist(), conf=self.conf, n=self.n_frames, **extra)) + "\n")
            self.log.flush()


def main():
    rclpy.init()
    n = CliffDetector()
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if n.log:
            n.log.close()
        n.destroy_node()
        rclpy.try_shutdown()
