"""월드 기하 한 곳 정의. SDF 생성·keepout 마스크·평가(추락 판정)가 모두 이 값을 쓴다.

좌표는 map = Gazebo world (매핑 시작 위치를 world 원점 기준으로 맞춘다). 위층 바닥 윗면은 z = 0.
"""
import math

# 벽 (꺾은선). 위층 바닥 높이에서 LiDAR가 보는 것은 이것뿐이다. 계단 구멍은 벽이 아니라 바닥이 없는 곳이라 안 보인다.
WALLS = [
    # 복도 남쪽 벽(문 홈 포함) → 계단 A 수직갱 남·동·북 벽
    [(0.5, 5.4), (0.5, 3.0), (4.6, 3.0), (4.6, 2.6), (5.6, 2.6), (5.6, 3.0), (11.0, 3.0), (11.0, 5.4), (9.0, 5.4)],
    # 가지 복도 동쪽 벽 → 계단 B 수직갱 북·서 벽 (사무실 문 위쪽까지)
    [(9.0, 5.4), (9.0, 11.2), (6.6, 11.2), (6.6, 9.4)],
    # 가지 복도 서쪽 벽(사무실 문 아래쪽) → 복도 북쪽 벽(문 홈 포함)
    [(6.6, 8.4), (6.6, 5.4), (3.4, 5.4), (3.4, 5.8), (2.4, 5.8), (2.4, 5.4), (0.5, 5.4)],
    # 사무실
    [(6.6, 7.2), (3.6, 7.2), (3.6, 10.0), (6.6, 10.0)],
]
WALL_T = 0.10          # 벽 두께
WALL_Z = (-2.0, 0.6)   # 벽은 아래층 바닥부터 (계단 수직갱 벽이 되도록). 위로는 0.6 m만 — 기록 카메라에서 바닥을 가리지 않게

# 계단 구멍 (x0, y0, x1, y1, 내려가는 방향). 벽이 없는 변이 가장자리.
VOIDS = {
    "A": (8.2, 3.0, 11.0, 5.4, "+x"),   # 복도가 가지 복도로 꺾이는 모퉁이. 서쪽(복도)·북쪽(가지 복도 입구 동쪽 0.8 m)이 열려 있다
    "B": (6.6, 9.5, 9.0, 11.2, "+y"),   # 가지 복도 끝. 사무실 문(y 8.4–9.4) 바로 위
}
RISER, TREAD = 0.18, 0.25
LOWER_Z = -2.0

FLOOR_RECT = (0.3, 2.4, 11.2, 11.4)
SLAB_T = 0.2


def slabs(cut_wet=False):
    """위층 바닥 판: FLOOR_RECT에서 계단 구멍을 뺀 직사각형들 (윗면 z = 0). cut_wet이면 젖은 구간도 빼 둔다 (따로 깐다)."""
    holes = list(VOIDS.values()) + ([WET_PATCH] if cut_wet else [])
    xs = sorted({FLOOR_RECT[0], FLOOR_RECT[2]} | {v[i] for v in holes for i in (0, 2)})
    out = []
    for xa, xb in zip(xs[:-1], xs[1:]):
        cuts = sorted((v[1], v[3]) for v in holes if v[0] <= xa and xb <= v[2])
        y = FLOOR_RECT[1]
        for ya, yb in cuts + [(FLOOR_RECT[3], FLOOR_RECT[3])]:
            if ya > y:
                out.append((xa, y, xb, ya))
            y = max(y, yb)
    return out

# 깊이카메라가 못 보는 바닥 (Gazebo는 적외선 흡수·햇빛을 렌더링하지 않으므로 cliff_detector가 이 구역 픽셀에 결측·이상치를 주입)
DARK_MAT = (3.6, 3.0, 4.6, 4.3)    # 검은 고무 매트: 반사가 약해 깊이 결측 (월드에서는 보이기만 하고 충돌은 없다)
MAT_DROPOUT = 0.75                 # 매트 위 픽셀·IR 빔이 반사를 못 받을 확률 (sweep으로 바꿔 본다)
SUN_OUTLIER = 0.15                 # 햇빛 구역 깊이 픽셀이 엉뚱한 값을 낼 확률
SUN_PATCH = (6.6, 6.2, 9.0, 7.4)   # 창가 햇빛: 깊이 이상치
VOID_DROPOUT = 0.0                 # '검은 계단' 실험: 계단 구멍 안으로 들어간 깊이 광선이 반사를 못 받을 확률 (본 실험은 0)
# '젖은 바닥' 실험: 계단 A 바로 앞 모퉁이(로봇이 감속·회전하고, S2에서 사람을 기다리며 제자리 회전하는 곳)를 물청소한 구역.
# FLOOR_MU < 1이면 이 구역만 마찰계수를 낮춘 월드를 회차 폴더에 만들어 쓴다 (본 실험은 1.0 = Gazebo 기본 마찰, 기존 월드 그대로).
WET_PATCH = (6.0, 3.0, 8.2, 6.2)
FLOOR_MU = 1.0

START = (1.3, 4.2, 0.0)
GOAL = (5.3, 8.9, math.pi)         # 사무실 안 (문은 x = 6.6, y 8.4–9.4, 폭 1.0 m)

# 위에서 내려다보는 기록용 카메라 (리포트 영상). 바닥(z=0)에서 픽셀 ↔ 좌표가 선형.
OVERHEAD = dict(x=5.75, y=6.9, z=24.0, hfov=2 * math.atan(5.75 / 24.0), w=720, h=600)   # 영상 저장 크기와 같게 (통신량 ↓)

# 서 있는 사람 무리 (모퉁이 안쪽에 서서 대화 중). 시드마다 ±JITTER 흔든다.
GROUPS = {
    "A": [(6.95, 4.75), (7.45, 5.05), (7.05, 5.6), (7.6, 5.75)],
    "B": [(7.0, 8.15), (7.5, 8.45), (8.05, 8.6), (7.25, 7.6)],   # 문 남쪽과 남동쪽을 막는다 → 북쪽(계단 B 쪽)으로 돌아 들어와야 한다
}
GROUP_JITTER = 0.15     # 시드마다 사람 위치를 ±15 cm 흔든다
GROUP_LEAVE_T = 45.0    # 목표 전송 후 이 시각에 흩어진다 (기다리면 지나갈 수 있게)
PERSON_R = 0.2

# 위치 착각(S3): 로봇이 이 x를 지나면 바퀴가 모르는 이동을 넣는다 (사람이 밀거나 바퀴가 미끄러짐).
PUSH_TRIGGER_X = 5.8
PUSH_DX = (0.8, 1.0)

ROBOT_R = 0.25

# 하향 IR 낭떠러지 센서 (룸바식): 몸체 앞쪽 가장자리 4곳, 바닥 위 2.5 cm에서 수직 아래를 본다.
IR_ANGLES = (-60, -20, 20, 60)     # 진행 방향 기준 [deg]
IR_RADIUS = 0.24                   # 중심에서 [m]
IR_CLIFF = 0.07                    # 이보다 멀면(또는 반사 없음) 낭떠러지
KEEPOUT_MARGIN = 0.30   # 운영자가 그리는 keepout = 계단 구멍 + 30 cm (Nav2 필터는 인플레이션을 안 받으므로)

SCENARIOS = {
    "S1": dict(label="평상시", groups=[], push=False, keepout=("A", "B")),
    "S2": dict(label="계단 앞 사람들", groups=["A"], push=False, keepout=("A", "B")),
    "S3": dict(label="위치 착각", groups=[], push=True, keepout=("A", "B")),
    "S4": dict(label="keepout 안 그린 계단", groups=["B"], push=False, keepout=("A",)),
    # 움직이는 사람 (10/19 추가): 서 있는 무리 대신 5명이 무작위 위치·시각에 나타나 걸어 다닌다 (walkers.py)
    "S5": dict(label="움직이는 사람들", groups=[], push=False, keepout=("A", "B"), walkers=True),
    "S6": dict(label="움직이는 사람들 + 계단 B keepout 누락", groups=[], push=False, keepout=("A",), walkers=True),
    # 기억 시나리오 (10/20 추가, 계단 A keepout 누락 → 카메라 기억만이 계단을 막을 수 있다)
    "S7": dict(label="계단 옆 목적지 (A 누락)", groups=[], push=False, keepout=("B",), goal="STAIRSIDE", goal_jitter=0.05),
    "S8": dict(label="계단 옆에서 뒤로 비켜 주기 (A 누락)", groups=[], push=False, keepout=("B",), goal="STAIRSIDE",
               goal_jitter=0.05, yield_back=True),
}
# S7·S8의 '계단 옆 배달 지점': 계단 A 북쪽 가장자리(y 5.4) 너머, 가장자리에서 0.55 m. 북쪽을 보고 서서 계단을 등진다.
# 가는 길에 모퉁이를 돌며 계단이 시야 밖으로 나가고(S7), 도착 뒤엔 계단이 등 뒤에 있다(S8).
STAIRSIDE = (8.6, 5.95, math.pi / 2)
# S8: 도착하면 앞(북쪽)에 사람들이 나타나 길을 비켜 달라고 한다 → 로봇이 뒤로 물러난다 (서빙로봇의 '비켜 주기').
# 물러나기는 Nav2 BackUp 동작이라 로컬 비용지도로 충돌을 검사한다 → 등 뒤 계단을 기억해야 멈춘다.
YIELD_PEOPLE = [(7.9, 7.0), (8.4, 7.1), (8.8, 6.9)]
YIELD_BACK = (0.6, 0.9)      # 물러나는 거리 [m] (시드마다)
YIELD_SPEED = 0.10           # 물러나는 속도 [m/s]
YIELD_REST = 6.0             # 물러난 뒤 사람들이 지나갈 때까지 기다리는 시간 [s] → 그다음 사무실로
CONDITIONS = {
    "P0": dict(label="SLAM 지도만", keepout=False, cliff=None),
    "P1": dict(label="+ Keepout (현장 관행)", keepout=True, cliff=None),
    "P1i": dict(label="+ 하향 IR 센서 4개 (급정지)", keepout=True, cliff=None, ir=True),
    "P2": dict(label="+ 깊이카메라 단일 프레임", keepout=True, cliff="frame", frame="odom"),
    "P3m": dict(label="+ 베이즈 (기억을 map 프레임에)", keepout=True, cliff="bayes", frame="map"),
    "P3": dict(label="+ 베이즈 (기억을 odom 프레임에, 제안)", keepout=True, cliff="bayes", frame="odom"),
    # 검은 계단 실험에서 추가: 깊이카메라가 결측으로만 보는 계단을 하향 IR이 보완하는가
    "P3i": dict(label="+ 베이즈 (odom) + 하향 IR", keepout=True, cliff="bayes", frame="odom", ir=True),
    # 개선안 (10/20 추가): '값 없음'이 쌓인 주의 칸(점수 0.7–2.0)을 '비싸지만 지나갈 수 있는 길'(60/100)로 Nav2에 보낸다
    "P3h": dict(label="+ 베이즈 (odom) + 결측 주의 비용", keepout=True, cliff="bayes", frame="odom", caution=60),
    # 피드백 1 대응 (10/21)
    # 2×2 요인 설계: 필터와 기억을 따로 바꾼다
    "P2o": dict(label="+ 깊이카메라 단일 프레임 + odom 기억", keepout=True, cliff="frame", frame="odom", memory="keep"),
    "P3n": dict(label="+ 베이즈, 기억 없음", keepout=True, cliff="bayes", frame="odom", memory="none"),
    # P3+: 값 없음 = 모름(3상태) + 정지거리 속도 제한 + IR 기억·후진 + 지면 평면 RANSAC + 거리별 임계 + 시점 변화 게이트
    "P3p": dict(label="P3+ (3상태·속도 제한·IR 기억·지면 평면)", keepout=True, cliff="bayes", frame="odom", ir=True, plus=True),
}
# 낭떠러지 격자 범위: map 프레임은 world와 같고, odom 프레임은 출발 위치(1.3, 4.2, 0°)가 원점
CLIFF_GRID = {"map": (0.0, 1.5, 120, 110), "odom": (-2.0, -3.0, 130, 120)}


def in_rect(x, y, r, pad=0.0):
    return r[0] - pad <= x <= r[2] + pad and r[1] - pad <= y <= r[3] + pad


def void_depth(x, y):
    """로봇 중심이 계단 구멍 안으로 들어간 깊이 [m] (밖이면 음수 = 가장자리까지 거리)."""
    best = -1e9
    for x0, y0, x1, y1, _ in VOIDS.values():
        dx = max(x0 - x, 0.0, x - x1)
        dy = max(y0 - y, 0.0, y - y1)
        if dx == 0.0 and dy == 0.0:
            d = min(x - x0, x1 - x, y - y0, y1 - y)
        else:
            d = -math.hypot(dx, dy)
        best = max(best, d)
    return best
