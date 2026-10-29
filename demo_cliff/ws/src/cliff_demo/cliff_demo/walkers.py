"""움직이는 사람 (S5·S6): 건물 안 통행 지점 사이를 걷는 원기둥.

- 6명. 시드마다 각 사람이 나타나는 시각·위치, 속도, 목적지, 걷는 구간 수를 무작위로 정한다 (사람마다 따로 난수를 써서,
  로봇과 마주치기 전까지는 조건이 달라도 같은 시드면 같은 움직임).
- 통행 지점(WAYPOINTS) 사이를 직선으로 걷는다. 지점과 구간은 벽에서 0.3 m, 계단 가장자리에서 0.4 m 넘게 떨어져 있다
  (사람은 계단에 들어가지 않는다). 목적지는 모퉁이·사무실 문 같은 붐비는 곳을 더 자주 고른다.
- 목적지에 닿으면 잠깐 멈췄다가(0–3 s) 다음 목적지로 간다. 몇 구간 걸으면 사라졌다가 다른 곳에서 다시 나타난다.
  나타나는 곳은 로봇에서 3 m 넘게 떨어진 곳이다 (눈앞에 갑자기 생기지 않게).
- 로봇 앞을 막게 되면 멈춰 기다린다 (사람이 로봇을 피해 준다). 3 s 넘게 막히면 다른 목적지로 바꾼다.

Gazebo에서는 정적 원기둥 모델을 /set_entity_state로 5 Hz 옮긴다 (LiDAR·깊이카메라에 그대로 보인다).
"""
import math

import numpy as np

from cliff_demo import geometry as G

N_WALKERS = 6
SPEED = (0.8, 1.3)          # 걷는 속도 [m/s]
LEGS = (2, 5)               # 한 번 나타나서 걷는 구간 수 (2–4)
FIRST_APPEAR = (0.0, 10.0)  # 처음 나타나는 시각 [s] (로봇이 모퉁이·문에 닿는 15–25 s에 마주치게)
GONE = (2.0, 8.0)           # 사라졌다가 다시 나타나기까지 [s]
DWELL = (0.0, 5.0)          # 목적지에서 멈춰 서 있는 시간 [s] (대화·두리번)
YIELD_R = 0.75              # 로봇 중심과 이 거리 안으로는 다가가지 않는다 [m]
GAP_R = 0.45                # 다른 사람과 이 거리 안으로는 다가가지 않는다 [m]
BLOCK_T = 3.0               # 이만큼 막히면 목적지를 바꾼다 [s]
NO_DWELL_R = 1.0            # 로봇이 이 거리 안에 있으면 목적지에서 멈춰 서지 않고 바로 지나간다 [m]
UPDATE_DT = 0.2             # Gazebo에 위치를 보내는 간격 [s] (5 Hz — 요청이 많으면 통신이 밀려 TF가 늦는다)
APPEAR_MIN = 3.0            # 로봇에서 이만큼 떨어진 곳에서만 나타난다 [m]
WALL_CLEAR, VOID_CLEAR = 0.30, 0.40

# 통행 지점: 이름 → (x, y, 붐비는 정도 가중치)
WAYPOINTS = {
    "W0": (0.9, 3.6, 1), "W1": (0.9, 4.8, 1),            # 복도 서쪽 끝
    "W2": (2.6, 3.6, 1), "W3": (2.6, 4.8, 1),
    "W4": (4.2, 3.6, 1), "W5": (4.2, 4.8, 1),
    "W6": (5.8, 3.6, 1), "W7": (5.8, 4.8, 1),
    "K0": (7.2, 3.6, 2), "K1": (7.2, 4.8, 3), "K2": (7.7, 4.2, 3),   # 계단 A 앞 모퉁이 (붐빔)
    "B0": (7.3, 6.0, 3), "B1": (8.3, 6.6, 1),            # 가지 복도 (입구 B0 붐빔)
    "B2": (7.2, 7.4, 1), "B3": (8.4, 7.6, 1),
    "B4": (7.4, 8.6, 2), "B5": (8.4, 8.8, 1),            # 사무실 문 앞
    "D0": (6.6, 8.9, 1),                                 # 사무실 문 (지나가기만 하고 서 있지 않는다)
    "O1": (5.6, 8.0, 1), "O2": (4.4, 9.2, 1), "O3": (4.4, 7.9, 1),   # 사무실 안
}


def _pt_seg(p, a, b):
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / L2))
    return math.hypot(p[0] - ax - t * dx, p[1] - ay - t * dy)


def _pt_void(p):
    best = 9e9
    for x0, y0, x1, y1, _ in G.VOIDS.values():
        dx = max(x0 - p[0], 0.0, p[0] - x1)
        dy = max(y0 - p[1], 0.0, p[1] - y1)
        best = min(best, -1.0 if dx == dy == 0 else math.hypot(dx, dy))
    return best


def _clear(a, b):
    n = max(2, int(math.hypot(b[0] - a[0], b[1] - a[1]) / 0.05))
    walls = [(p, q) for line in G.WALLS for p, q in zip(line[:-1], line[1:])]
    for i in range(n + 1):
        p = (a[0] + (b[0] - a[0]) * i / n, a[1] + (b[1] - a[1]) * i / n)
        if _pt_void(p) < VOID_CLEAR:
            return False
        if any(_pt_seg(p, w0, w1) < WALL_CLEAR for w0, w1 in walls):
            return False
    return True


def _graph():
    names = list(WAYPOINTS)
    adj = {k: {} for k in names}
    for i, a in enumerate(names):
        for b in names[i + 1:]:
            pa, pb = WAYPOINTS[a][:2], WAYPOINTS[b][:2]
            if _clear(pa, pb):
                d = math.hypot(pb[0] - pa[0], pb[1] - pa[1])
                adj[a][b] = adj[b][a] = d
    return adj


ADJ = _graph()


def route(a, b):
    """통행 지점 a → b 최단 경로 (지점 이름 목록, a 제외)."""
    dist, prev, todo = {a: 0.0}, {}, {a}
    while todo:
        u = min(todo, key=dist.get)
        todo.remove(u)
        if u == b:
            break
        for v, w in ADJ[u].items():
            if dist[u] + w < dist.get(v, 9e9):
                dist[v], prev[v] = dist[u] + w, u
                todo.add(v)
    if b not in prev and a != b:
        return []
    path = [b]
    while path[-1] != a:
        path.append(prev[path[-1]])
    return path[::-1][1:]


class Walker:
    def __init__(self, k, seed):
        self.name = f"walker{k}"
        self.rng = np.random.default_rng(seed * 100 + k)
        self.park = (-20.0 - 2.0 * k, -20.0)   # 건물 밖 대기 자리 (센서에 안 보임)
        self.pos = self.park
        self.visible = False
        self.t_next = self.rng.uniform(*FIRST_APPEAR)
        self.path, self.legs, self.speed = [], 0, 1.0
        self.goal = self.node = None
        self.dwell_until = None
        self.blocked_since = None
        self.appear_n = 0
        self.block_n = 0

    def _pick_goal(self, avoid):
        names = [n for n in WAYPOINTS if n != avoid]
        w = np.array([WAYPOINTS[n][2] for n in names], float)
        return names[int(self.rng.choice(len(names), p=w / w.sum()))]

    def _appear(self, robot, others):
        names = list(WAYPOINTS)
        order = self.rng.permutation(len(names))
        for i in order:
            n = names[i]
            x, y = WAYPOINTS[n][:2]
            if robot is not None and math.hypot(x - robot[0], y - robot[1]) < APPEAR_MIN:
                continue
            if any(math.hypot(x - o[0], y - o[1]) < 1.0 for o in others):
                continue
            self.node, self.pos, self.visible = n, (x, y), True
            self.speed = self.rng.uniform(*SPEED)
            self.legs = int(self.rng.integers(*LEGS))
            self.goal = self._pick_goal(n)
            self.path = route(n, self.goal)
            self.appear_n += 1
            return True
        return False

    def _vanish(self, t):
        self.visible, self.pos = False, self.park
        self.t_next = t + self.rng.uniform(*GONE)

    def step(self, t, dt, robot, others):
        """t: 시나리오 시각, dt: 지난 갱신 뒤 지난 시간, robot: 실제 (x, y), others: 다른 사람 위치들."""
        if not self.visible:
            if t >= self.t_next and not self._appear(robot, others):
                self.t_next = t + 1.0
            return
        if self.dwell_until is not None:
            if t < self.dwell_until:
                return
            self.dwell_until = None
            self.legs -= 1
            if self.legs <= 0:
                self._vanish(t)
                return
            self.goal = self._pick_goal(self.node)
            self.path = route(self.node, self.goal)
        if not self.path:
            self.dwell_until = t + self.rng.uniform(*DWELL)   # 경로가 없는 드문 경우(목적지 = 현재 지점)
            return
        tx, ty = WAYPOINTS[self.path[0]][:2]
        x, y = self.pos
        d = math.hypot(tx - x, ty - y)
        s = min(d, self.speed * dt)
        nx, ny = (x + (tx - x) / d * s, y + (ty - y) / d * s) if d > 1e-6 else (tx, ty)
        # 로봇·다른 사람과 가까워지는 걸음이면 멈춰 기다린다 (멀어지는 걸음은 허용 → 서로 끼어 멈추지 않게)
        def closer(o, r):
            return math.hypot(nx - o[0], ny - o[1]) < r and math.hypot(nx - o[0], ny - o[1]) < math.hypot(x - o[0], y - o[1])
        if (robot is not None and closer(robot, YIELD_R)) or any(closer(o, GAP_R) for o in others):
            if self.blocked_since is None:
                self.blocked_since = t
                self.block_n += 1
            elif t - self.blocked_since > BLOCK_T:
                self.blocked_since = None
                here = min(WAYPOINTS, key=lambda n: math.hypot(WAYPOINTS[n][0] - x, WAYPOINTS[n][1] - y))
                self.goal = self._pick_goal(self.path[0])
                self.path = [here] + route(here, self.goal) if here != self.path[0] else route(here, self.goal)
            return
        self.blocked_since = None
        self.pos = (nx, ny)
        if s >= d - 1e-6:
            self.node = self.path.pop(0)
            if not self.path:
                wait = self.rng.uniform(*DWELL)       # 난수는 항상 뽑는다 (같은 시드끼리 순서가 어긋나지 않게)
                near = robot is not None and math.hypot(nx - robot[0], ny - robot[1]) < NO_DWELL_R
                self.dwell_until = t + (0.0 if self.node == "D0" or near else wait)


if __name__ == "__main__":
    # 통행 그래프 점검: 연결되지 않은 지점이 없어야 한다
    print({k: sorted(v) for k, v in ADJ.items()})
    lonely = [k for k, v in ADJ.items() if not v]
    print("연결 안 된 지점:", lonely)
    print("W0 → D0:", route("W0", "D0"))
    print("K2 → B5:", route("K2", "B5"))
