"""판정 기준 v5 확인 실험 계획 — 실행(run_v5.sh)과 분석(analyze_v5.py)이 이 파일 하나를 읽는다.

    python3 tools/v5_plan.py order      # 실행 순서 (탭 구분: 묶음 시나리오 방식 시드 월드 최고속도 IR중심 바퀴가감속 지연)
    python3 tools/v5_plan.py            # 묶음별 회차 수

시드는 앞 실험(1–20, 101–120, 201–220, 301–320)에서 쓰지 않은 401–420을 쓴다. 격자 G만 421–425.
실행 순서: 시드 하나를 블록으로 두고, 블록 안에서 그 시드를 쓰는 모든 (묶음, 시나리오, 방식)을 시드로 섞는다 (v4와 같다).
모든 회차가 월드 2다. 상수는 월드 1 값 그대로다 (기준 v5 2장). S9는 월드 2에서 S9w2 좌표를 쓴다 (geometry.py 월드 2 블록).
"""
import random
import sys

SEEDS = list(range(401, 421))
G_SEEDS = list(range(421, 426))
WORLD = 2
W2_MAT = "3.0,2.0,4.0,3.1"      # 월드 2 검은 매트 — v3 H12와 같은 자리 (복도 남쪽 벽에 붙고 경로를 덮지 않는다)
IR_R0, IR_R1 = 0.24, 0.30       # 앞 IR 중심 거리: 지금 배치, 옮긴 배치
V_B = 0.45                      # B · 격자의 최고 속도. nav2.yaml 기본과 같은 값이지만 결과에 남겨 첫 정지 계산(설정 속도의 80 %)에 쓴다

# 감속 단계: 이름 → (목표 실제 감속 [m/s²], 바퀴 플러그인 설정값 CLIFF_WHEEL_ACC).
# 'base'는 v4까지의 설정(3.0)이고 목표를 두지 않는다. 나머지 설정값은 사전 시험의 제동 시험으로 맞춘다 (기준 v5 10장).
# 아래 설정값은 맞추기 전의 첫 어림이다 — 제동 시험 결과로 고친 뒤 고정한다.
DECEL = {"base": (None, 3.0), "d18": (1.8, 1.8), "d12": (1.2, 1.2), "d08": (0.8, 0.8)}
DELAYS = (0.0, 0.1)             # 인지 지연 [s]: 기본, +0.1 s (IR 판정 반영과 카메라 처리에 똑같이)

# 실험 전 예측 (기준 v5 5장): 첫 정지 위치 ≈ s_F − v·ΔL − (v²/2)·(1/a − 1/a_base).
# s_F는 v4 F 0.45 m/s(기본 설정)의 첫 정지 여유 중앙값, a와 a_base는 제동 시험으로 잰 그 감속 단계와 기본 설정의 감속이다.
FIRST_STOP_F = {IR_R0: 0.157, IR_R1: 0.201}
EDGE_FAIL = 0.15

# 묶음: 이름 → 시나리오, 방식, 그 밖 설정. 같은 방식 이름으로 설정만 다른 칸은 묶음을 나눈다 (결과 폴더가 묶음별이다)
GROUPS = {
    "WS3": dict(scens=["S3", "S3j"], conds=["P3", "P3m"]),
    "WS3r": dict(scens=["S3"], conds=["P1r"]),                                   # 참고: keepout만 있으면 S3가 얼마나 위험한가
    "WS8": dict(scens=["S8"], conds=["P3", "P3n", "P3i", "P2i", "P1r"]),         # P1r은 성립 확인 (막지 않으면 위험한 배치인가)
    "B1": dict(scens=["S9"], conds=["P3i", "P1ir"], vmax=V_B, ir_r=IR_R0, decel="base"),
    "B2": dict(scens=["S9"], conds=["P3i"], vmax=V_B, ir_r=IR_R0, decel="d08"),
    "B2r": dict(scens=["S9"], conds=["P1ir"], vmax=V_B, ir_r=IR_R1, decel="d08"),
}
for dk in DECEL:
    for delay in DELAYS:
        tag = f"G-{dk}-{'L' if delay else '0'}"
        GROUPS[tag] = dict(scens=["S9"], conds=["P3i", "P1ir"], vmax=V_B, ir_r=IR_R0, decel=dk, delay=delay, seeds=G_SEEDS)
        GROUPS[tag + "r"] = dict(scens=["S9"], conds=["P1ir"], vmax=V_B, ir_r=IR_R1, decel=dk, delay=delay, seeds=G_SEEDS)

# 주 가설: (이름, 앞 묶음, 뒤 묶음, 시나리오들, 앞 방식, 뒤 방식, 지표). Holm 보정 5개.
PRIMARY = [
    ("W1", "WS3", "WS3", ["S3", "S3j"], "P3", "P3m", "arrival"),
    ("W2", "WS8", "WS8", ["S8"], "P3", "P3n", "safety"),
    ("W3", "WS8", "WS8", ["S8"], "P3i", "P2i", "safety"),
    ("B1", "B1", "B1", ["S9"], "P3i", "P1ir", "safety"),
    ("B2", "B2", "B2r", ["S9"], "P3i", "P1ir", "safety"),
]
NO_ARRIVAL_RULE = {"B1", "B2"}  # 목표가 구멍 안이라 도착 조건을 보지 않는다 (v4 F와 같다)


def cfg(g):
    """묶음의 실행 설정: (최고 속도, IR 중심, 바퀴 가감속 설정값, 지연)."""
    c = GROUPS[g]
    return c.get("vmax", 0), c.get("ir_r", IR_R0), DECEL[c.get("decel", "base")][1], c.get("delay", 0.0)


def runs():
    out = []
    for g, c in GROUPS.items():
        vmax, ir_r, acc, delay = cfg(g)
        for s in c["scens"]:
            for p in c["conds"]:
                for k in c.get("seeds", SEEDS):
                    out.append((g, s, p, k, WORLD, vmax, ir_r, acc, delay))
    return out


def order():
    rs = runs()
    out = []
    for k in SEEDS + [k for k in G_SEEDS if k not in SEEDS]:
        block = [r for r in rs if r[3] == k]
        random.Random(k).shuffle(block)
        out += block
    return out


if __name__ == "__main__":
    if sys.argv[1:] == ["order"]:
        for r in order():
            print("\t".join(map(str, r)))
    else:
        rs = runs()
        print(f"총 {len(rs)}회")
        for g in GROUPS:
            print(g, sum(r[0] == g for r in rs))
