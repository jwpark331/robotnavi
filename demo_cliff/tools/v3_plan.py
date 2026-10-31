"""판정 기준 v3 확인 실험 계획 — 실행(run_v3.sh)과 분석(analyze_v3.py)이 이 파일 하나를 읽는다.

    python3 tools/v3_plan.py order      # 실행 순서 (탭 구분: 묶음 시나리오 방식 시드 μ 월드 최고속도)

시드는 사전 실험(1–20)과 v2(101–120)에서 쓰지 않은 201–220을 쓴다.
실행 순서: 시드 하나를 블록으로 두고, 블록 안에서 그 시드를 쓰는 모든 (묶음, 시나리오, 방식)을 시드로 섞는다.
최고 속도 0은 nav2.yaml 그대로(0.45 m/s)다.
"""
import random
import sys

SEEDS = list(range(201, 221))
W2_MAT = "3.0,2.0,4.0,3.1"    # 월드 2 검은 매트 (H8 탐색 때 옮긴 자리. 복도 남쪽 벽에 붙고 경로를 덮지 않는다)

# 묶음: 이름 → 시나리오, 방식, 젖은 구역 μ, 월드, 최고 속도
GROUPS = {
    "H9a": dict(scens=["S3", "S4"], conds=["P1ir", "P3i"], vmax=0.8),
    "H9r": dict(scens=["S3", "S4"], conds=["P1ir"]),                  # 선택 A: 같은 시드의 0.45 m/s 기준 (P3i는 H11 묶음)
    "H10": dict(scens=["S3", "S4"], conds=["P1r", "P3"]),
    "H11": dict(scens=["S3", "S4", "S8"], conds=["P2i", "P3i"]),
    "H12": dict(scens=["S2", "S4"], conds=["P1ir", "P3i"], mu_by_scen={"S2": 0.02}, world=2),
}

# 비교: (이름, A 묶음, B 묶음, 시나리오들, A, B, 지표). 지표 safety = 안전 실패(낮을수록 좋음), arrival = 도착(높을수록 좋음)
# 주 가설은 Holm 보정, 나머지는 보정 없이 p와 신뢰구간만
PRIMARY = [
    ("H9a", "H9a", "H9a", ["S3", "S4"], "P3i", "P1ir", "safety"),
    ("H10", "H10", "H10", ["S3", "S4"], "P3", "P1r", "safety"),
    ("H11", "H11", "H11", ["S8"], "P3i", "P2i", "safety"),
]
SECONDARY = [
    ("H9r", "H11", "H9r", ["S3", "S4"], "P3i", "P1ir", "safety"),
    ("H11b", "H11", "H11", ["S3", "S4"], "P3i", "P2i", "safety"),
    ("H12-S2", "H12", "H12", ["S2"], "P3i", "P1ir", "safety"),
    ("H12-S4", "H12", "H12", ["S4"], "P3i", "P1ir", "safety"),
]


def runs():
    out = []
    for g, c in GROUPS.items():
        for s in c["scens"]:
            mu = c.get("mu_by_scen", {}).get(s, 1.0)
            for p in c["conds"]:
                for k in SEEDS:
                    out.append((g, s, p, k, mu, c.get("world", 1), c.get("vmax", 0)))
    return out


def order():
    rs = runs()
    out = []
    for k in SEEDS:
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
