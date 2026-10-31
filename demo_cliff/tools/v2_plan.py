"""판정 기준 v2 확인 실험 계획 — 실행(run_v2.sh)과 분석(analyze_v2.py)이 이 파일 하나를 읽는다.

    python3 tools/v2_plan.py order      # 실행 순서 (탭 구분: 묶음 시나리오 방식 시드 μ 결측률 월드)

시드는 사전 실험(1–20)에서 본 적 없는 101–120을 쓴다. 칸당 20회 묶음은 101–120, 10회 묶음은 101–110.
실행 순서: 시드 하나를 블록으로 두고, 블록 안에서 그 시드를 쓰는 모든 (묶음, 시나리오, 방식)을 시드로 섞는다.
"""
import random
import sys

S20 = list(range(101, 121))
S10 = list(range(101, 111))

# 묶음: 이름 → 시나리오, 방식, 시드, 젖은 구역 μ, 계단 결측률, 월드
GROUPS = {
    "H1": dict(scens=["S2"], conds=["P1ir", "P3i", "P3"], seeds=S20, mu=0.02),
    "H2": dict(scens=["S3", "S3j"], conds=["P1ir", "P3m", "P3", "P3i"], seeds=S20),
    "H3": dict(scens=["S4", "S6"], conds=["P1ir", "P3i"], seeds=S20),
    "H4": dict(scens=["S8"], conds=["P3n", "P3", "P3i", "P1ir", "P1irb"], seeds=S20),
    "H5": dict(scens=["S1", "S2"], conds=["P2", "P2on", "P3n", "P3"], seeds=S10),
    "H6": dict(scens=["S3", "S4"], conds=["P1ir", "P3", "P3i"], seeds=S10, void=1.0),
    "H7": dict(scens=["S1", "S5"], conds=["P1ir", "P3i"], seeds=S20),
    "H8": dict(scens=["S2", "S4"], conds=["P1ir", "P3i"], seeds=S20, mu_by_scen={"S2": 0.02}, world=2),
    "demo": dict(scens=["S2"], conds=["P0"], seeds=S10),
}

# 비교: (이름, 묶음, 시나리오들, A, B, 지표). 지표 safety = 안전 실패(낮을수록 좋음), arrival = 도착(높을수록 좋음)
# 주 가설은 Holm 보정, 나머지는 보정 없이 p와 신뢰구간만
PRIMARY = [
    ("H1", "H1", ["S2"], "P3i", "P1ir", "safety"),
    ("H2a", "H2", ["S3", "S3j"], "P3", "P3m", "arrival"),
    ("H2b", "H2", ["S3", "S3j"], "P3i", "P1ir", "safety"),
    ("H3", "H3", ["S4", "S6"], "P3i", "P1ir", "safety"),
    ("H4", "H4", ["S8"], "P3", "P3n", "safety"),
]
SECONDARY = [
    ("H4-ref", "H4", ["S8"], "P3i", "P1irb", "safety"),
    ("H5", "H5", ["S1", "S2"], "P3", "P2on", "arrival"),
    ("H5-ref", "H5", ["S1", "S2"], "P2", "P2on", "arrival"),
    ("H6a", "H6", ["S3", "S4"], "P3i", "P3", "safety"),
    ("H6b", "H6", ["S3", "S4"], "P3i", "P1ir", "safety"),
    ("H7", "H7", ["S1", "S5"], "P3i", "P1ir", "arrival"),
    ("H8-S2", "H8", ["S2"], "P3i", "P1ir", "safety"),
    ("H8-S4", "H8", ["S4"], "P3i", "P1ir", "safety"),
]


def runs():
    out = []
    for g, c in GROUPS.items():
        for s in c["scens"]:
            mu = c.get("mu_by_scen", {}).get(s, c.get("mu", 1.0))
            for p in c["conds"]:
                for k in c["seeds"]:
                    out.append((g, s, p, k, mu, c.get("void", 0.0), c.get("world", 1)))
    return out


def order():
    rs = runs()
    out = []
    for k in S20:
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
