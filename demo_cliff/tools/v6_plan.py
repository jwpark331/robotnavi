"""판정 기준 v6 확인 실험 계획 — 실행(run_v6.sh)과 분석(analyze_v6.py)이 이 파일 하나를 읽는다.

    python3 tools/v6_plan.py order      # 실행 순서 (탭 구분: 묶음 시나리오 방식 시드 월드 가장자리거리)
    python3 tools/v6_plan.py            # 묶음별 회차 수

로봇 (기준 v6 2장):
  P3y   ① 기억을 보고 안전한 만큼만 물러나는 로봇 (새 설계)
  P3    ② 기억은 있지만 후진을 Nav2에 맡기는 로봇 (지금 제안)
  P3n   ③ 카메라는 있지만 기억이 없는 로봇
  P1irb ④ 카메라 없이 앞 · 뒤에 하향 IR을 단 로봇
  P2nav ⑤ Nav2 깊이 레이어가 바닥 아래 점을 찍게 설정한 로봇
시드는 앞 실험에서 쓰지 않은 501–520. 실행 순서: 시드 하나를 블록으로, 블록 안에서 (묶음, 시나리오, 방식)을 시드로 섞는다 (v4 · v5와 같다).
월드 2의 검은 매트는 v5와 같은 자리(v3 H12)다.
"""
import random
import sys

SEEDS = list(range(501, 521))
W2_MAT = "3.0,2.0,4.0,3.1"
YIELD_DS = (0.55, 1.10)
ROBOTS_Y = ["P3y", "P3", "P3n", "P1irb", "P2nav"]
YIELD_OK = 0.20         # 비켜 줌: 실제 후진 ≥ 0.20 m
MARGIN_OK, MARGIN_MAX_BAD = 0.25, 2   # 부 기준: ① 로봇의 후진 중 가장자리 거리 0.25 m 아래가 80회 중 2회 이하


def dtag(d):
    return f"{int(round(d * 100)):03d}"


# 묶음: 이름 → 시나리오, 방식, 월드, 가장자리 거리
GROUPS = {}
for w in (1, 2):
    for d in YIELD_DS:
        GROUPS[f"Y-w{w}-d{dtag(d)}"] = dict(scens=["S8"], conds=ROBOTS_Y, world=w, yield_d=d)
GROUPS["N"] = dict(scens=["S1", "S2"], conds=["P3", "P2nav"], world=1, yield_d=0.55)

Y_NEAR = [g for g, c in GROUPS.items() if g.startswith("Y") and c["yield_d"] == 0.55]
Y_ALL = [g for g in GROUPS if g.startswith("Y")]
# 주 가설: (이름, 묶음들, 시나리오들, 앞 방식, 뒤 방식, 지표). Holm 보정 3개.
PRIMARY = [
    ("가설 1", Y_NEAR, ["S8"], "P3y", "P3n", "safety"),
    ("가설 2", Y_ALL, ["S8"], "P3y", "P3", "yield"),
    ("가설 3", ["N"], ["S1", "S2"], "P3", "P2nav", "arrival"),
]


def runs():
    out = []
    for g, c in GROUPS.items():
        for s in c["scens"]:
            for p in c["conds"]:
                for k in SEEDS:
                    out.append((g, s, p, k, c["world"], c["yield_d"]))
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
