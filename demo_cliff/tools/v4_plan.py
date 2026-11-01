"""판정 기준 v4 확인 실험 계획 — 실행(run_v4.sh)과 분석(analyze_v4.py)이 이 파일 하나를 읽는다.

    python3 tools/v4_plan.py order      # 실행 순서 (탭 구분: 묶음 시나리오 방식 시드 최고속도 계단결측 종료x IR중심)

시드는 앞 실험(1–20, 101–120, 201–220)에서 쓰지 않은 301–320을 쓴다. F만 301–305.
실행 순서: 시드 하나를 블록으로 두고, 블록 안에서 그 시드를 쓰는 모든 (묶음, 시나리오, 방식)을 시드로 섞는다.
최고 속도 0은 nav2.yaml 그대로(0.45 m/s)다. 월드는 1뿐이다.
"""
import random
import sys

SEEDS = list(range(301, 321))
F_SEEDS = list(range(301, 306))
IR_R0, IR_R1 = 0.24, 0.30       # 앞 IR 중심 거리: 지금 값, B에서 옮겨 보는 값
CAM_KILL_X = 5.0                # D: 참값 x가 이 값 이상이 되면 cliff_detector 종료 (계단 A는 x ≈ 5.2부터 보이고 S3 밀림은 x = 5.8)
F_SPEEDS = (0.3, 0.45, 0.6, 0.8)

# 묶음: 이름 → 시나리오, 방식, 최고 속도, 계단 결측(VOID), 카메라 종료 x, IR 중심 거리, 시드
# 같은 방식 이름으로 설정만 다른 칸은 묶음을 나눈다 (결과 폴더가 묶음별이다)
GROUPS = {
    # A의 P2i · P3i는 계단 앞에서 스스로 느려져(사전 시험 8번) 속도 조건을 같은 시드의 카메라 없는 칸(B24의 P1ir)으로 본다
    "A": dict(scens=["S3", "S4"], conds=["P2i", "P3i"], vmax=0.8, speed_ref="B24"),
    "B24": dict(scens=["S3", "S4"], conds=["P1ir"], vmax=0.8, ir_r=IR_R0),
    "B30": dict(scens=["S3", "S4"], conds=["P1ir"], vmax=0.8, ir_r=IR_R1),
    "C": dict(scens=["S3", "S4"], conds=["P1ir", "P3i", "P3p"], vmax=0.8, void=1.0),
    "D": dict(scens=["S3", "S4"], conds=["P3i", "P3p"], vmax=0.8, camkill=CAM_KILL_X),
    "E45": dict(scens=["S5"], conds=["P1ir", "P3i"]),
    "E80": dict(scens=["S5"], conds=["P1ir", "P3i"], vmax=0.8),
}
for v in F_SPEEDS:
    GROUPS[f"F{round(v * 100)}"] = dict(scens=["S9"], conds=["P1ir", "P3i"], vmax=v, seeds=F_SEEDS, ir_r=IR_R0)
    GROUPS[f"F{round(v * 100)}r"] = dict(scens=["S9"], conds=["P1ir"], vmax=v, seeds=F_SEEDS, ir_r=IR_R1)

# 비교: (이름, 앞 묶음, 뒤 묶음, 시나리오들, 앞 방식, 뒤 방식, 지표). 지표 safety = 안전 실패, contact = 접촉 회차 (둘 다 낮을수록 좋음)
# 주 가설은 Holm 보정, 부 가설은 보정 없이 p와 신뢰구간만
PRIMARY = [
    ("A", "A", "A", ["S3", "S4"], "P3i", "P2i", "safety"),
    ("B", "B30", "B24", ["S3", "S4"], "P1ir", "P1ir", "safety"),
    ("C", "C", "C", ["S3", "S4"], "P3p", "P3i", "safety"),
    ("D", "D", "D", ["S3", "S4"], "P3p", "P3i", "safety"),
]
SECONDARY = [
    ("E 0.45 m/s", "E45", "E45", ["S5"], "P3i", "P1ir", "contact"),
    ("E 0.8 m/s", "E80", "E80", ["S5"], "P3i", "P1ir", "contact"),
    ("E P1ir 속도", "E80", "E45", ["S5"], "P1ir", "P1ir", "contact"),
    ("E P3i 속도", "E80", "E45", ["S5"], "P3i", "P3i", "contact"),
]


def runs():
    out = []
    for g, c in GROUPS.items():
        for s in c["scens"]:
            for p in c["conds"]:
                for k in c.get("seeds", SEEDS):
                    out.append((g, s, p, k, c.get("vmax", 0), c.get("void", 0.0), c.get("camkill", -1), c.get("ir_r", IR_R0)))
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
