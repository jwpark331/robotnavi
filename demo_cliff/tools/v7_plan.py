"""판정 기준 v7 (odom 배율 오차) 확인 실험 계획 — run_v7.sh와 analyze_v7.py가 이 파일 하나를 읽는다.

    python3 tools/v7_plan.py order      # 실행 순서 (탭 구분: 묶음 시나리오 방식 시드 월드 가장자리거리 배율)
    python3 tools/v7_plan.py            # 묶음별 회차 수

① P3y(기억을 보고 안전한 만큼만 물러나는 로봇)만, S8 · 등 뒤 가장자리 0.55 m, 월드 1 · 2.
odom 배율 s(CLIFF_ODOM_SCALE): 바퀴 플러그인의 지름만 s배 → odom 이동 · 회전 = 실제의 s배 (기준 v7 1장).
시드 701–720. 실행 순서: 시드 하나를 블록으로, 블록 안에서 (월드, s)를 시드로 섞는다.
"""
import random
import sys

SEEDS = list(range(701, 721))
SCALES = (0.90, 0.95, 0.98, 1.00, 1.02, 1.05, 1.10)
W2_MAT = "3.0,2.0,4.0,3.1"      # v5 · v6와 같은 자리
YIELD_D = 0.55
ROBOT = "P3y"
HOLD_MAX = 1                    # 칸 판정: 두 월드 합 40회 중 안전 실패 1 이하면 "버팀" (기준 v7 5장)
CASTER = 0.17                   # 부 지표: 뒤 캐스터가 바닥을 잃는 선 (예비 B8)
SCALE_TOL = 0.02                # 성립 조건: 후진 구간 odom/참값 이동 비 중앙값이 s × (같은 월드 s = 1.00 중앙값) ± 0.02
                                # (v6 ① 회차로 시험해 보니 오차 없이도 0.989 · 1.016이었다)


def stag(s):
    return f"{int(round(s * 100)):03d}"


GROUPS = {f"O-w{w}-s{stag(s)}": dict(world=w, scale=s) for w in (1, 2) for s in SCALES}


def runs():
    return [(g, "S8", ROBOT, k, c["world"], YIELD_D, c["scale"]) for g, c in GROUPS.items() for k in SEEDS]


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
