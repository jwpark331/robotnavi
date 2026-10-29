# 보이지 않는 계단 — Gazebo 데모

> 새 컴퓨터에서 시작하는 방법, 실험 스크립트 목록, 다른 컴퓨터 결과 다루는 규칙은 저장소 맨 위 [README](../README.md)에 있다. 아래는 10/12 시점의 데모 설명이다.

2024년 6월 구미시의회 행정 로봇이 계단 아래에서 멈춘 채 발견된 사건을 출발점으로, 하나의 가설을 ROS 2 + Gazebo로 재현한다.

> 수평 2D LiDAR로 만든 SLAM 지도에는 **내려가는 계단이 빈 바닥으로 찍힌다.**
> 평소에는 경로가 계단을 비켜 가서 문제가 안 드러나지만, 사람을 피해 돌거나 위치를 착각하면 Nav2가 계단 위로 간다.

그리고 세 가지 방어가 각각 어느 경우를 막는지 본다: 운영자가 그린 **keepout**, 깊이카메라 **단일 프레임** 판정, 깊이카메라 **베이즈 필터**(제안).
사건의 실제 원인은 공식 확인되지 않았다. 이 데모는 원인 규명이 아니라 가설 하나의 재현이다.

## 실행

```bash
bash install_sim.sh          # 한 번: Gazebo Classic 11, Nav2, slam_toolbox (sudo 암호 필요)
cd ws && colcon build --symlink-install && cd ..
SEEDS="1 2 3 4 5" ./run_all.sh   # 본 실험: S1–S4 × 6조건 × 시드 5 = 120회차 (약 4–5시간)
./run_sweep.sh                   # 견고성 sweep: 밀림 거리 0–1.2 m, 매트 결측률 0–90 % (58회차, 약 2시간)
                                 # 끝나면 results/report.html
```

- 일부만: `SCENS="S3" CONDS="P1 P3" ./run_all.sh`
- Gazebo 창으로 보기: `GUI=true SCENS=S2 CONDS=P0 ./run_all.sh`
- 시드 여러 개(사람 위치·밀림 거리가 조금씩 바뀜): `SEEDS="1 2 3 4 5" ./run_all.sh`
- 리포트만 다시: `python3 tools/make_report.py results/runs results/report.html` → `python3 -m http.server 8767 --directory results`
- 지도 다시 만들기: `source env.sh && ros2 launch cliff_demo mapping.launch.py map_path:=$PWD/ws/src/cliff_demo/maps/city_hall`
  → `cd ws/src/cliff_demo && python3 -m cliff_demo.make_masks maps` → 다시 빌드

> EVL 실시간 커널(`6.12.76-cip18+`)에서는 무거운 시뮬 중 hard lockup으로 재부팅된 적이 있다. `uname -r`이 `6.8.0-*-generic`인지 먼저 확인한다. `run_all.sh`는 EVL이면 멈춘다.

## 월드

시청 2층을 단순화했다. 복도(서→동)가 모퉁이에서 북쪽 가지 복도로 꺾이고, 가지 복도 끝 서쪽에 사무실(목표)이 있다.

- **계단 A** — 모퉁이에 붙어 있다. 복도 쪽과 가지 복도 입구 쪽 두 면이 열려 있다.
- **계단 B** — 가지 복도 끝, 사무실 문 바로 위.
- **검은 매트** — 복도 바닥. 적외선 반사가 약해 깊이 결측이 난다고 가정한다.
- **창가 햇빛** — 가지 복도 바닥. 깊이 이상치가 난다고 가정한다.

기하는 모두 `ws/src/cliff_demo/cliff_demo/geometry.py` 한 곳에 있다. SDF 월드, keepout 마스크, 추락 판정이 모두 그 값을 쓴다.

## 시나리오 × 조건

| 시나리오 | 내용 |
| --- | --- |
| S1 평상시 | 사람 없음 |
| S2 계단 앞 사람들 | 모퉁이 안쪽(가지 복도 입구)에 4명. 45초 뒤 흩어진다 |
| S3 위치 착각 | 복도 중간에서 로봇을 앞으로 0.8–1.0 m 옮긴다. 바퀴가 안 돌았으니 인코더 odom은 모른다(사람이 밀거나 미끄러진 경우) |
| S4 keepout 안 그린 계단 | 사무실 문 앞 모퉁이 안쪽에 3명. 운영자가 계단 B keepout을 빠뜨렸다 |

| 조건 | costmap filter |
| --- | --- |
| P0 SLAM 지도만 | 없음 |
| P1 + Keepout (현장 관행) | `KeepoutFilter` ← 운영자가 그린 마스크 (계단 + 30 cm) |
| P1i + 하향 IR 4개 (급정지) | P1 + 룸바식 하향 거리센서 4개 → `ir_guard`가 전진만 막음 (반론 "IR이면 되지 않나" 비교군) |
| P2 + 깊이카메라 단일 프레임 | P1 + 두 번째 `KeepoutFilter` ← `cliff_detector mode=frame` |
| P3m + 베이즈 (기억을 map 프레임에) | P1 + 두 번째 `KeepoutFilter` ← `cliff_detector mode=bayes frame=map` |
| P3 + 베이즈 (기억을 odom 프레임에, 제안) | P1 + 두 번째 `KeepoutFilter` ← `cliff_detector mode=bayes frame=odom` |

Nav2는 Humble 기본 파라미터 그대로다(AMCL, NavFn, DWB, 회전·후진 복구 행동 트리). 로봇 크기·속도만 맞췄다 — `params/nav2.yaml` 머리말 참고.
Nav2가 복구 끝에 포기하면 시나리오 실행기가 관제 시스템처럼 같은 목표를 다시 보낸다(최대 4번, 150초 안에서).

## 낭떠러지 층 (`cliff_detector.py`)

1. 깊이 픽셀마다 광선을 바닥면(z = 0)까지 따라가서, 그 바닥 셀에 표를 준다.
   - 바닥보다 8 cm 넘게 아래에서 반사 → 바닥 없음
   - 바닥 높이에서 반사 → 바닥
   - 반사 없음 → 결측
2. 한 셀에 픽셀이 여럿이므로 비율로 한 표를 정한다.
3. 판정
   - **베이즈**: 셀마다 log-odds. 바닥 없음 +0.9, 바닥 −0.7, 결측 +0.35이되 1.2까지만. 결측만으로는 낭떠러지 확정(L > 2.0)이 되지 않는다. 격자에 남으므로 시야에서 벗어나도 기억한다.
     - 기억을 **map 프레임**에 두면, 위치를 착각한 채 본 가장자리가 그 틀린 자리에 박제된다. AMCL이 나중에 위치를 바로잡으면 그 기억이 실제로는 바닥인 곳에 '유령 낭떠러지'로 남는다(S3에서 가지 복도 입구를 120초 막음).
     - 기억을 **odom 프레임**에 두면, AMCL이 map→odom을 고칠 때 기억도 로봇과 같이 옮겨진다. Nav2 지역 코스트맵이 odom 프레임인 것과 같은 이유다.
   - **단일 프레임**: 이번 프레임에서 바닥 없음·결측 셀이 이웃에 2개 이상 뭉치면 낭떠러지(룸바식 보수적 판정). 기억이 없다.
4. 낭떠러지 셀을 30 cm 부풀려 `/cliff_filter_mask`(OccupancyGrid, map 프레임)로 5 Hz 발행한다. Nav2의 두 번째 KeepoutFilter가 이 마스크를 받아 지역·전역 코스트맵에 금지 구역으로 넣는다. Humble의 KeepoutFilter는 마스크가 갱신되면 바꿔 끼우고, 마스크 프레임과 코스트맵 프레임이 달라도 변환한다.

Gazebo는 검은 바닥의 적외선 흡수나 햇빛 간섭을 렌더링하지 못한다. 그래서 그 구역에 들어간 픽셀만 참값 위치로 골라 결측 75 %, 이상치 15 %로 바꾼다. 판단 자체에는 참값을 쓰지 않는다.

## 강의 연결

| 강의 | 이 데모에서 |
| --- | --- |
| L2 LiDAR SLAM | slam_toolbox 매핑 — 계단이 빈 바닥으로 찍히는 이유 |
| L3 센서 (LiDAR 한계, 노이즈, 외부 센서 단점) | 2D LiDAR는 바닥 아래를 못 본다. 깊이카메라 결측·이상치 |
| L5·L6 좌표변환 | 깊이 픽셀 → camera_optical → base_footprint → map. keepout이 map 프레임이라 위치를 착각하면 실제 세상에서 어긋난다 |
| L7 차동구동 | 로봇 모델, 매핑 주행(RTR) |
| L9 AEB 이진 분류 | 낭떠러지 판정의 confusion matrix. 놓침(FN) = 추락이므로 재현율을 먼저 본다 |
| (확장) 베이즈 필터 | 셀별 log-odds 누적 — 강의에는 개념(L3 sensor fusion)만 있다 |

## 파일

```
install_sim.sh   run_all.sh   env.sh
tools/            make_report.py, report_template.html, kill_sim.sh, probe.py
ws/src/cliff_demo/
  cliff_demo/     geometry.py  make_world.py  make_masks.py  map_builder.py  cliff_detector.py  scenario.py
  launch/         sim.launch.py  mapping.launch.py  demo.launch.py
  urdf/           servi.urdf.xacro   (차동구동 서빙로봇형, 앞쪽 270° LiDAR, 하향 깊이카메라, 뒤쪽 센서 없음)
  worlds/         city_hall.world    (make_world.py가 생성)
  maps/           city_hall.*        (slam_toolbox 결과), keepout_AB.*, keepout_A.*
  params/         nav2.yaml
results/
  runs/<S>_<P>_<seed>/  result.json  track.json  cliff.jsonl  video.mp4  launch.log  nav2_params.yaml
  report.html
```

## 한계

- 사람은 서 있는 원기둥이다.
- 깊이 결측·이상치의 위치와 비율은 가정이다.
- 시드당 1회로는 경향만 볼 수 있다. 비율로 말하려면 시드를 늘려야 한다.
- S3의 '밀림'은 위험한 방향(계단 쪽)으로 넣은 스트레스 테스트다.

## 추가 실험 (주제 1 확정 후, 2026-10-12)

| 실험 | 무엇을 막으려고 | 설정 |
| --- | --- | --- |
| **P1i 하향 IR 비교군** | "IR 낭떠러지 센서(룸바식)면 되지 않나" | 몸체 앞 가장자리 4곳(±20°, ±60°, 반지름 0.24 m), 바닥 위 2.5 cm, 7 cm 넘거나 반사 없으면 2회 연속 판정 후 전진 차단. 검은 매트 위 빔은 결측 주입(룸바 검은 카펫 문제) |
| **시드 5개** | "1회씩이라 우연 아닌가" | 시드가 사람 위치(±15 cm), 밀림 거리(0.8–1.0 m), 깊이·IR 잡음을 바꾼다 |
| **밀림 거리 sweep** | "0.85 m 밀림은 과장 아닌가" | S3 × {P1, P1i, P3} × 0.0–1.2 m(0.2 간격) × 시드 2 → keepout이 버티는 위치 오차 한계 |
| **결측률 sweep** | "결함 주입 값을 맞춘 것 아닌가" | S1 × {P2, P3} × 매트 결측 0 / 50 / 75 / 90 % × 시드 2 → 결측이 늘어도 베이즈가 버티나 |
| **위치 오차 측정 수정** | "오차 측정이 부풀려진 것 아닌가" | `/amcl_pose`(0.25 m마다 갱신) 대신 같은 시각의 TF `map→base_footprint`로 잰다 |

- 이전 20회차(시드 1, IR 비교군 없음, 오차 측정 수정 전)는 `results/runs_v1/`에 보관했다.
- 회차가 많아져 리포트는 요약만 HTML에 넣고, 궤적·낭떠러지 기록은 회차 폴더의 `replay.json`을 필요할 때 읽는다. 그래서 `file://`로 열면 리플레이가 안 되고, `python3 -m http.server 8767 --directory results`로 띄워서 본다.
