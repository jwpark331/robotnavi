# 보정 실행 비교 — calib_JW-laptop

새 컴퓨터:

```
기록 시각: 2026-10-30 00:56:25
호스트: JW-laptop
OS: Ubuntu 22.04.5 LTS
커널: 6.8.0-138-generic
CPU: AMD Ryzen 7 8845HS w/ Radeon 780M Graphics (16코어)
메모리: 30Gi
GPU: VGA compatible controller: NVIDIA Corporation Device 28e0 (rev a1);VGA compatible controller: Advanced Micro Devices, Inc. [AMD/ATI] Device 1900 (rev cc)
NVIDIA: NVIDIA GeForce RTX 4060 Laptop GPU, 580.178.04
Gazebo: version 11.10.2
RMW: rmw_cyclonedds_cpp
ros-humble-gazebo-ros-pkgs: 3.9.0-1jammy.20260915.233200
ros-humble-nav2-bringup: 1.1.20-1jammy.20260916.005516
ros-humble-navigation2: 1.1.20-1jammy.20260916.003813
ros-humble-rmw-cyclonedds-cpp: 1.3.5-1jammy.20260907.205235
ros-humble-slam-toolbox: 2.6.10-1jammy.20260916.003050
```

비교한 회차 18개 (장치 결함으로 뺀 회차 0개)

| 회차 | 원래 노트북 | 새 컴퓨터 | 안전 판정 같음 | 도착 여부 같음 |
|---|---|---|---|---|
| S2_P0_1 | fall 17 s, 가장자리 0.06 m | fall 17 s, 가장자리 0.07 m | ✓ | ✓ |
| S2_P0_2 | fall 17 s, 가장자리 0.06 m | fall 17 s, 가장자리 0.07 m | ✓ | ✓ |
| S2_P0_3 | fall 17 s, 가장자리 0.06 m | fall 17 s, 가장자리 0.06 m | ✓ | ✓ |
| S2_P1_1 | arrived 64 s, 가장자리 0.51 m | arrived 66 s, 가장자리 0.54 m | ✓ | ✓ |
| S2_P1_2 | arrived 71 s, 가장자리 0.43 m | failed 127 s, 가장자리 0.28 m | ✓ | ✗ |
| S2_P1_3 | failed 141 s, 가장자리 0.25 m | failed 129 s, 가장자리 0.24 m | ✓ | ✓ |
| S2_P3_1 | arrived 65 s, 가장자리 0.52 m | arrived 63 s, 가장자리 0.52 m | ✓ | ✓ |
| S2_P3_2 | arrived 64 s, 가장자리 0.40 m | arrived 61 s, 가장자리 0.54 m | ✓ | ✓ |
| S2_P3_3 | arrived 64 s, 가장자리 0.54 m | arrived 67 s, 가장자리 0.55 m | ✓ | ✓ |
| S3_P1_1 | arrived 28 s, 가장자리 0.25 m | fall 39 s, 가장자리 0.06 m | ✗ | ✗ |
| S3_P1_2 | arrived 27 s, 가장자리 0.17 m | arrived 28 s, 가장자리 0.26 m | ✓ | ✓ |
| S3_P1_3 | arrived 53 s, 가장자리 0.14 m | fall 14 s, 가장자리 0.07 m | ✓ | ✗ |
| S3_P3_1 | arrived 27 s, 가장자리 0.37 m | arrived 27 s, 가장자리 0.37 m | ✓ | ✓ |
| S3_P3_2 | arrived 27 s, 가장자리 0.36 m | arrived 26 s, 가장자리 0.39 m | ✓ | ✓ |
| S3_P3_3 | arrived 144 s, 가장자리 0.51 m | arrived 141 s, 가장자리 0.47 m | ✓ | ✓ |
| S3_P3m_1 | failed 136 s, 가장자리 0.46 m | failed 139 s, 가장자리 0.36 m | ✓ | ✓ |
| S3_P3m_2 | failed 141 s, 가장자리 0.51 m | failed 138 s, 가장자리 0.45 m | ✓ | ✓ |
| S3_P3m_3 | failed 140 s, 가장자리 0.40 m | failed 137 s, 가장자리 0.44 m | ✓ | ✓ |

- 안전 판정(추락·걸침 여부)이 같은 회차: **17/18**
- 도착 여부가 같은 회차: **15/18**
- 둘 다 도착한 회차의 시간 차 (새 − 원래) 중앙값: -0.3 s (8쌍)

## 방식별 경향

| 시나리오·방식 | 원래: 안전 실패 / 도착 | 새: 안전 실패 / 도착 | 원래 노트북에서 나온 경향 |
|---|---|---|---|
| S2 P0 | 3/3 / 0/3 | 3/3 / 0/3 | 계단으로 떨어진다 |
| S2 P1 | 0/3 / 2/3 | 0/3 / 1/3 | 대체로 도착 |
| S2 P3 | 0/3 / 3/3 | 0/3 / 3/3 | 안 떨어지고 도착 |
| S3 P1 | 1/3 / 3/3 | 2/3 / 1/3 | 일부 떨어짐 |
| S3 P3 | 0/3 / 3/3 | 0/3 / 3/3 | 안 떨어지고 도착 |
| S3 P3m | 0/3 / 0/3 | 0/3 / 0/3 | 유령 낭떠러지에 갇힌다 (도착 못 함) |

**판정: 같은 경향.** 핵심 대비(지도만 → 떨어짐, 지도 좌표 기억 → 갇힘, 제안 → 안전)가 원래 노트북과 같다. 이 컴퓨터에서 새 실험을 돌려도 된다. 다만 기존 폴더에 섞지 말고 새 폴더에 쌓고, 비교는 같은 컴퓨터 결과끼리 한다.
