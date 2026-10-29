# 기억 시나리오(H1)와 개선안 P3h(H2) 결과

판정 기준은 실험 전에 고정했다 (`보이지 않는 계단 — 판정 기준 (2026-10-20).md`). **주 지표: 안전 실패율(추락 + 걸침)**, 부 지표: 도착률.

## H1. 기억이 필요한 상황 (계단 A keepout 누락)

- **S7 계단 옆 목적지:** 계단 A 모서리 너머(가장자리에서 약 0.55 m)로 간다. 모서리를 돌 때 계단이 시야 밖으로 나간다.
- **S8 계단 옆에서 비켜 주기:** S7 지점에 도착하면 앞에 사람들이 나타나고, 로봇이 0.6–0.9 m 뒤로 물러난다(Nav2 BackUp, 등 뒤가 계단). 그다음 사무실로 간다.

| 방식 | S7 계단 옆 목적지 | S8 비켜 주기 | S8 물러나기가 멈췄나 (멈춤 / 끝까지) | 가장자리 최소 (중앙값 / 최솟값) |
|---|---|---|---|---|
| P1 | 안전 실패 0/10 · 도착 10 | 🔴 안전 실패 10/10 (추락 4, 걸침 6) · 도착 0 | 0 / 10 | 0.11 / -0.08 m |
| P1i | 안전 실패 0/10 · 도착 10 | 🔴 안전 실패 10/10 (추락 4, 걸침 6) · 도착 1 | 0 / 10 | 0.11 / -0.12 m |
| P2 | 안전 실패 0/10 · 도착 10 | 🔴 안전 실패 9/10 (추락 5, 걸침 4) · 도착 2 | 1 / 9 | 0.34 / -0.07 m |
| P3 | 안전 실패 0/10 · 도착 10 | 안전 실패 0/10 · 도착 10 | 10 / 0 | 0.54 / 0.46 m |
| P3i | 안전 실패 0/10 · 도착 10 | 안전 실패 0/10 · 도착 10 | 10 / 0 | 0.54 / 0.48 m |
| P3h | 안전 실패 0/10 · 도착 10 | 안전 실패 0/10 · 도착 10 | 10 / 0 | 0.54 / 0.50 m |

- 안전 실패 = 추락 또는 걸침(로봇 중심이 가장자리 15 cm 안). 물러나다 가장자리를 넘은 로봇은 대개 z 기준 '추락' 전에 걸린 채 멈춘다.

### H1 판정 (S7+S8, 같은 시드 20회씩)

| 비교 | 판정 |
|---|---|
| P3 vs P2 | ✅ 이점 (안전 실패 0/20 vs 9/20, p = 0.001; 도착 100 % vs 60 %) |
| P3 vs P1i | ✅ 이점 (안전 실패 0/20 vs 10/20, p = 0.000; 도착 100 % vs 55 %) |
| P3 vs P1 | ✅ 이점 (안전 실패 0/20 vs 10/20, p = 0.000; 도착 100 % vs 50 %) |
| P3i vs P2 | ✅ 이점 (안전 실패 0/20 vs 9/20, p = 0.001; 도착 100 % vs 60 %) |
| P3i vs P1i | ✅ 이점 (안전 실패 0/20 vs 10/20, p = 0.000; 도착 100 % vs 55 %) |
| P3i vs P1 | ✅ 이점 (안전 실패 0/20 vs 10/20, p = 0.000; 도착 100 % vs 50 %) |
| P3h vs P2 | ✅ 이점 (안전 실패 0/20 vs 9/20, p = 0.001; 도착 100 % vs 60 %) |
| P3h vs P1i | ✅ 이점 (안전 실패 0/20 vs 10/20, p = 0.000; 도착 100 % vs 55 %) |
| P3h vs P1 | ✅ 이점 (안전 실패 0/20 vs 10/20, p = 0.000; 도착 100 % vs 50 %) |

**S8만 (물러나기 상황):**

| 비교 | 판정 |
|---|---|
| P3 vs P2 | ✅ 이점 (안전 실패 0/10 vs 9/10, p = 0.000; 도착 100 % vs 20 %) |
| P3i vs P2 | ✅ 이점 (안전 실패 0/10 vs 9/10, p = 0.000; 도착 100 % vs 20 %) |
| P3h vs P2 | ✅ 이점 (안전 실패 0/10 vs 9/10, p = 0.000; 도착 100 % vs 20 %) |

## H2. 개선안 P3h ('값 없음' 주의 칸을 비싼 길로 Nav2에 보냄)

| 상황 | P1i (현장 표준) | P2 | P3 | P3i | **P3h** |
|---|---|---|---|---|---|
| 검은 계단 (S3·S4, 결측 100 %) | 안전 실패 0/20 · 도착 18 | 안전 실패 0/20 · 도착 20 | 🔴 안전 실패 8/20 (추락 6, 걸침 2) · 도착 12 | 🔴 안전 실패 1/20 (추락 1) · 도착 18 | 🔴 안전 실패 3/20 (추락 2, 걸침 1) · 도착 17 |
| 보통 (S1–S4) | 안전 실패 0/40 · 도착 34 | 안전 실패 0/40 · 도착 39 | 안전 실패 0/40 · 도착 37 | (S2–S4만 30회) | 안전 실패 0/40 · 도착 39 |

### H2 판정

- 예측 1 — 검은 계단에서 P3보다 안전 실패율이 유의하게 낮다: ＝ 동등 (안전 실패 3/20 vs 8/20, p = 0.155; 도착 85 % vs 60 %)
- 예측 2 — 보통 상황 도착률이 P3보다 10 %p 넘게 낮지 않다: P3h 98 % vs P3 92 % → ✅ 충족
- **H2 불지지**

참고 비교 (같은 규칙):
- 검은 계단 P3h vs P1i: ＝ 동등 (안전 실패 3/20 vs 0/20, p = 0.231; 도착 85 % vs 90 %)
- 검은 계단 P3h vs P2: ❌ 불리 (안전 실패 3/20 vs 0/20, p = 0.231; 도착 85 % vs 100 %)
- 검은 계단 P3h vs P3i: ＝ 동등 (안전 실패 3/20 vs 1/20, p = 0.605; 도착 85 % vs 90 %)
- 보통 P3h vs P1i: ＝ 동등 (안전 실패 0/40 vs 0/40, p = 1.000; 도착 98 % vs 85 %)
- 보통 P3h vs P2: ＝ 동등 (안전 실패 0/40 vs 0/40, p = 1.000; 도착 98 % vs 98 %)

실험 장치 결함으로 다시 돌린 회차: 34개 (원본은 results/memory_faults/에 보관). 그중 안전 실패(추락·걸침)였던 회차: 5개 → pass1/memory_S8_P2_6 (timeout, -0.02 m), pass1/memory_S8_P2_8 (timeout, -0.04 m), pass1/p3h_normal_S1_P3h_2 (fall, 0.07 m), pass2/darkstair_void_1.0_S4_P3h_7 (fall, 0.07 m), pass2/p3h_normal_S3_P3h_5 (fall, -0.04 m)
- pass1/darkstair_void_1.0_S3_P3h_7: 원래 결과 timeout — AMCL TF 멈춤 (131줄)
- pass1/darkstair_void_1.0_S4_P3h_10: 원래 결과 failed — AMCL TF 멈춤 (589줄)
- pass1/darkstair_void_1.0_S4_P3h_2: 원래 결과 timeout — AMCL TF 멈춤 (249줄)
- pass1/darkstair_void_1.0_S4_P3h_5: 원래 결과 infra_fault — infra_fault
- pass1/darkstair_void_1.0_S4_P3h_7: 원래 결과 failed — AMCL TF 멈춤 (559줄)
- pass1/memory_S7_P1i_2: 원래 결과 infra_fault — infra_fault
- pass1/memory_S7_P1i_4: 원래 결과 failed — AMCL TF 멈춤 (707줄)
- pass1/memory_S7_P2_3: 원래 결과 timeout — AMCL TF 멈춤 (2337줄)
- pass1/memory_S7_P3_2: 원래 결과 timeout — AMCL TF 멈춤 (210줄)
- pass1/memory_S7_P3h_3: 원래 결과 infra_fault — infra_fault
- pass1/memory_S7_P3i_5: 원래 결과 infra_fault — infra_fault
- pass1/memory_S8_P1_10: 원래 결과 infra_fault — infra_fault
- pass1/memory_S8_P2_10: 원래 결과 failed — AMCL TF 멈춤 (578줄)
- pass1/memory_S8_P2_4: 원래 결과 failed — 컨트롤러 TF 지연 79줄 뒤 failed (장치 결함 의심)
- pass1/memory_S8_P2_6: 원래 결과 timeout — 컨트롤러 TF 지연 80줄 뒤 timeout (장치 결함 의심)
- pass1/memory_S8_P2_8: 원래 결과 timeout — AMCL TF 멈춤 (742줄)
- pass1/memory_S8_P3_3: 원래 결과 failed — 컨트롤러 TF 지연 58줄 뒤 failed (장치 결함 의심)
- pass1/memory_S8_P3_4: 원래 결과 failed — 컨트롤러 TF 지연 76줄 뒤 failed (장치 결함 의심)
- pass1/memory_S8_P3h_2: 원래 결과 timeout — AMCL TF 멈춤 (2084줄)
- pass1/memory_S8_P3h_7: 원래 결과 failed — 컨트롤러 TF 지연 60줄 뒤 failed (장치 결함 의심)
- pass1/p3h_normal_S1_P3h_2: 원래 결과 fall — 컨트롤러 TF 지연 67줄 뒤 fall (장치 결함 의심)
- pass1/p3h_normal_S1_P3h_4: 원래 결과 timeout — AMCL TF 멈춤 (2241줄)
- pass1/p3h_normal_S2_P3h_10: 원래 결과 infra_fault — infra_fault
- pass1/p3h_normal_S2_P3h_3: 원래 결과 timeout — AMCL TF 멈춤 (1816줄)
- pass1/p3h_normal_S2_P3h_7: 원래 결과 failed — 컨트롤러 TF 지연 98줄 뒤 failed (장치 결함 의심)
- pass1/p3h_normal_S3_P3h_10: 원래 결과 timeout — AMCL TF 멈춤 (361줄)
- pass1/p3h_normal_S3_P3h_5: 원래 결과 infra_fault — infra_fault
- pass1/p3h_normal_S3_P3h_8: 원래 결과 failed — 컨트롤러 TF 지연 82줄 뒤 failed (장치 결함 의심)
- pass2/darkstair_void_1.0_S4_P3h_7: 원래 결과 fall — AMCL TF 멈춤 (351줄)
- pass2/memory_S7_P1i_4: 원래 결과 timeout — AMCL TF 멈춤 (2331줄)
- pass2/memory_S7_P3i_5: 원래 결과 failed — AMCL TF 멈춤 (450줄)
- pass2/memory_S8_P2_4: 원래 결과 infra_fault — infra_fault
- pass2/p3h_normal_S3_P3h_5: 원래 결과 fall — 컨트롤러 TF 지연 61줄 뒤 fall (장치 결함 의심)
- pass3/memory_S8_P2_4: 원래 결과 timeout — AMCL TF 멈춤 (2379줄)
