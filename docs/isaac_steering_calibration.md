# 실측 기반 Isaac 조향 보정 절차

## 목적과 범위

실제 지면 시험의 전륜 -45°, 후륜 +45° 응답을 Isaac Sim에서 재현한다. 실제 공중시험은
수행하지 않았으므로 `air` 조건은 접촉을 제거한 이론 기반 actuator 확인이며 실측 공중
응답이 아니다. 토크·전류 실측도 없으므로 지면 보정으로 얻은 마찰값은 실제 물성치가
아니라 **등가 접촉 마찰계수**로만 해석한다.

실제 기준값은 `results/real_ground_analysis`에 있다.

| 지표 | 전륜 | 후륜 |
|---|---:|---:|
| T90 | 3.4740 s | 3.4753 s |
| ±1.5° 정착시간 | 3.7475 s | 3.7487 s |
| 최대 조향속도 | 13.8515°/s | 13.8674°/s |

Isaac 명령 생성기는 실제 2ELD2 설정을 조향각 단위로 환산해 사용한다.

```text
velocity     50000 count/s  = 13.732910°/s
acceleration 50000 count/s² = 13.732910°/s²
deceleration 80000 count/s² = 21.972656°/s²
```

## 1. 첫 실행: 가상 공중 조건

### 터미널: Isaac Sim 실행

```bash
cd /home/jk/Research/patient_initial_ws
/home/jk/miniconda3/envs/isaac_env/bin/isaacsim
```

Isaac Sim에서 다음 순서로 진행한다.

1. `File > Open`
2. `/home/jk/Research/patient_initial_ws/isaac_model/patient_transport.usd` 열기
3. Timeline이 `Stop` 상태인지 확인
4. `Window > Script Editor` 열기
5. stage 준비 helper를 로드

```python
exec(open("/home/jk/Research/patient_initial_ws/scripts/isaac_prepare_steering_stage.py").read())
```

6. 공중 조건과 120 Hz를 세션 레이어에 설정. 이 명령은 세션 레이어에서 로봇 중력을
   끄고 바닥 collision도 끈다. 단순히 5 m 올리기만 하면 약 1초 뒤 낙하하므로 이 격리가
   반드시 필요하다.

```python
prepare_steering_stage(condition="air", physics_hz=120)
```

7. Timeline의 `Play` 누르기
8. runner를 로드

```python
exec(open("/home/jk/Research/patient_initial_ws/scripts/isaac_dual_steering_test.py").read())
```

Console에서 다음이 확인되어야 한다.

```text
Isaac dual-steering calibration runner loaded.
Timeline playing: True
DOF indices: front=4, rear=5
```

다음 한 줄을 실행한다.

```python
start_dual_steering_test(condition="air", run_id="run03", physics_hz=120)
```

현재 진단용 runtime gain은 앞선 5° 공중 step 시험에서 정상 추종을 확인한
`kp=10000`, `kd=1000`이다. 기존 run01과 run02는 5 m 높이에서 중력으로 낙하하여 약
1초 뒤 바닥에 다시 충돌했으므로 gain 후보 판정에서 제외한다. run03부터 중력과 바닥
collision을 모두 제거한 실제 contact-free 조건이다. 이 단계가 통과하기 전에는 gain이나
마찰계수를 확정하지 않는다.

run03에서는 위치 profile만 보내고 목표속도를 0으로 유지해 damping에 의한 약 1.26°의
추종 지연이 확인됐다. run04부터는 실제 Profile Position 내부 궤적과 같이 위치와 해당
시점의 목표속도를 함께 전달한다. 따라서 run03은 접촉 제거 검증용이고, 최종 공중 기준은
run04 이후 결과로 정한다.

약 9초 동안 Timeline을 건드리지 않는다. 시험은 다음 순서로 자동 진행된다.

```text
0° 정착 2 s
→ 모델을 5 m 들어 올려 접촉 제거
→ 전륜 -45° / 후륜 +45° Profile Position 명령
→ 목표 유지 2 s
→ CSV/metadata 저장
→ 조향 목표 0° 복귀
```

다음 출력이 보이면 저장 완료다.

```text
Saved CSV: .../results/isaac_steering/air_mu_none_run01.csv
Saved metadata: .../results/isaac_steering/air_mu_none_run01.json
```

저장 문구를 확인한 후 Timeline의 `Stop`을 누른다. Stop 전에는 USD를 저장하지 않는다.

## 2. 가상 공중 결과 비교

일반 터미널에서 실행한다. Isaac Sim은 닫지 않아도 된다.

```bash
cd /home/jk/Research/patient_initial_ws
python3 scripts/compare_real_sim_steering.py \
  --sim results/isaac_steering/air_mu_none_run03.csv
```

출력 예시는 다음과 같다.

```text
front: passed=..., RMSE=... deg, T90=... s, settling=... s, vmax=... deg/s
rear:  passed=..., RMSE=... deg, T90=... s, settling=... s, vmax=... deg/s
```

가상 공중 조건에서는 우선 Profile Position 목표 추종을 본다. `passed=False`이면 즉시
마찰 보정으로 넘어가지 않고 생성된 CSV와 비교 그래프를 점검한다.

## 3. 지면 마찰 후보 시험

공중 조건이 정상일 때만 진행한다. 첫 후보는 마찰 0.00이다.

1. Isaac Sim에서 Timeline `Stop`
2. Script Editor에서 stage 준비 helper를 로드

```python
exec(open("/home/jk/Research/patient_initial_ws/scripts/isaac_prepare_steering_stage.py").read())
```

3. 120 Hz·±45° limit·임시 접촉재질 설정. ground 준비는 로봇 중력과 바닥 collision을
   다시 켠다.

```python
prepare_steering_stage(condition="ground", friction=0.00, physics_hz=120)
```

4. Timeline `Play`
5. runner 로드

```python
exec(open("/home/jk/Research/patient_initial_ws/scripts/isaac_dual_steering_test.py").read())
```

6. 지면 시험 실행

```python
start_dual_steering_test(condition="ground", friction=0.00, run_id="run01")
```

저장 완료 후 약 5초 더 기다려 조향이 0°로 복귀하게 하고 Timeline을 Stop한다. 일반
터미널에서 비교한다.

```bash
cd /home/jk/Research/patient_initial_ws
python3 scripts/compare_real_sim_steering.py \
  --sim results/isaac_steering/ground_mu_0p0000_run01.csv
```

동일한 절차로 다음 후보를 시험한다.

```python
prepare_steering_stage(condition="ground", friction=0.02, physics_hz=120)
prepare_steering_stage(condition="ground", friction=0.05, physics_hz=120)
prepare_steering_stage(condition="ground", friction=0.10, physics_hz=120)
```

각각 Stop 상태에서 준비→Play→runner load 후 아래 중 대응하는 명령 하나만 실행한다.

```python
start_dual_steering_test(condition="ground", friction=0.02, run_id="run01")
start_dual_steering_test(condition="ground", friction=0.05, run_id="run01")
start_dual_steering_test(condition="ground", friction=0.10, run_id="run01")
```

## 4. 보정 판정

비교기는 다음 조건을 모두 만족하면 축별로 `passed=True`를 출력한다.

| 판정값 | 허용범위 |
|---|---:|
| 실측 평균곡선 대비 RMSE | ≤ 0.5° |
| T90 절대차 | ≤ 0.10 s |
| 정착시간 절대차 | ≤ 0.10 s |
| 최대속도 상대오차 | ≤ 5% |
| 최종오차 | ≤ 0.1° |
| 오버슈트 | ≤ 0.2° |

마찰 후보 중 양 축을 모두 통과하면서 RMSE 합이 가장 작은 값을 선택한다. 공중 조건에서
확정한 kp/kd는 마찰 탐색 중 바꾸지 않는다. 여러 값을 동시에 조정하면 토크 실측 없이
gain과 마찰을 구분할 수 없기 때문이다.

## 5. 생성 파일

```text
results/isaac_steering/*.csv       Isaac 원시 시계열
results/isaac_steering/*.json      실행조건 metadata
results/isaac_comparison/*.csv     지표와 합격 여부
results/isaac_comparison/*.png     실측 평균과 Isaac 곡선 비교
```

가상 공중 `run01` 결과부터 확인한 뒤 필요할 때만 kp/kd 후보를 추가한다. 지면 마찰 grid는
공중 결과가 정상임을 확인하기 전에 일괄 실행하지 않는다.

## 6. 지면 등가 마찰 보정 결과

공중 기준은 `air_mu_none_run04`이며 양 축 모두 통과했다. 지면 후보 비교 결과는 다음과
같다. 최종각 도달성을 반영하기 위해 최종오차 허용값은 초기 0.5°에서 0.1°로 강화했다.

| 등가 마찰계수 | 전륜 RMSE | 후륜 RMSE | 최대 최종오차 | 강화 기준 판정 |
|---:|---:|---:|---:|---|
| 0.0000 | 0.286° | 0.296° | 0.001° | 통과 |
| 0.0025 | 0.247° | 0.254° | 0.084° | 통과 |
| 0.0060 | 0.220° | 0.227° | 0.211° | 실패 |
| 0.0200 | 0.452° | 0.485° | 0.754° | 실패 |

`0.0025`는 0이 아닌 접촉 저항을 반영하면서 곡선 RMSE, 시간응답 및 0.1° 최종오차를
동시에 만족하는 현재 최종 후보이다. 이 값은 실제 타이어 재료의 직접 측정값이 아니라,
현재 collision 형상과 acceleration drive 설정에 종속된 **등가 접촉 마찰계수**이다.

### 0.0025 반복 재현성

동일한 초기화 절차로 `run01`~`run03`을 반복한 결과, 세 회차의 306개 원시 샘플에서
관절 위치, 속도 및 measured effort가 모두 수치적으로 동일했다. 아래 표의 표준편차는
3회 표본표준편차이다.

| 지표 | 전륜 평균 ± 표준편차 | 후륜 평균 ± 표준편차 |
|---|---:|---:|
| RMSE | 0.246831 ± 0.000000° | 0.254048 ± 0.000000° |
| T90 | 3.451398 ± 0.000000 s | 3.451778 ± 0.000000 s |
| ±1.5° 정착시간 | 3.727663 ± 0.000000 s | 3.728309 ± 0.000000 s |
| 최대속도 | 13.722601 ± 0.000000°/s | 13.722372 ± 0.000000°/s |
| 최종오차 | 0.077785 ± 0.000000° | 0.084229 ± 0.000000° |
| 오버슈트 | 0.000000 ± 0.000000° | 0.000000 ± 0.000000° |

각 회차는 120 Hz에서 732 physics step과 6.100000 s의 simulation time을 동일하게
기록했다. wall time 차이는 GUI 렌더링 속도 차이이며 물리 결과에는 영향을 주지 않았다.
따라서 현재의 결정론적 시험 조건에서는 `mu_eq=0.0025`의 계산 재현성이 확인됐다.
