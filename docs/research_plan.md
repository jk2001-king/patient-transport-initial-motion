# 초기 주행 안정성 연구 계획

## 1. 연구 범위와 응답 정의

플랫폼은 differential drive나 일반 4WS로 환원하지 않고 다음으로 정의한다.

```text
front center steer-drive + rear center steer-drive + four passive casters
```

초기 구간은 명령 시작 `t=0`부터 고정된 `T_init`까지로 정의한다. `T_init`은 실험 전에
고정하며 결과를 본 뒤 바꾸지 않는다. 한 숫자로 “안정성”을 미리 합치지 않고 다음
응답을 각각 보존한다.

- 정확도: lateral/path deviation, heading error
- 동특성: yaw-rate peak, yaw-acceleration peak, overshoot
- 수렴성: settling time, settling distance
- 승차감: planar acceleration과 jerk의 peak/RMS
- 물리 부담: drive/steering effort peak와 integral, slip proxy

주요 결과변수와 허용오차, settling dwell time은 pilot run 전에 설정한다. 센서 차분으로
가속도·jerk를 계산할 때는 filter 종류와 cutoff도 실험 metadata로 고정한다.

## 2. 단계와 통과 조건

| Phase | 목적 | 산출물 | 다음 단계 통과 조건 |
|---|---|---|---|
| 0 | 모델 감사 | audit report, `robot.yaml`, 연구용 USD | 모든 joint/axis/zero/sign/physics 확인 |
| 1 | 계측·reset 검증 | inspector, resetter, logger | 동일조건 반복성과 단위시험 통과 |
| 2 | 기준 motion | straight/constant-curvature runner | 운동학 부호와 정상상태 응답 확인 |
| 3 | active-wheel screening | 초기각·transient·timing 결과 | 큰 효과 변수와 상호작용 식별 |
| 4 | caster/load screening | caster offset·payload 결과 | caster 효과 크기와 하중 의존성 판단 |
| 5 | 방법 선택 | decision report | 데이터로 후보 제어법 하나 우선 선정 |
| 6 | 제안법 검증 | controller + baseline comparison | holdout 조건에서도 개선 확인 |
| 7 | Nav2 연동 | dual-steer adapter/controller | 현상 분리 결과를 훼손하지 않음 |
| 8 | Sim-to-Real | calibration + matched tests | 동일 metric/condition 체계로 비교 |

Phase 통과 전 다음 단계 코드는 최소화한다. 특히 Phase 5 이전에는 MPC, probing,
feasibility filter를 모두 구현하지 않는다.

## 3. 구현 순서

### Phase 0–1: 신뢰 가능한 실험 장치

1. `inspect_stage.py`: articulation, joint, drive, rigid body, material, graph 열거
2. `calibrate_steering.py`: straight raw zero와 sign을 사람 확인과 함께 기록
3. `reset_robot.py`: root/joint pose·velocity, caster 초기각, settling을 원자적으로 수행
4. `command_adapter.py`: physical angle↔raw angle, ground speed↔joint rad/s 변환
5. `logger.py`: command/state/effort와 experiment metadata 동시 저장
6. `run_experiment.py`: YAML→reset→settle→run→CSV/metadata
7. `run_batch.py`: case×repeat 실행, deterministic case ID, 실패 재실행 정보 저장
8. `analyze.py`: metric table과 표준 plot 생성

Isaac API를 호출하는 adapter와 순수 운동학/지표 코드를 분리한다. 그래야 순수 코드는
Isaac 없이 빠르게 단위시험할 수 있다.

### Phase 2: reference motion

- Straight: `v=v_ref`, `omega=0`
- Constant curvature: 우선 `delta_f=+15°`, `delta_r=-15°` 후보
- 실제 limit와 wheelbase 감사 후 각도를 확정
- step command뿐 아니라 동일 duration의 rate-limited command를 명시적으로 구분

일반 저속 planar 근사의 기준은 다음과 같다.

```text
kappa ≈ (tan(delta_f) - tan(delta_r)) / L
delta_f* = atan2(omega*l_f, v)
delta_r* = atan2(-omega*l_r, v)
```

`v≈0` 근방의 `atan2` 결과, steering limit, steering-rate limit, wheel-speed 산식은 별도
edge case로 다룬다. 실제 front/rear steering limit은 ±45°이므로 모든 reference와 controller
출력은 `[-0.7853981634, +0.7853981634] rad`로 제한한다. 이 식이 실제 6-wheel contact의
완전 모델이라고 가정하지 않는다.

### Phase 3: active-wheel 우선 screening

초기 차체 상태는 `(x,y,yaw,v,omega)=(0,0,0,0,0)`으로 고정한다. caster는 각 reference에
대한 이상 방향으로 맞춘다.

- ideal target/target
- front error: `0/target`
- rear error: `target/0`
- both error: `0/0`
- asymmetric: 예 `+5°/-20°`

큰 steering error 한 조건에서 timing policy를 비교한다.

- simultaneous steering + drive
- steering-first: 양쪽 error가 tolerance 이내이고 dwell time을 만족하면 drive
- speed-ramp: error가 클 때 저속, 작아질수록 nominal speed

정책 비교에서 초기 state, steering profile limit, reference, settling, timestep, seed는
동일하게 유지한다.

### Phase 4: caster와 payload

Straight caster offsets: `0°, +90°, 180°`, 좌우 비대칭, 전후 비대칭.

회전에서는 caster별 ideal direction을 먼저 계산한다.

```text
phi_i* = atan2(v_y + omega*x_i, v_x - omega*y_i)
```

그 기준에 `+90°, -90°, 180°` offset을 준다. caster 각도는 wrap convention을 하나로
정하고 metadata에 ideal, offset, raw reset value를 모두 기록한다.

하중은 empty/nominal/heavy로 나누되 total mass만 바꾸지 않는다. 실제 payload geometry로
mass, inertia, CoM을 함께 변경하고 조합별로 기록한다.

## 4. 실험 설계와 분석

- 첫 pilot은 오류·단위·접촉 거동을 찾기 위한 소수 case이며 결론에 사용하지 않는다.
- 본 실험은 case마다 동일한 반복 횟수와 고정 seed 목록을 쓴다.
- 실행 순서는 가능한 한 무작위화하되 case ID는 결정적으로 생성한다.
- 결과에는 config snapshot, USD hash, git commit, Isaac version, timestep을 저장한다.
- 평균만 제시하지 않고 effect size, 반복 변동, confidence interval을 함께 제시한다.
- active angle×timing, caster offset×payload 같은 사전 지정 상호작용을 검사한다.
- screening 후 가장 영향이 큰 변수만 촘촘한 grid/response-surface 실험으로 확장한다.

권장 1차 판정은 “통계적으로 유의한가”보다 “물리적으로 의미 있는 임계치를 넘는가”다.
caster 효과가 repeat noise나 실용 임계치보다 작으면 caster-centric controller는 보류한다.

## 5. 제어 방법 선택 규칙

| 관찰 결과 | 우선 후보 |
|---|---|
| 요구 초기 motion이 joint/rate/contact constraint 밖에 자주 존재 | initial feasible-motion filter/trajectory regeneration |
| 미관측 저항·하중·caster 상태에 따른 run-to-run 차이가 큼 | 짧은 probing + online estimation |
| 알려진 steering transient와 drive timing의 결합이 지배적 | coupled initial controller 또는 short-horizon MPC |
| 단순 speed-ramp가 대부분의 위험을 제거 | 복잡한 제어기 대신 검증 가능한 rule-based policy |
| caster 효과가 작음 | caster는 강건성 검증 변수로 남기고 active-wheel 중심 제어 |

선정한 방법은 simultaneous, steering-first, speed-ramp 중 적어도 두 baseline과 동일한
holdout 초기조건에서 비교한다.

## 6. Nav2 경계

Phase 0–6은 Isaac physics + simple reference + direct joint command로 수행한다. 이후:

```text
Static Map → Nav2 Global Planner → nav_msgs/Path
           → Initial Motion Controller → Dual-Steer Adapter → Isaac robot
```

초기에는 ground-truth localization을 사용한다. `/cmd_vel`을 wheel joint에 직접 연결하지
않으며 `v, omega`에서 front/rear steering과 drive speed를 계산하는 adapter를 둔다.
AMCL/SLAM/3D LiDAR localization은 초기조건 효과가 분리된 뒤 추가한다.

## 7. 결정이 필요한 입력

- 기준 USD가 발견된 `patient_transport.usd`인지 다른 파일인지
- 실제 플랫폼의 `l_f`, `l_r`, wheel radii 또는 이를 검증할 CAD 기준
- nominal/heavy payload 질량·장착 위치·형상
- steering/drive actuator의 실제 rate, acceleration, torque limits
- initial interval과 실용적 안정성 허용치
- Isaac GUI/GPU가 가능한 실행 방식과 ROS 2 배포판
