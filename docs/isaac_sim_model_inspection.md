# Isaac Sim 5.0 모델 점검·보정 실무 가이드

이 문서는 `isaac_model/patient_transport.usd`를 기준으로 한다. GUI에서 보이는 USD 각도는
degree이고 Python Articulation API의 각도는 radian이라는 차이를 항상 유지한다.

## 1. 모델 열기와 reference 확인

터미널에서 Isaac Sim을 실행한다.

```bash
cd /home/jk/Research/patient_initial_ws
/home/jk/miniconda3/envs/isaac_env/bin/isaacsim
```

이 설치의 `isaacsim` 명령은 첫 positional argument를 USD가 아니라 `.kit` experience로
해석하므로 USD 경로를 명령 뒤에 바로 붙이지 않는다. Isaac Sim에서 `File > Open`을
선택하고 다음 파일을 연다.

```text
/home/jk/Research/patient_initial_ws/isaac_model/patient_transport.usd
```

Stage 창에 최소한 다음 prim이 보여야 한다.

```text
/World/patient_transport
/World/patient_transport/base_link
/World/patient_transport/joints
/physicsScene
/FlatGrid
```

`World/patient_transport`가 비어 있거나 노란 경고가 보이면 reference가 끊긴 것이다.
`Layers` 창에서 root layer의 reference가 다음 경로로 resolve되는지 확인한다.

```text
src/patient_transport_description/urdf/patient_transport/patient_transport.usd
```

FlatGrid만 누락되는 경우에는 NVIDIA 원격 asset 접근 문제다. 로봇 reference 문제와
구분한다. 최종 실험에서는 외부 네트워크에 의존하지 않는 local ground를 사용한다.

## 2. articulation과 12 DOF 확인

`Tools > Physics > Physics Inspector`를 연다. Isaac Sim 5.0에서 Physics Inspector 메뉴는
이 경로에 있다. Stage에서 `/World/patient_transport/base_link`를 선택한다.

확인할 내용:

1. base_link에 `Physics Articulation Root`와 `Rigid Body`가 표시되는가
2. Physics Inspector의 articulation 목록에 robot이 나타나는가
3. 아래 12개 joint가 각각 하나의 DOF로 나타나는가
4. slider를 아주 작게 움직였을 때 대응 링크 하나만 예상 축으로 움직이는가

```text
front_steering_joint       Z revolute
rear_steering_joint        Z revolute
front_wheel_joint          X revolute
rear_wheel_joint           X revolute
caster_fl_swivel_joint     Z revolute
caster_fl_wheel_joint      X revolute
caster_fr_swivel_joint     Z revolute
caster_fr_wheel_joint      X revolute
caster_rl_swivel_joint     Z revolute
caster_rl_wheel_joint      X revolute
caster_rr_swivel_joint     Z revolute
caster_rr_wheel_joint      X revolute
```

DOF 판정 기준은 단순히 link가 보이는지가 아니다. joint prim의 type이
`PhysicsRevoluteJoint`이고, body0/body1이 유효하며, articulation runtime 목록에 나타나야
한다. 현재 정적 검사에서는 12개 모두 이 조건 중 앞의 두 조건을 만족한다.

### caster가 passive인지 판정

Stage에서 각 `caster_*_swivel_joint`를 선택하고 Property 패널에서 다음을 본다.

- Revolute Joint axis: Z
- Lower/Upper Limit: `-inf/+inf`
- Angular Drive stiffness: 0
- Angular Drive damping: 0
- simulation 중 controller target을 보내지 않음

현재 caster joint에는 Angular Drive API 자체는 존재한다. 그러나 gain이 0이므로 target
position 0은 구속력을 만들지 않는다. 최종 판정은 Play 후 차체를 천천히 움직였을 때
swivel angle이 contact physics에 따라 변하는지 확인하는 것이다.

## 3. 각 joint를 Property에서 보는 법

예를 들어 Stage에서 다음을 선택한다.

```text
/World/patient_transport/joints/front_steering_joint
```

Property 패널에서 확인한다.

- `Physics > Revolute Joint`: axis, lower/upper limit, body0/body1
- `Physics > Angular Drive`: drive type, target, stiffness, damping, max force
- `Raw USD Properties`: 실제 authored attribute와 적용 API

현재 front/rear steering은 acceleration drive, target 0°, stiffness 4000, damping 400,
max force 1500이다. wheel drive는 force/velocity drive, stiffness 0, damping 1이며 max
force가 사실상 무제한이다. 이 값은 baseline으로 기록하되 실제 actuator와 대응된다고
가정하지 않는다. 현재 USD의 steering limit은 ±90°지만 실제 플랫폼 제한은 ±45°이므로
연구용 override에서는 반드시 `Lower=-45°`, `Upper=+45°`로 설정한다.

### acceleration drive를 유지해도 되는가

유지해도 된다. acceleration drive는 원하는 가속도 응답을 만들도록 inertia 영향을
정규화하므로, CAD→URDF import 직후 안정적으로 position을 추종시키는 데 실용적이다.
지금 단계에서 force drive로 바꿔 모델을 다시 불안정하게 만들 필요는 없다.

다만 다음 해석상의 제한이 있다.

- payload나 steering-link inertia가 변해도 응답 변화가 실제 모터보다 작게 나타날 수 있다.
- max force와 measured effort를 실제 motor torque처럼 곧바로 해석하기 어렵다.
- 따라서 acceleration baseline 결과는 주로 궤적·각도·settling 비교에 사용한다.
- payload가 지배변수로 확인되면 동일 case를 force drive 또는 실측 step response 모델로
  재실행해 결론이 drive model에 의존하는지 확인한다.

즉 연구 순서는 `acceleration baseline 유지 → 영향변수 screening → 필요할 때만 actuator
model fidelity 향상`이다.

## 4. physical straight zero와 sign 보정

연구 좌표계를 먼저 고정한다.

- body +X: 전방
- body +Y: 좌측
- body +Z: 위
- positive steering: wheel rolling direction이 +X에서 +Y 쪽으로 회전

### GUI 절차

1. Timeline을 Stop한다.
2. Top view로 바꾼다.
3. front steering slider를 움직여 wheel plane이 body +X와 정확히 평행한 자세를 찾는다.
4. 그때 runtime joint position을 `q_f0`로 기록한다.
5. rear도 동일하게 `q_r0`를 기록한다.
6. raw angle을 약 `+5°` 움직인다.
7. physical steering이 정의한 양의 방향이면 `s=+1`, 반대면 `s=-1`로 기록한다.
8. wheel velocity도 작은 양의 값으로 시험해 차체 +X로 움직이는 sign을 기록한다.

계산은 항상 다음 변환을 통과시킨다.

```text
delta = s * (q_raw - q0)
q_raw_command = q0 + delta_command / s
```

`config/robot.yaml`의 `raw_straight_rad`, `physical_sign`, `velocity_sign`은 이 절차가 끝난
후에만 채운다. USD Property의 0°와 실제 플랫폼 homing의 ±90°를 혼합하지 않는다.

### Script Editor에서 상태 확인

Play를 누른 후 `Window > Script Editor`에서 다음 형태로 확인한다.

```python
from isaacsim.core.prims import SingleArticulation

robot = SingleArticulation("/World/patient_transport/base_link")
robot.initialize()

print(robot.dof_names)
print("position [rad] =", robot.get_joint_positions())
print("velocity [rad/s] =", robot.get_joint_velocities())
```

배열 index를 손으로 고정하지 말고 반드시 이름으로 만든다.

```python
index = {name: i for i, name in enumerate(robot.dof_names)}
front_steer_i = index["front_steering_joint"]
rear_steer_i = index["rear_steering_joint"]
```

## 5. reset과 caster 초기각 지정

reset 순서는 매 case 동일해야 한다.

```text
Pause/Stop
→ root pose/velocity 설정
→ 12개 joint position 설정
→ 12개 joint velocity를 0으로 설정
→ physics step 시작
→ 2 s settling
→ logging 시작
→ reference command 시작
```

`set_joint_positions`는 joint를 즉시 이동시키는 state setter이므로 reset에만 사용한다.
주행 중 steering 제어에는 `apply_action`을 사용한다. caster swivel도 reset에서만 position을
설정하고 settling 이후에는 caster index에 position/velocity/effort command를 보내지 않는다.

caster physical angle `phi`에도 steering과 같은 별도 zero/sign 변환을 둔다.

```text
q_caster_raw = q_caster_zero + phi / caster_sign
```

회전 reference의 caster 초기 이상각은 모든 caster를 0으로 두는 것이 아니라 각 설치
위치 `(x_i,y_i)`의 속도로 계산한다.

```text
vx_i = vx - wz*y_i
vy_i = vy + wz*x_i
phi_i_ideal = atan2(vy_i, vx_i)
```

그 후 `phi_i = wrap(phi_i_ideal + offset_i)`로 `±90°`, `180°` 조건을 만든다.

## 6. steering과 wheel 명령 만들기

내부 입력은 SI 단위로 유지한다.

```text
steering position: rad
steering velocity: rad/s
wheel angular velocity: rad/s
ground speed: m/s
```

body twist `(vx, vy, wz)`에 대해 wheel 위치 `(x_i,y_i)`의 속도는:

```text
v_point = [vx - wz*y_i, vy + wz*x_i]
```

steering angle `delta_i`의 rolling unit vector가
`t_i=[cos(delta_i), sin(delta_i)]`이면 ground rolling speed는:

```text
v_roll_i = dot(v_point, t_i)
wheel_joint_velocity_i = velocity_sign_i * v_roll_i / wheel_radius_i
```

현재 active wheel radius는 `0.075 m`이다. 예를 들어 직진 ground speed가 `0.15 m/s`이면
크기는 `0.15/0.075 = 2 rad/s`다. 실제 명령 부호는 calibration 후 적용한다.

position과 velocity command는 한 DOF에 동시에 주지 않는다. steering은 position target,
drive wheel은 velocity target을 사용하며 caster command는 비워 둔다.

## 7. timestep·material·drive 값 지정

현재 USD의 `timeCodesPerSecond=60`은 physics timestep이 아니다. PhysicsScene에는 timestep이
저장되어 있지 않다. 연구용 overlay 또는 실행 코드에서 `physics_dt`를 명시한다.

권장 pilot 설정:

```yaml
physics_dt_s: 0.008333333333333333  # 120 Hz
rendering_dt_s: 0.016666666666666666 # 60 Hz
settling_duration_s: 2.0
initial_window_s: 2.0
run_duration_s: 5.0
```

이는 확정 물리값이 아니다. 동일 case를 60/120/240 Hz에서 한 번씩 실행해 주요 metric이
수렴하는지 확인하고 가장 낮은 수렴 timestep을 고정한다.

현재 robot collision에는 명시적인 physics material이 없다. 실험 전 최소한 ground,
active wheel, caster wheel 세 material을 분리하고 static/dynamic friction과 restitution을
명시한다. 측정값이 없다면 pilot용 시작값으로 다음을 사용할 수 있지만 본 실험 전에는
실물 재질 또는 traction test로 교정한다.

```yaml
ground:       {static_friction: 0.8, dynamic_friction: 0.6, restitution: 0.0}
active_wheel: {static_friction: 0.8, dynamic_friction: 0.6, restitution: 0.0}
caster_wheel: {static_friction: 0.8, dynamic_friction: 0.6, restitution: 0.0}
```

Property에서 collision prim을 선택하고 `Create/Add > Physics > Physics Material`로 material을
만든 뒤 collider의 physics material binding에 연결한다. 원본 layer가 아니라 연구용
override layer가 edit target인지 먼저 확인한다.

drive 값은 우선 기존값을 보존한다. acceleration drive도 그대로 사용한다. 다만 drive wheel
max force가 무제한이므로 실제 motor torque limit 또는 실험으로 식별한 값을 얻기 전 effort
관련 결론을 내리지 않는다.

## 8. 첫 실행값과 합격 기준

안전한 pilot용 시작값:

```yaml
reference_speed_m_s: 0.15
turn_steering_front_rad: 0.261799  # +15 deg
turn_steering_rear_rad: -0.261799  # -15 deg
steering_gate_tolerance_rad: 0.0174533 # 1 deg
steering_gate_dwell_s: 0.2
low_speed_m_s: 0.03
nominal_speed_m_s: 0.15
repeats: 5
```

첫 단계의 목적은 성능 우열이 아니라 다음 sanity check 통과다.

- 5회 reset 후 초기 pose/joint state가 허용 오차 내 동일
- +steering 명령이 정의한 physical +방향과 일치
- +wheel velocity가 body +X 이동과 일치
- command가 없는 caster가 자유롭게 swivel/spin
- 바닥 관통, contact 폭발, 고주파 진동이 없음
- logger timestamp 간격이 physics step의 정수배

## 9. 첫 세 실험 묶음

1. `zero_motion`: drive/steering 명령 없이 5 s, settling drift와 caster 자유도 확인
2. `straight_sign`: caster 정렬, `0.03 → 0.15 m/s`, drive sign과 wheel radius 확인
3. `turn_sign`: `front=+5°`, `rear=-5°`, 저속 주행으로 yaw sign과 조향 sign 확인

이 세 묶음이 통과한 후에만 `±15°` initial-angle sensitivity와 세 timing policy 비교로
넘어간다.
