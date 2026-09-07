# Isaac 모델 감사 보고서

상태: **정적 USD 감사 완료, GUI/동적 검증 미완료**

이 문서는 Stage/UI와 읽기 전용 감사 스크립트에서 직접 확인한 값만 기록한다.
이름이나 부호를 관례로 채우지 않는다.

## 환경에서 확인된 사실

| 항목 | 확인값 | 상태/근거 |
|---|---:|---|
| Isaac Sim Python package | 5.0.0.0 | 로컬 conda metadata |
| 최근 Isaac Sim app | 5.0.0 | 실행 로그 |
| Kit | 107.3.1 | 실행 로그 |
| 현재 workspace 기준 USD | `isaac_model/patient_transport.usd` | 원본과 SHA-256 동일 |
| 원본 | `/home/jk/Research/4WS_bicycle_ws/isaac_test/patient_transport.usd` | 보존 |
| default prim / up axis / scale | `/World` / Z / 1 m | composed Stage 정적 검사 |
| articulation root | `/World/patient_transport/base_link` | ArticulationRootAPI |
| robot joint 수 | revolute 12개 | active 4 + caster 8 |
| Action Graph | 없음 | composed Stage에서 graph prim 없음 |
| GPU physics 실행 | 미검증 | 현재 세션에서 NVIDIA device/driver 미노출 |

실제 플랫폼의 front/rear 최대 조향각은 사용자 확인값 기준 **±45°**다. 현재 USD joint
limit은 ±90°이므로 둘이 일치하지 않는다. 원본은 보존하고 연구용 override layer에서
두 steering joint의 lower/upper limit을 `-45°/+45°`로 제한한다. 내부 config에는
`±0.7853981634 rad`로 기록한다.

wrapper는 아래 상대 reference에 의존한다.

```text
../src/patient_transport_description/urdf/patient_transport/patient_transport.usd
```

이를 구성하는 base/physics/robot/sensor USD도 함께 복사했다. FlatGrid는 NVIDIA의 원격
Isaac 5.0 asset을 참조하므로 offline 환경에서는 바닥이 누락될 수 있다. 본 실험용
overlay에서는 로컬 ground plane을 명시적으로 만드는 편이 안전하다.

## 정적으로 확인된 조인트

| 역할 | prim name | type | axis | limits (USD) | body0 → body1 | drive |
|---|---|---|---|---|---|---|
| front steering | `front_steering_joint` | revolute | Z | ±90.0002° | base → front steer | acceleration, K=4000, D=400, max=1500 |
| rear steering | `rear_steering_joint` | revolute | Z | ±90.0002° | base → rear steer | acceleration, K=4000, D=400, max=1500 |
| front drive | `front_wheel_joint` | revolute | X | unlimited | front steer → front wheel | force velocity, K=0, D=1, max=float max |
| rear drive | `rear_wheel_joint` | revolute | X | unlimited | rear steer → rear wheel | force velocity, K=0, D=1, max=float max |
| caster FL/FR/RL/RR swivel | `caster_*_swivel_joint` | revolute | Z | unlimited | base → caster fork | K=0, D=0 |
| caster FL/FR/RL/RR spin | `caster_*_wheel_joint` | revolute | X | unlimited | caster fork → wheel | K=0, D=0 |

caster joint에도 Angular Drive API가 붙어 있지만 stiffness와 damping이 모두 0이다.
따라서 현재 USD상 구동 토크는 발생하지 않는 passive DOF로 해석된다. 동적 실행에서
command를 전혀 보내지 않은 채 자유회전하는지를 최종 확인해야 한다.

## 확인된 기하

- front/rear steering axis: `x=+0.645/-0.645 m`, 따라서 `l_f=l_r=0.645 m`, `L=1.29 m`
- caster swivel 위치: `(±0.645, ±0.3365) m`
- caster wheel center는 swivel에서 local Y 방향으로 `0.095 m` 떨어져 있음
- 따라서 caster trail의 크기는 `0.095 m`; 방향 부호는 각 caster local frame과 함께 확인
- active wheel collision radius: `0.075 m`, width `0.060 m`
- caster wheel collision radius: `0.050 m`, width `0.030 m`
- base mass: `220 kg`, base local CoM: `(0, 0, 0.1) m`
- 모든 rigid body mass 합: 약 `240.3372 kg`

## 현재 모델에서 주의할 항목

- PhysicsScene에 timestep이 authored되어 있지 않다. `timeCodesPerSecond=60`은 USD timeline
  단위이며 physics timestep이라고 간주하면 안 된다.
- collision에 별도의 Physics Material/friction binding이 확인되지 않았다.
- drive wheel max force가 float 최대값으로 사실상 무제한이다. 실제 motor torque를 반영하기
  전에는 effort 비교가 물리적 의미를 갖기 어렵다.
- steering drive가 `acceleration` type이므로 링크 관성 변화에 둔감한 이상화 drive다.
  현재 모델을 정상 작동시키기 위한 baseline으로는 유지한다. payload/actuator-load 연구
  단계에서만 force drive 또는 실측 steering step response와 교차검증한다.
- USD target/state zero는 0°지만 이것이 물리적 직진 및 실제 homing zero와 같은지는 아직
  확인되지 않았다.

## 기준 모델 확정 후 채울 항목

### Stage

- [ ] 원본 USD 절대경로와 SHA-256
- [ ] 모든 reference/payload/sublayer와 해석 성공 여부
- [ ] default prim, up axis, metersPerUnit
- [ ] articulation root 경로 및 중복 articulation 여부
- [ ] PhysicsScene 경로
- [ ] timeStepsPerSecond와 substeps
- [ ] CPU/GPU dynamics, solver type, position/velocity iteration
- [ ] ROS 2 Action Graph/OmniGraph 경로와 노드 목록

### 조인트 GUI 교차검증

아래 항목은 정적으로 확인했지만 GUI와 동적 실행으로 교차검증한다.

| 역할 | prim path | type | axis | body0 → body1 | limits | drive mode | stiffness | damping | max force | 확인 |
|---|---|---|---|---|---|---|---:|---:|---:|---|
| front steering | `/World/patient_transport/joints/front_steering_joint` | revolute | Z | ±90° | acceleration | 4000 | 400 | 1500 | [ ] |
| rear steering | `/World/patient_transport/joints/rear_steering_joint` | revolute | Z | ±90° | acceleration | 4000 | 400 | 1500 | [ ] |
| front drive | `/World/patient_transport/joints/front_wheel_joint` | revolute | X | unlimited | force/velocity | 0 | 1 | unbounded | [ ] |
| rear drive | `/World/patient_transport/joints/rear_wheel_joint` | revolute | X | unlimited | force/velocity | 0 | 1 | unbounded | [ ] |
| caster FL swivel | TODO | TODO | TODO | TODO | TODO | passive | — | TODO | — | [ ] |
| caster FL spin | TODO | TODO | TODO | TODO | TODO | passive | — | TODO | — | [ ] |
| caster FR swivel | TODO | TODO | TODO | TODO | TODO | passive | — | TODO | — | [ ] |
| caster FR spin | TODO | TODO | TODO | TODO | TODO | passive | — | TODO | — | [ ] |
| caster RL swivel | TODO | TODO | TODO | TODO | TODO | passive | — | TODO | — | [ ] |
| caster RL spin | TODO | TODO | TODO | TODO | TODO | passive | — | TODO | — | [ ] |
| caster RR swivel | TODO | TODO | TODO | TODO | TODO | passive | — | TODO | — | [ ] |
| caster RR spin | TODO | TODO | TODO | TODO | TODO | passive | — | TODO | — | [ ] |

검사 시 passive caster joint에 angular drive target이 숨어 있지 않은지 반드시 확인한다.

### 조향 보정

물리적 직진 자세를 눈으로 확인하고 raw 값을 읽은 뒤 작은 양의 physical steering을
시험하여 sign을 결정한다.

```text
delta_f = s_f (q_f - q_f0)
delta_r = s_r (q_r - q_r0)
```

| 항목 | front | rear |
|---|---:|---:|
| straight raw angle `q0` [rad] | TODO | TODO |
| sign `s` (`+1`/`-1`) | TODO | TODO |
| physical lower limit [rad] | -0.7853981634 | -0.7853981634 |
| physical upper limit [rad] | +0.7853981634 | +0.7853981634 |
| max steering rate [rad/s] | TODO | TODO |

### 기하와 물리

- [ ] `l_f`, `l_r`, active wheel radius와 좌표를 transform에서 산출
- [ ] caster swivel axis와 wheel center 간 trail의 크기/방향을 산출
- [ ] 네 caster의 위치 `(x_i, y_i)`와 wheel radius를 산출
- [ ] rigid body별 mass, inertia tensor, principal axes, CoM
- [ ] payload가 결합되는 body와 fixed-joint 방식
- [ ] 각 collision shape와 collision approximation
- [ ] 바닥/능동륜/caster의 material binding
- [ ] static friction, dynamic friction, restitution, friction combine mode

### 동적 sanity check

- [ ] 2 s settling 후 root pose/velocity drift가 허용범위 이내
- [ ] steering +명령 시 physical `delta`가 정의한 양의 방향으로 증가
- [ ] drive +명령 시 차체가 정의한 +x 방향으로 전진
- [ ] caster command 없이 swivel/spin이 contact에 의해 자유롭게 반응
- [ ] 같은 reset/seed/step 수에서 결과가 반복 가능
- [ ] effort/torque 센서값의 제공 여부와 단위 확인

## 연구용 USD 정책

원본은 workspace 밖의 immutable source로 취급한다. `assets/usd/`에는 원본을 직접
수정한 사본 대신 가능하면 reference/override layer를 둔다. 연구 layer에는 물리
보정, sensor, Action Graph 등 연구 변경만 기록하고, 원본 경로와 해시를 metadata에
남긴다.
