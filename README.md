# Patient Transport Initial Motion Stability

전·후 2개의 steer-drive wheel과 4개의 passive caster를 가진 환자이승로봇의
초기 주행 안정성을 연구하기 위한 ROS 2–Isaac Sim 워크스페이스다.

현재 목표는 **조향 구성, 능동륜 초기 조향상태, 경로 첫 구간의 요구운동**을 함께 보고
즉시 출발 또는 조향오차 기반 제한 출발을 선택하는 configuration-aware initial motion
control을 검증하는 것이다. Passive caster 초기방향은 보조 변수로 유지하되 caster-only
controller는 선행 screening에서 반복 가능한 차체 교란이 확인되지 않아 주 연구범위에서
제외했다.

현재 플랫폼의 front/rear steering 물리 한계는 **±45°**다. 워크스페이스 USD에는
±45°, 120 Hz, 검증된 steering drive gain과 조향응답 등가 접촉값이 적용되어 있다.

## 연구 질문

```text
초기 차체 상태 + 초기 바퀴 상태 + 경로 첫 구간 요구조건
                    ↓
              초기 주행 응답
```

1. 현재 전·후륜 조향상태에서 경로 첫 명령을 즉시 실행해도 되는가?
2. 전륜 단독, 후륜 단독, 전후륜 역상 조향에서 어떤 조향오차를 출발 판단에 사용해야 하는가?
3. 경로 정확도를 유지하면서 불필요한 정렬 대기시간을 줄일 수 있는가?

## 현재 상태

- 실제 플랫폼 caster Go/No-Go 20회와 Isaac Sim caster screening에서 caster 초기방향에
  따른 반복 가능한 횡편차·yaw 증가를 확인하지 못했다.
- 조향 raw zero `+90°`, 물리 조향범위 `±45°`, 전진 구동부호 front/rear=`+/-`,
  물리주기 120 Hz를 확인했다.
- 실제 조향응답으로 최대 조향속도 `13.732910°/s`와 조향 drive gain을 보정했다.
- `front_only`, `rear_only`, `coordinated_counterphase` 세 운동학 구성을 구현했다.
- 단일축 조향의 body-center sideslip을 보정하는 configuration-aware tracking model을
  구현하고 단위시험으로 검증했다.
- 초기 조향오차를 common/counter component로 분해하여 조향 구성별 출발 기준을 적용한다.
- ROS 2–Isaac Sim 폐루프 reference-path 및 Nav2 goal 시험을 구성했다.
- Nav2 반복시험에서 전륜 단독 `+40°`, 후륜 단독 `+40°`, 전후륜 동상 `+25°/+25°`
  조건 모두 3/3 goal 성공을 유지하면서 제안방법이 동시출발 대비 초기 최대 횡편차와
  yaw를 약 52–67% 감소시켰다. 현재 기준값은 screening 결과이며 일반화 검증이 남아 있다.

모델 확인은 [docs/isaac_sim_model_inspection.md](docs/isaac_sim_model_inspection.md), 실측
응답 기반 조향 보정은 [docs/isaac_steering_calibration.md](docs/isaac_steering_calibration.md),
초기 연구계획은 [docs/research_plan.md](docs/research_plan.md)를 참조한다.

## 모델 요약

| 항목 | 값 |
|---|---:|
| 구동 구조 | front/rear center steer-drive + passive caster ×4 |
| articulation root | `/World/patient_transport/base_link` |
| revolute DOF | 12개 (active 4 + passive 8) |
| wheelbase | 1.29 m |
| active wheel radius | 0.075 m |
| caster wheel radius / trail | 0.050 m / 0.095 m |
| steering physical limit | ±45° |
| body forward / left axis | local `+X` / `+Y` |
| positive-forward drive sign | front `+`, rear `-` |
| assembled rigid-body mass | 약 240.337 kg |

## 실험 데이터 흐름

```text
Nav2 또는 reference-path tracker
               ↓ Twist
configuration-aware initial-motion controller
               ↓ JointState target
Isaac Sim platform → ground-truth odometry / joint states
               ↓
JSON + plots + transient/goal metrics
```

## 빠른 시작

```bash
conda activate isaac_env
isaacsim
```

Isaac Sim에서 `File > Open`으로 `isaac_model/patient_transport.usd`를 연다. Script Editor에서
ROS 2 graph를 준비하고 Timeline을 Play한 뒤 패키지를 빌드한다.

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select patient_initial_stability
source install/setup.bash
ros2 launch patient_initial_stability sim_nav2_bringup.launch.py \
  configuration:=coordinated_counterphase \
  initial_alignment_policy:=configuration_adaptive
```

Stage 준비와 조향 보정 절차는 기술 문서를 따른다. `results/`는 실행 시 생성되며 Git에는
포함하지 않는다.

## 디렉터리 구조

```text
isaac_model/                    보정된 실행용 USD
config/robot.yaml              조인트·기하·물리 기준값
config/experiments/             재현 가능한 초기 실험 정의
docs/                           모델 점검·보정·연구 절차
scripts/                        Isaac 준비·실험·분석 진입점
src/patient_initial_stability/  ROS 2 제어기·Nav2·시험 패키지
results/                        생성 결과 (Git 제외)
```

## 즉시 다음 작업

1. 초기각·속도·경로곡률을 바꾼 반복시험으로 현재 threshold의 민감도를 검증한다.
2. 곡선 시작, 좁은 통로 등 초기 deviation이 navigation 여유를 줄이는 조건을 추가한다.
3. 초기 횡가속도·yaw rate·jerk를 기록하여 환자 승차감 관련 지표를 평가한다.
4. 실제 플랫폼의 전륜·후륜·전후륜 모드에 적용하고 localization/IMU/모터전류를 기록한다.
5. 필요할 때만 caster 초기상태와 바퀴별 지지하중을 보조 변수로 추가한다.

## 불변 원칙

- 내부 계산과 로그는 SI 단위를 사용한다.
- raw joint angle과 physical steering angle을 분리한다.
- 조인트 이름, axis, zero, sign은 USD/Stage 확인 없이 추측하지 않는다.
- caster swivel은 reset 때만 초기화하고 simulation 중에는 구동하지 않는다.
- timestep, seed, settling 시간, 모든 초기조건을 metadata로 보존한다.
- caster 영향이 작다는 결과도 유효한 연구 결과로 취급한다.
- 현재 threshold를 최적값으로 주장하지 않고 반복·민감도 검증 전에는 screening 값으로 부른다.
