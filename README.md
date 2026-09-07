# Patient Initial Motion Stability Workspace

전·후 2개의 steer-drive wheel과 4개의 passive caster를 가진 환자이승로봇의
초기 주행 안정성을 연구하기 위한 워크스페이스다.

이 저장소의 첫 목표는 제어기 구현이 아니다. 먼저 실제 USD 모델을 감사하고,
재현 가능한 초기조건 아래에서 어떤 변수가 초기 응답을 지배하는지 분리한다.

현재 플랫폼의 front/rear steering 물리 한계는 **±45°**다. USD 원본에는 ±90°가
authored되어 있으므로 연구용 override에서만 ±45°로 제한한다.

## 연구 질문

```text
초기 차체 상태 + 초기 바퀴 상태 + 경로 첫 구간 요구조건
                    ↓
              초기 주행 응답
```

다음 세 질문은 순서가 있는 독립 후보이며 처음부터 하나의 제어기로 합치지 않는다.

1. 현재 상태에서 요구 motion을 바로 실행할 수 있는가? (initial feasibility)
2. 출발 순간 실제 바퀴와 구동계 상태는 어떤가? (probing/estimation)
3. steering과 drive를 어떻게 함께 명령해야 안정적인가? (coupled control)

## 현재 상태

- 기준 wrapper USD를 `isaac_model/patient_transport.usd`에 원본과 동일하게 복사했다.
- wrapper가 참조하는 최소 USD dependency 5개도 `src/patient_transport_description/`에
  복사하여 robot reference가 해석되도록 했다.
- 로컬 환경에는 Isaac Sim `5.0.0.0`이 설치되어 있다.
- 기존 로그에서 Isaac Sim `5.0.0`, Kit `107.3.1` 실행 이력을 확인했다.
- 원본 wrapper와 복사본의 SHA-256은
  `0eefdabc717cbe110749dde3c20c61f47e6d47f96601d3d70e00be91f010aedb`로 동일하다.
- 현재 실행 환경에서는 NVIDIA GPU/드라이버가 노출되지 않아 물리 시뮬레이션 검증은
  수행하지 못했다. GUI가 가능한 호스트 세션에서 다시 확인해야 한다.

자세한 현황은 [docs/model_audit.md](docs/model_audit.md), Isaac Sim에서 직접 확인하는
방법은 [docs/isaac_sim_model_inspection.md](docs/isaac_sim_model_inspection.md), 전체
진행안은 [docs/research_plan.md](docs/research_plan.md)를 참조한다.

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
| assembled rigid-body mass | 약 240.337 kg |

## 실험 데이터 흐름

```text
experiment YAML
      ↓
deterministic reset → physics settling → reference motion
      ↓                                      ↓
state/command/effort logger              direct joint command
      ↓
CSV + metadata + plots + metric summary
```

Phase 1~3에서는 Nav2, SLAM, localization을 연결하지 않는다. 초기 wheel state와 physics의
영향이 분리된 이후에만 `Nav2 Path → Initial Motion Controller → Dual-Steer Adapter`를
연결한다.

## 빠른 시작

```bash
cd /home/jk/Research/patient_initial_ws
/home/jk/miniconda3/envs/isaac_env/bin/isaacsim
```

Isaac Sim에서 `File > Open`으로 `isaac_model/patient_transport.usd`를 연다. 첫 실행에서는
`docs/isaac_sim_model_inspection.md`의 `zero_motion`, `straight_sign`, `turn_sign`만 수행한다.

## 디렉터리 구조

```text
isaac_model/                   실행용 wrapper USD
assets/usd/                    향후 연구용 override/overlay layer
config/robot.example.yaml      조인트 보정·기하·물리 설정 스키마
config/experiments/            재현 가능한 실험 정의
docs/model_audit.md            모델 감사 보고서와 확인 체크리스트
docs/research_plan.md          단계, 실험설계, 완료 조건
results/                       생성 결과 (git 제외)
scripts/                       감사·실험·분석 진입점
src/patient_initial_stability/ Isaac 비종속 핵심 로직
tests/                         운동학·변환·지표 단위시험
```

## 즉시 다음 작업

1. GUI가 가능한 Isaac Sim에서 `isaac_model/patient_transport.usd`를 연다.
2. `docs/isaac_sim_model_inspection.md` 순서로 visual/physical zero와 sign을 확인한다.
3. 확인 결과로 `config/robot.yaml`의 `null` 항목을 채운다.
4. 원본을 수정하지 않고 연구용 USD/overlay를 만든다.
5. joint enumeration과 calibration 도구부터 구현한다.

## 불변 원칙

- 내부 계산과 로그는 SI 단위를 사용한다.
- raw joint angle과 physical steering angle을 분리한다.
- 조인트 이름, axis, zero, sign은 USD/Stage 확인 없이 추측하지 않는다.
- caster swivel은 reset 때만 초기화하고 simulation 중에는 구동하지 않는다.
- timestep, seed, settling 시간, 모든 초기조건을 metadata로 보존한다.
- Phase 1~3에서는 Nav2를 사용하지 않는다.
- caster 영향이 작다는 결과도 유효한 연구 결과로 취급한다.
