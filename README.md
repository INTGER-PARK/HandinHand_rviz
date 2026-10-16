# HandinHand RViz2 visualization (`hih_rviz`)

HandinHand_sim의 **Leader · Follower · Payload** 상태와 접촉을 RViz2에서 확인하는
별도 ROS 2 Python 패키지입니다. Ubuntu 22.04 / ROS 2 Humble을 대상으로 작성했습니다.
인터페이스 기준은 `INTGER-PARK/HandinHand_sim`의 **9b1e82e**입니다.

시각화 노드는 상태와 피드백을 구독하여 `/hih_viz/*`와 TF만 발행합니다.
기존 position/attitude command, admittance enable, actuator input에는 발행하지 않습니다.
기존 시뮬레이터 저장소의 파일을 수정하지 않습니다.

## 1. 무엇을 볼 수 있나요?

| 항목 | Leader / Follower | Payload |
| --- | --- | --- |
| 위치·자세 | state 또는 선택한 GT, 3D 기체·Body 축·수치 | 동일 MuJoCo world의 GT, 3D 상자·Body 축·수치 |
| 선속도 | World 화살표·수치 | World 화살표·수치 |
| 선가속도 | IMU specific force를 World 선가속도로 변환 또는 World 입력 사용 | GT World 속도의 simulation-time 차분 |
| 각속도 | Body 수치 + World로 회전한 화살표 | Body 수치 + World 화살표 |
| 실제 궤적 | 측정/GT 위치의 이력 | GT 위치의 이력 |
| 명령 궤적 | upstream `cmd`와 `cmd_admittance`를 각각 분리 | payload 목표 명령을 가정하지 않음 |
| CoP | MoB 노란 점, Solver 청록 점; 각각 실제 판면에 표시 | 별도 payload CoP 추정기를 가정하지 않음 |
| 판 경계 | 실제 판 외곽 + 실행 중인 CoP 제어기의 안전 경계 | 상자의 실제 외형 |
| 힘·모멘트 | MoB 추정 렌치와 Solver 접촉 렌치 | 두 드론의 접촉 합력/모멘트와 각 면의 GT 렌치 |
| solver 접촉점 | telemetry 래퍼 사용 시 선택적으로 표시 | 두 드론 판에서 표시 |

기체 형상은 plate와 rotor를 사용하는 **간략 모델**입니다. MuJoCo mesh/servo 자세를
그대로 가져오는 RobotModel은 아닙니다. Payload 외형과 접촉판 크기는 설정에서 바꿀 수 있습니다.

## 2. 빌드

먼저 기존 HandinHand_sim을 빌드하고, 그 시뮬레이터가 실행되는 Python/ROS 환경을 사용합니다.
`hih_interfaces`를 이 저장소에 복제하거나 메시지 정의를 다시 작성할 필요는 없습니다.
이 문서의 기존 workspace 경로는 본인의 경로에 맞게 변경하세요.

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_project/HandinHand_sim/install/setup.bash

sudo apt update
sudo apt install ros-humble-rviz2 ros-humble-tf2-ros \
  ros-humble-nav-msgs ros-humble-geometry-msgs ros-humble-visualization-msgs \
  ros-humble-std-srvs python3-numpy python3-yaml

mkdir -p ~/ros2_project/hih_visualization_ws/src
cd ~/ros2_project/hih_visualization_ws/src
git clone https://github.com/INTGER-PARK/HandinHand_rviz.git
# ZIP으로 받은 경우에는 여기에 HandinHand_rviz 폴더를 압축 해제합니다.

cd ~/ros2_project/hih_visualization_ws
colcon build --symlink-install --packages-select hih_rviz
source install/setup.bash
```

새 터미널에서도 `/opt/ros/humble`, 기존 simulator, visualization workspace 순서로 source합니다.
시뮬레이터와 다른 컴퓨터라면 ROS_DOMAIN_ID, ROS_LOCALHOST_ONLY 및 DDS 네트워크 설정도 맞춥니다.

## 3. 권장 실행: payload까지 모두 표시

기존 `cooperation_launch.py` 실행을 종료한 뒤 아래 launch로 **대체**합니다.

```bash
ros2 launch hih_rviz simulation.launch.py
```

이 launch는 기존 role-local controller/allocator/MoB와 **단 하나의 CooperationPlant**를
실행합니다. Plant executable만 `hih_plant_telemetry`로 바꾸어, 같은 MjData의 GT를 추가 발행합니다.
기본적으로 MuJoCo viewer와 CSV 기록은 끄고 RViz만 켭니다.

MuJoCo viewer도 함께 보고, Follower admittance를 launch에서 켜려면:

```bash
ros2 launch hih_rviz simulation.launch.py \
  viewer:=true embedded_admittance:=true admittance_enabled:=true
```

`embedded_admittance`는 **노드 실행 여부**, `admittance_enabled`는 그 노드의 **초기 ON/OFF**입니다.
둘의 기본값은 false이며, admittance를 켜면 기존 auto approach가 작동할 수 있습니다.
이 패키지는 trajectory 키 재입력이나 신호 지연에 따른 admittance OFF를 추가하지 않습니다.

별도 터미널에서 기존 조작 도구를 그대로 사용합니다.

```bash
ros2 run hih_cmd hih_teleop
# trajectory는 별도 터미널에서:
ros2 run hih_cmd hih_trajectory
```

CoP attitude/translation, torque DOB 노드는 원래 cooperation launch와 마찬가지로 자동 실행하지 않습니다.
실험에 필요한 노드만 기존 명령으로 실행하면 됩니다.

| simulation launch 인자 | 기본값 | 의미 |
| --- | --- | --- |
| `viewer` | false | MuJoCo viewer 추가 |
| `rviz` | true | RViz2 실행 |
| `cop_touch` | false | 기존 4-touch 계산/발행; RViz에서는 사용하지 않음 |
| `cop_solver` | true | Solver CoP 계산/발행 |
| `embedded_admittance` | false | Follower admittance 노드 실행 |
| `admittance_enabled` | false | 실행한 admittance 노드의 초기 enabled |
| `record_data` | false | 기존 hih_logging recorder 실행 |
| `log_output_dir` | 실행 디렉터리의 hih_recordings | CSV 저장 경로 |
| `drone_state_source` | YAML 설정 | `measured` 또는 `ground_truth` |
| `config` | 기본 visualization.yaml | 시각화 설정 파일 |

기존 launch와 달리 viewer/기록/4-touch의 기본값을 위와 같이 정했습니다.
기존 recorder의 topic registry에는 새 `/hih/ground_truth/*`가 등록되어 있지 않습니다.
RViz를 나중에 재생하려면 아래 rosbag 안내에 따라 GT도 함께 기록하세요.

## 4. 이미 실행 중인 시뮬레이터에 붙이기

```bash
ros2 launch hih_rviz visualization.launch.py
```

이 모드는 **추가 물리 시뮬레이터를 시작하지 않습니다**. 드론의 기존 토픽을 바로 표시합니다.
현재 HandinHand_sim main에는 payload odometry가 없으므로, 원래 plant만 실행했다면
payload는 `waiting/stale`로 표시됩니다. 두 드론 사이의 중간점으로 payload를 추정해 그리지 않습니다.
Payload 전체 상태가 필요하면 3번의 telemetry launch를 사용하세요.

센서 모델을 꺼서 `state.acc`가 World 선가속도로 발행되는 실험에서는:

```bash
ros2 launch hih_rviz visualization.launch.py acceleration_mode:=world_linear
```

전체 GT를 기준으로 시각화하려면 telemetry launch에서:

```bash
ros2 launch hih_rviz simulation.launch.py drone_state_source:=ground_truth
```

측정 상태 모드에서는 드론의 noise/delay가 포함된 pose로 해당 드론의 CoP/렌치를 그립니다.
정밀한 GT geometry 비교에는 `ground_truth` 모드를 사용하세요.

## 5. RViz만 빠르게 확인하는 데모

실제 simulator/teleop/trajectory를 종료한 별도 ROS domain에서 실행합니다.
데모는 같은 입력 토픽에 **합성 데이터**를 발행하므로 실제 시뮬레이션과 함께 실행하지 않습니다.

```bash
ROS_DOMAIN_ID=91 ros2 launch hih_rviz demo.launch.py
```

이 모드는 MuJoCo와 flight controller를 실행하지 않지만, ROS 메시지용 `hih_interfaces`는 필요합니다.
안전 경계는 실제 CoP 노드가 없으면 자동으로 그려지지 않습니다. 데모용 경계를 원하면
아래 manual 경계 파라미터를 설정하세요.

## 6. 좌표계·단위·힘의 의미

World는 MuJoCo의 오른손 Z-up 좌표입니다. RPY는 radians이며
`R_WB = Rz(yaw) Ry(pitch) Rx(roll)`입니다. ROS quaternion은 **xyzw** 순서입니다.

| 입력 | 해석 |
| --- | --- |
| `state.pos`, `state.vel` | World, m 및 m/s |
| `state.rpy` | World에 대한 Body 자세, rad |
| `state.w_rpy` | Body angular velocity, rad/s; Euler angle derivative가 아님 |
| high-fidelity `state.acc` | Body IMU specific force, m/s² |
| sensor model OFF의 `state.acc` | World linear acceleration, m/s² |
| MoB `external_wrench_hat_second_order` | 환경이 드론에 가하는 힘/모멘트, Body 성분, 모멘트 원점은 CoM |
| `contact_wrench_real` | payload-on-drone, Body 성분, 모멘트 원점은 plate geom center |
| `cop_hat`, `cop_solver_real` | +X 접촉판 **면 중심 C**에서의 y/z, m |
| payload `contact_wrench_from_drones` | drone-on-payload 합, payload Body 성분, payload CoM 모멘트 |
| payload `contact_wrench_from_{role}` | 각 face sensor 축/원점. +Fx가 양쪽 모두 압축 |

IMU 변환은 `a_W = R_WB f_B + g_W`입니다. 정지 hover이면 specific force가 약 9.81이라도
World 선가속도 화살표는 0이 됩니다. 실제 중력값이 바뀌면 `gravity_world`를 맞춥니다.

CoP는 `p_WCoP = p_WB + R_WB (r_BC + R_BC [0, y_C, z_C])`로 변환합니다.
Leader와 Follower의 +y를 임의로 뒤집지 않습니다. 마주 보는 두 Body의 yaw 차이가
World 표시 방향을 자연스럽게 반전시킵니다.

기본 XML에서 plate **geom center는 x=0.212 m**, CoP **face center는 x=0.215 m**입니다.
따라서 MoB 렌치 화살표는 CoM에, solver 렌치는 plate geom center에 표시합니다.
모멘트 원점을 섞어서 추정 오차처럼 보이게 하지 않습니다.
각속도와 모멘트 화살표는 회전축 방향의 벡터 표현입니다.

Payload 각 face sensor는 payload 기준 Follower 면 `[-0.2,0,0]`, Leader 면 `[+0.2,0,0]`이고,
Leader face 축은 payload 축에서 yaw=pi 회전합니다. Payload의 두 면 렌치와 드론이 받는
반작용 렌치는 부호/좌표계가 다르므로 같은 숫자로 해석하지 마세요.

GT 가속도는 `mj_objectVelocity(..., mjOBJ_XBODY, ..., flg_local=0)`의 WORLD 속도를
**simulation time**으로 차분합니다. 첫 샘플·같은 시간·reset·1초 초과 간격에서는 발행하지 않습니다.
순간 qacc/새 IMU/독립 observer가 아니며 finite difference와 decimation 오차가 있습니다.
원래 MjData를 읽고 기존 contact 계산 결과를 재사용합니다. 별도 mj_step/mj_forward는 호출하지 않습니다.
읽기 lock/ROS 발행의 계산 비용은 추가되지만 제어 법칙이나 물리 파라미터를 바꾸지는 않습니다.

## 7. 화면과 설정 변경

`config/visualization.yaml`이 시각화 기본값의 유일한 설정 파일입니다.
`world_frame`, topic template, 표시 항목, 크기, 화살표 배율, 이력 길이를 여기서 수정합니다.
사용자 설정 파일을 따로 복사해 실행할 수도 있습니다.

```bash
ros2 launch hih_rviz visualization.launch.py config:=/absolute/path/my_visualization.yaml
```

파라미터는 **시작 시 읽으며 실행 중 변경은 거부**합니다. 수정 후 노드를 재시작하세요.
실행 시 지정한 `drone_state_source` / `acceleration_mode` 인자만 YAML의 해당 값보다 우선합니다.
`--symlink-install`이면 소스 YAML 수정 후에도 설치된 파일 연결을 사용할 수 있습니다.
기본 설정을 일반 build로 설치했다면 다시 build해야 합니다.

| 색상 / 항목 | 의미 |
| --- | --- |
| 주황 / 파랑 / 연보라 | Leader / Follower / Payload와 실제 궤적 |
| 흰 궤적 | Leader upstream position reference 이력 |
| 초록 / 빨강 / 보라 화살표 | World velocity / World acceleration / angular velocity |
| 노랑 / 청록 | MoB CoP·추정 힘 / Solver CoP·접촉 힘 |
| 흰 사각형 | 실제 판 외곽 |
| 초록 / 분홍 사각형 | attitude / translation CoP 노드에서 읽은 안전 경계 |

RViz Displays에서 TF, reference/admittance Path를 개별로 켜고 끌 수 있습니다.
MarkerArray의 namespace들도 개별 표시할 수 있습니다.
Fixed Frame은 기본 `world`입니다. Body frame으로 바꾸면 궤적을 해석하기 어려워집니다.
전체 상태가 겹쳐 보이면 `show_labels: false`로 수치를 숨기고 화살표/모델만 보세요.

안전 경계는 각 CoP 노드의 `safe_y_m` / `safe_z_m`를 2초마다 읽습니다.
attitude와 translation의 경계가 다르면 각각 표시합니다. 노드가 없으면 값을 지어내지 않습니다.
시각화용 manual 사각형이 필요한 경우에만 `manual_safety_half_y_m`와 `manual_safety_half_z_m`를
양수로 설정하세요. **반폭 [m]이며 controller 설정에는 영향을 주지 않습니다.**

화살표 길이는 SI 벡터에 표시 배율을 곱한 후 `max_arrow_length_m`로 제한합니다.
수치와 표준 메시지에는 원래 물리값을 그대로 사용하며 표시 제한은 제어 제한이 아닙니다.

이력은 기본 1000점 / 최근 120초로 제한됩니다. 이력만 비우려면:

```bash
ros2 service call /hih_rviz/clear_history std_srvs/srv/Trigger '{}'
```

## 8. 토픽

`{role}`은 `leader` 또는 `follower`, `{body}`는 `leader/follower/payload`입니다.

| 구독 입력 | 타입 | 필수 여부 |
| --- | --- | --- |
| `/{role}/minitrone/state` | hih_interfaces/MinitroneState | measured 드론 표시 |
| `/hih/ground_truth/{body}/odometry` | nav_msgs/Odometry | payload 또는 GT 모드 |
| `/hih/ground_truth/{body}/acceleration` | geometry_msgs/AccelStamped | GT 가속도 |
| `/{role}/minitrone/cmd` | hih_interfaces/Cmd | upstream 경로, 선택 |
| `/{role}/minitrone/cmd_admittance` | hih_interfaces/Cmd | admittance 출력 경로, 선택 |
| `/{role}/minitrone/cop_hat` | hih_interfaces/CenterOfPressure | MoB CoP, 선택 |
| `/{role}/minitrone/cop_solver_real` | hih_interfaces/CenterOfPressure | Solver CoP, 선택 |
| `/{role}/minitrone/external_wrench_hat_second_order` | hih_interfaces/Wrench | 추정 렌치, 선택 |
| `/{role}/minitrone/contact_wrench_real` | hih_interfaces/Wrench | Solver 렌치, 선택 |
| `/{role}/minitrone/contact_diagnostics` | hih_interfaces/ContactDiagnostics | 선택 접촉점; 래퍼가 추가 발행 |
| `/payload/contact_wrench_from_drones` | hih_interfaces/Wrench | payload 접촉 합력, 선택 |
| `/payload/contact_wrench_from_{role}` | hih_interfaces/Wrench | payload 각 면 렌치, 선택 |

| 시각화 출력 | 타입 / 좌표 |
| --- | --- |
| `/hih_viz/markers` | visualization_msgs/MarkerArray, World |
| `/hih_viz/status` | std_msgs/String, missing/stale/source 상태 |
| `/hih_viz/{body}/odometry` | nav_msgs/Odometry; pose=World, twist=Body(child_frame_id) |
| `/hih_viz/{body}/acceleration` | geometry_msgs/Vector3Stamped, World **선가속도** |
| `/hih_viz/{body}/path` | nav_msgs/Path, 실제 위치 이력 |
| `/hih_viz/{role}/reference_path` | nav_msgs/Path, upstream 명령 이력 |
| `/hih_viz/{role}/admittance_path` | nav_msgs/Path, admittance 명령 이력 |
| `/hih_viz/{role}/wrench_estimated` | geometry_msgs/WrenchStamped, Body/CoM |
| `/hih_viz/{role}/wrench_contact` | geometry_msgs/WrenchStamped, Body/plate geom center |
| `/hih_viz/payload/wrench_contact` | geometry_msgs/WrenchStamped, payload Body/CoM |
| `/tf` | world → hih_viz/{body}/base_link → drone contact/plate_center |

`/hih/ground_truth/*` 및 새 contact diagnostics는 **plant telemetry 래퍼**의 출력입니다.
그 외 `/hih_viz/*`와 TF는 visualizer의 출력입니다.
이 표의 경로는 미래의 전체 planned trajectory가 아니라 지금까지 발행된 명령의 이력입니다.
controller 내부에서 합산되는 모든 CoP/attitude offset을 포함한 최종 desired pose라고 가정하지 않습니다.

원래 custom 메시지에는 Header가 없으므로 수신 시각으로 stamp를 추가합니다.
상태/렌치/CoP의 엄밀한 sample synchronization이나 latency 보정은 수행하지 않습니다.
표시용 odometry covariance는 새로 추정하지 않으며 기본 0 배열에 불확실성 평가 의미를 부여하지 않습니다.
같은 ROS 시간 기준을 사용해야 합니다. 기본 simulator는 /clock을 발행하지 않으므로
일반 실행은 `use_sim_time=false`를 사용합니다.

## 9. rosbag 재생

원본 headerless 입력과 payload GT를 함께 기록합니다. 작은 예시는 다음과 같습니다.

```bash
ros2 bag record -o hih_rviz_run \
  /leader/minitrone/state /follower/minitrone/state \
  /hih/ground_truth/payload/odometry /hih/ground_truth/payload/acceleration \
  /leader/minitrone/cop_hat /follower/minitrone/cop_hat \
  /leader/minitrone/cop_solver_real /follower/minitrone/cop_solver_real \
  /leader/minitrone/external_wrench_hat_second_order \
  /follower/minitrone/external_wrench_hat_second_order \
  /leader/minitrone/contact_wrench_real /follower/minitrone/contact_wrench_real \
  /leader/minitrone/cmd /follower/minitrone/cmd \
  /payload/contact_wrench_from_drones

# 실제 simulator를 종료한 뒤, 두 터미널에서:
ros2 bag play hih_rviz_run --clock
ros2 launch hih_rviz visualization.launch.py use_sim_time:=true
```

선택 표시 항목(각 face wrench, admittance 경로, GT 드론 state, 접촉점)도 필요하면 해당
입력 토픽을 record 목록에 추가하세요. rosbag을 기록하면서 RViz 출력까지 기록했다면,
재생할 때 visualizer와 같은 `/hih_viz/*`에 중복 발행하지 않게 하세요.

## 10. 문제가 있을 때

```bash
ros2 topic echo /hih_viz/status
ros2 topic hz /follower/minitrone/state
ros2 topic hz /hih/ground_truth/payload/odometry
ros2 topic echo /follower/minitrone/cop_solver_real --once
ros2 run tf2_ros tf2_echo world hih_viz/follower/base_link
```

| 증상 | 확인할 내용 |
| --- | --- |
| 모든 표시가 없음 | ROS 환경 source, domain, Fixed Frame=world, `/hih_viz/status` |
| payload만 없음 | 원래 cooperation plant에 attach했는지 확인; payload GT가 필요 |
| CoP 점 없음 | 해당 source 발행, valid=true, normal_force>0 확인 |
| 안전 사각형 없음 | CoP 노드 실행/이름/parameter service 확인; manual 기본은 숨김 |
| 정지인데 가속도가 약 9.81 | acceleration_mode가 sensor model 모드와 맞는지 확인 |
| GT 가속도만 잠깐 없음 | 첫 sample/물리 시간 정지/reset에서는 차분 미발행이 정상 |
| 과거 화살표가 없어짐 | stale timeout 또는 invalid 데이터에 따른 삭제가 정상 |
| rviz2 package 없음 | ros-humble-rviz2 설치 또는 `rviz:=false`로 데이터 출력만 실행 |

geometry/topic/frame이 원본 simulator에서 바뀌면 YAML도 맞추세요. `world_frame` 이름을 바꾸면
RViz Fixed Frame과 GT exporter의 `telemetry_world_frame`도 같은 이름을 사용해야 하며,
이름 변경 자체가 좌표 회전을 수행하는 것은 아닙니다.

## 11. 검증과 제한

ROS 없는 환경에서 실행 가능한 검사:

```bash
python3 -m pip install pytest numpy PyYAML mujoco==3.3.7
python3 -m pytest -q test
```

검사는 frame math, facing-drone CoP 부호, tilted hover의 specific-force 변환,
bounded history/reset, 실제 visualizer callback의 invalid/stale 삭제와 메시지 원점,
live safety query, 실제 MuJoCo XBODY velocity 및 exporter의 단일-world 읽기를 확인합니다.
ROS transport 대체물은 `test/ros_stubs.py` 안에서만 사용합니다.
MuJoCo가 없으면 그 라이브러리를 사용하는 검사만 skip합니다.

개발 환경에서는 ROS 2/DDS 및 RViz GUI가 설치되어 있지 않아 전체 colcon build,
실제 simulator와의 ROS topic 연결 및 RViz 렌더링은 아직 실행 검증하지 못했습니다.
배포 후 5번 데모와 3번 실험 launch로 화면/연결을 확인하세요.

## 12. 저장소와 업데이트

이 패키지는 https://github.com/INTGER-PARK/HandinHand_rviz 에서 별도로 관리합니다.
기존 `HandinHand_sim`과 독립된 저장소이며, simulator 설정 변경을 자동으로 가져오지 않습니다.

```bash
cd ~/ros2_project/hih_visualization_ws/src/HandinHand_rviz
git pull --ff-only
cd ~/ros2_project/hih_visualization_ws
colcon build --symlink-install --packages-select hih_rviz
source install/setup.bash
```

시뮬레이터 메시지·XML·좌표계가 변경되면 이 패키지의 입력 규약과 YAML도 확인하세요.

## 참고

- ROS Marker 형식: https://github.com/ros2/common_interfaces/blob/humble/visualization_msgs/msg/Marker.msg
- ROS Odometry 좌표 규약: https://github.com/ros2/common_interfaces/blob/humble/nav_msgs/msg/Odometry.msg
- MuJoCo object velocity: https://mujoco.readthedocs.io/en/3.3.7/APIreference/APIfunctions.html#mj-objectvelocity
