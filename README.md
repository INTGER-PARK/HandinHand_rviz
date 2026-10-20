# HandinHand RViz2 visualization (`hih_rviz`)

HandinHand_sim의 **Leader · Follower · Payload** 상태와 접촉을 RViz2에서 확인하는
별도 ROS 2 Python 패키지입니다. Ubuntu 22.04 / ROS 2 Humble을 대상으로 작성했습니다.
드론 인터페이스 기준은 `INTGER-PARK/HandinHand_sim`의 **9b1e82e**입니다.
Payload 전체 표시는 원본 simulator의 payload telemetry 추가 버전이 필요합니다.

**이미 실행 중인 MuJoCo 시뮬레이터를 관찰하는 시각화 전용 패키지입니다.**
기존 상태·명령·피드백을 구독하고 `/hih_viz/*`와 표시용 TF만 발행합니다.
시뮬레이터, 제어기, admittance, allocator, observer, teleop, trajectory를 실행하지 않습니다.
기존 position/attitude command, admittance enable, actuator input, 상태·힘 피드백 토픽에는
발행하지 않으며, 원격 파라미터도 변경하지 않습니다. 기존 시뮬레이터 파일을 수정하지 않습니다.

0.2.0부터 plant를 실행하던 `simulation.launch.py`/`hih_plant_telemetry`와 원본 입력 토픽에
합성 데이터를 발행하던 `demo.launch.py`/`hih_visualization_demo`를 제거했습니다.
실행 방법은 아래의 `visualization.launch.py` 하나입니다.

## 1. 무엇을 볼 수 있나요?

| 항목 | Leader / Follower | Payload |
| --- | --- | --- |
| 위치·자세 | 기존 state, 3D 기체·Body 축·수치 | 원본 plant의 payload GT, 3D 상자·Body 축·수치 |
| 선속도 | World 화살표·수치 | World 화살표·수치 |
| 선가속도 | IMU specific force를 World 선가속도로 변환 또는 World 입력 사용 | 원본 plant가 World 속도를 simulation time으로 차분한 값 |
| 각속도 | Body 수치 + World로 회전한 화살표 | Body 수치 + World 화살표 |
| 실제 궤적 | 측정/GT 위치의 이력 | GT 위치의 이력 |
| 명령 궤적 | upstream `cmd`와 `cmd_admittance`를 각각 분리 | payload 목표 명령을 가정하지 않음 |
| CoP | MoB 노란 점, Solver 청록 점; 각각 실제 판면에 표시 | 별도 payload CoP 추정기를 가정하지 않음 |
| 판 경계 | 실제 판 외곽 + 실행 중인 CoP 제어기의 안전 경계 | 상자의 실제 외형 |
| 힘·모멘트 | MoB 추정 렌치와 Solver 접촉 렌치 | 두 드론의 접촉 합력/모멘트와 각 면의 GT 렌치 |
| solver 접촉점 | 기존 contact_diagnostics 입력이 있을 때 선택적으로 표시 | 두 드론 판에서 표시 |

**업데이트한 원본 HandinHand_sim의 기존 `cooperation_plant`가 payload odometry와
World acceleration을 기본 30 Hz로 발행합니다.** 이 패키지는 그 출력을 구독하여 payload의
실제 3D 형상·위치·자세·선속도·선가속도·각속도·이동 궤적과 기존 접촉 렌치를 표시합니다.
두 드론 사이의 중간점으로 상태를 가정하지 않습니다. 같은 MuJoCo 데이터의 실제 관측값입니다.
RViz는 plant를 대체하거나 추가 실행하지 않습니다.

원본 simulator는 [payload telemetry 설명](https://github.com/INTGER-PARK/HandinHand_sim/blob/main/README_PAYLOAD_RVIZ_KR.md)에
따라 업데이트·재빌드하고, 기존 plant를 다시 시작해야 합니다. 구버전 plant 또는 telemetry OFF
상태에서는 payload만 `waiting/stale`입니다. 이번 출력 추가는 drone GT/joint_states와
contact_diagnostics 토픽을 제공하지 않습니다. 드론은 기본 `measured` 모드로 표시합니다.

기체와 payload 형상은 **설치된 `hih_plant/xml/Cooperation.xml`을 직접 읽어 재사용**합니다.
MuJoCo가 `Leader.xml` / `Follower.xml` / `contact_box.xml`의 include·재질·`fromto`·관절을
해석한 결과에서 프레임, 다리, 접촉판, 배터리, FC, 서보 케이스, 모터, 프로펠러 날개를 가져옵니다.
RViz 전용으로 따로 정한 기체 치수나 임의의 rotor 배치를 사용하지 않습니다.
각 geom의 실제 크기·Body 상대 위치/회전·RGBA를 유지합니다. 두 드론의 구분 색상은
궤적·축·라벨에 사용하며, 기체 자체는 XML의 원래 재질 색상을 사용합니다.

`state.servo`의 degrees를 radians로 변환해 네 서보 관절을 움직이며, GT 모드에서는
외부 `/hih/ground_truth/{role}/joint_states` 입력의 실제 joint position [rad]을 사용합니다.
이 값이 없거나 stale이면 서보 이하의 움직이는 부품을 숨기고 상태를 표시합니다.
알 수 없는 관절 자세를 0도라고 가정하지 않습니다.

## 2. 빌드

먼저 기존 HandinHand_sim을 빌드하고, 그 시뮬레이터가 실행되는 Python/ROS 환경을 사용합니다.
`hih_interfaces`를 이 저장소에 복제하거나 메시지 정의를 다시 작성할 필요는 없습니다.
이 문서의 기존 workspace 경로는 본인의 경로에 맞게 변경하세요.

Payload 출력을 포함하도록 원본 simulator를 먼저 업데이트합니다.

```bash
source /opt/ros/humble/setup.bash
sudo apt install ros-humble-nav-msgs ros-humble-geometry-msgs
cd ~/ros2_project/HandinHand_sim
git pull --ff-only
colcon build --symlink-install --packages-select hih_plant
source install/setup.bash
```

그다음 별도 visualization workspace를 빌드합니다.

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_project/HandinHand_sim/install/setup.bash

sudo apt update
sudo apt install ros-humble-rviz2 ros-humble-tf2-ros \
  ros-humble-nav-msgs ros-humble-geometry-msgs ros-humble-sensor-msgs ros-humble-visualization-msgs \
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
형상 로딩에는 기존 simulator의 MuJoCo Python 모듈과 `hih_plant` XML이 필요합니다.
검증 버전은 MuJoCo 3.3.7입니다. 시뮬레이터가 이미 실행되는 동일 Python 환경을 사용하세요.
시뮬레이터와 다른 컴퓨터라면 ROS_DOMAIN_ID, ROS_LOCALHOST_ONLY 및 DDS 네트워크 설정도 맞춥니다.

## 3. 실행: 기존 시뮬레이터에 붙이기

HandinHand_sim의 기존 launch와 실험 설정을 그대로 사용합니다.
새 터미널에서 기존 simulator와 visualization workspace를 source한 뒤 실행합니다.

```bash
ros2 launch hih_rviz visualization.launch.py
```

실행되는 노드는 `hih_visualizer`와 `rviz2`뿐입니다. 기존 MuJoCo viewer를 함께 켜도 됩니다.
RViz 종료/재시작은 시각화 노드만 종료/재시작하며 plant와 제어 노드는 그대로 실행됩니다.
Payload는 원본 `/hih/ground_truth/payload/odometry`와 `/hih/ground_truth/payload/acceleration`에
기본 연결되므로 별도 인자 없이 표시됩니다. `drone_state_source`는 `measured`로 유지하면 됩니다.

| launch 인자 | 기본값 | 의미 |
| --- | --- | --- |
| `rviz` | true | RViz 창 실행; false이면 표시용 토픽만 발행 |
| `config` | 기본 visualization.yaml | 시각화 설정 파일 |
| `use_sim_time` | false | rosbag 등의 /clock 사용 여부 |
| `drone_state_source` | YAML 설정(measured) | 기존 state 또는 이미 제공되는 ground_truth 입력 선택 |
| `acceleration_mode` | YAML 설정(body_specific_force) | 기존 state.acc의 해석 |

센서 모델을 꺼서 `state.acc`가 World 선가속도로 발행되는 실험에서는:

```bash
ros2 launch hih_rviz visualization.launch.py acceleration_mode:=world_linear
```

외부에서 동일한 실제 시뮬레이션의 GT 토픽을 이미 제공하는 경우에만:

```bash
ros2 launch hih_rviz visualization.launch.py drone_state_source:=ground_truth
```

측정 상태 모드에서는 드론의 noise/delay가 포함된 pose로 해당 드론의 CoP/렌치를 그립니다.
정밀한 GT geometry 비교에는 `ground_truth` 모드를 사용하세요.

## 4. 시각화가 제어에 개입하지 않는지 확인

`cmd`/`cmd_admittance`는 **구독만** 하며, 받은 위치를 표시용 Path로 변환합니다.
표시 설정·화살표 길이 제한·이력 삭제는 시각화 내부에만 적용됩니다.
안전 경계는 CoP 제어 노드의 `/get_parameters` 서비스로 `safe_y_m`/`safe_z_m`만 조회합니다.
`SetParameters` client나 enable/명령/actuator publisher를 만들지 않습니다.

실행 후 실제 ROS graph에서도 확인할 수 있습니다.

```bash
ros2 node info /hih_rviz
ros2 topic info /follower/minitrone/cmd --verbose
ros2 topic info /follower/minitrone/cmd_admittance --verbose
ros2 topic info /leader/minitrone/cmd --verbose
```

위 명령에서 `hih_rviz`는 command의 **SUBSCRIPTION** 항목에만 나타나야 합니다.
시각화 출력은 `/hih_viz/*`와 자체 `hih_viz/...` frame의 `/tf`입니다.
ROS 자체의 `/rosout`·`/parameter_events`도 있을 수 있으며 제어 입력은 아닙니다.
기본 RViz Tools는 Interact/MoveCamera/Select이며 goal/pose 명령 도구는 넣지 않았습니다.
ROS graph와 렌더링에 필요한 CPU·메모리·통신 비용은 추가됩니다.

## 5. 좌표계·단위·힘의 의미

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

선택 GT 가속도 입력은 World 성분의 `geometry_msgs/AccelStamped`입니다.
시각화 노드가 물리 시뮬레이션의 qacc나 새 센서를 생성하지 않습니다.
Payload 입력은 원본 plant의 Body 원점 `data.xpos`와 `data.xquat`입니다.
Odometry의 pose는 World, twist는 payload Body 성분이며, 모든 축의 자세를 반영합니다.
원본 plant의 가속도는 World 속도의 simulation-time 차분으로, 첫 샘플·정지·reset·긴 공백에서는
발행하지 않습니다. IMU specific force가 아니므로 payload GT 가속도에 중력을 다시 더하지 않습니다.

## 6. 화면과 설정 변경

`config/visualization.yaml`은 데이터 생성·좌표계·화살표 배율·이력 길이의 설정 파일입니다.
시작 시 화면에 보일 항목은 **`rviz/handinhand.rviz`의 Displays와 Namespaces**에서 설정합니다.
드론의 실제 부품 치수/배치는 simulator XML이 원본이며 YAML에서 별도로 재정의하지 않습니다.
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

기본 화면에는 **Leader / Follower / Payload 모델, TF, 궤적만** 표시합니다.

| 항목 | 시작 시 표시 |
| --- | --- |
| `leader/model`, `follower/model`, `payload/model` | ON |
| TF (`Body and contact frames`) | ON; frame 이름 문자는 OFF |
| 세 물체의 actual Path, Leader/Follower reference Path, Follower admittance Path | ON |
| 속도·가속도·각속도, 추정/접촉 렌치, CoP, 판/안전 경계, 접촉점 | OFF |
| 상태/수치/legend 문자, COM 구·십자·문자, World grid | OFF |

직접 켜려면 RViz 왼쪽 **Displays → Bodies - vectors - CoP - contact → Namespaces**를 펼쳐
원하는 namespace만 체크하세요. 예를 들어 `leader/cop_mob`, `follower/contact_force`,
`payload/com`을 개별로 선택할 수 있습니다. TF, Path와 grid는 각각의 Display 체크를 사용합니다.
늦게 데이터가 들어오는 CoP·COM·접촉점도 미리 저장된 기본 OFF 설정을 따릅니다.
해당 데이터가 아직 없거나 invalid이면 namespace/마커는 데이터가 유효해질 때 나타납니다.
업데이트한 기본 체크 상태를 적용하려면 **RViz 창도 다시 실행**하세요.
시각화 노드만 재시작하면 이미 열려 있는 RViz 창의 수동 체크 상태는 유지됩니다.

수동 체크로 바로 켤 수 있도록 기본 YAML의 `show_*` 플래그는 마커 **발행을 허용**합니다.
특히 `show_labels: true`와 `show_contact_points: true`여도 기본 화면에서는 namespace 체크가 OFF입니다.
사용자 YAML에서 이 값을 false로 바꾸면 해당 마커 발행이 중단되므로 체크만으로 표시할 수 없습니다.
Fixed Frame은 기본 `world`입니다. Body frame으로 바꾸면 궤적을 해석하기 어려워집니다.
상태 문자는 `*/state`, CoP 문자는 `*/cop_*_text`, 렌치 수치는 `*/*_wrench`,
상단 상태/색상 설명은 `legend`를 체크하면 표시됩니다. `/hih_viz/status`로도 상태를 확인할 수 있습니다.

Payload의 실제 **COM 마커는 `hih_payload_disturbance` 노드가 실행 중일 때만** 생성됩니다.
기본 표시 체크는 OFF이므로 **`payload/com`과 `payload/com_cross`를 직접 켜면**
자홍색 구와 십자를 볼 수 있습니다. `C` 입력 전이나 COM 이동을 일시정지한 상태에서도
노드가 실행 중이면 마커가 유지되며, wind 노드만 실행한 경우에는 생성되지 않습니다.
내부 COM을 볼 수 있도록 해당 노드의 heartbeat가 유효한 동안 payload 모델 alpha를 기본 0.25로 낮춥니다.
노드 종료 신호 또는 heartbeat/COM 위치의 수신 만료 시 마커를 지우고 원래 alpha를 복구합니다.
강제 종료나 통신 단절 시 기본 최대 약 0.7초(`feedback_timeout_s`) 후 숨겨집니다.

이 기능에는 업데이트한 **HandinHand_sim과 HandinHand_rviz 둘 다** 필요합니다.
Simulator의 `hih_plant`와 이 workspace의 `hih_rviz`를 각각 rebuild/source하고 재실행하세요.
실험 노드 실행법은 simulator의
[README_DISTURBANCES_KR.md](https://github.com/INTGER-PARK/HandinHand_sim/blob/main/README_DISTURBANCES_KR.md)를 참고하세요.
RViz에서 별도로 PointStamped display를 추가할 필요가 없습니다.

COM 구는 실제 `/hih/ground_truth/payload/com`의 World 위치를 사용합니다.
Body 중심이나 명령 offset으로 COM을 추정하지 않습니다.
`show_payload_com: true`는 fresh COM과 실행 상태 신호가 있을 때만 표시를 허용한다는 뜻입니다.
아예 숨기려면 false, 구 크기는 `payload_com_diameter_m`, 표시 중 상자 투명도는
`payload_com_model_alpha`로 설정하세요. COM 문자는 `payload/com_text`를 직접 체크합니다.
모든 설정은 표시용이며 시뮬레이터의 물리 파라미터나 COM 명령에는 영향을 주지 않습니다.

`model_xml_path`가 빈 문자열이면 source한 `hih_plant`의 `xml/Cooperation.xml`을 사용합니다.
다른 MJCF를 사용하려면 이 값에 절대 경로를 지정하세요. 현재 body/joint 이름 규약은 유지해야 합니다.
`model_geom_groups: [0, 1, 2]`는 MuJoCo viewer의 기본 표시 그룹과 같습니다.
숨겨진 geom 그룹도 보려면 `[0, 1, 2, 3, 4, 5]`로 변경합니다.

형상 변환용 별도 MjData에서는 `mj_kinematics`만 사용하며 물리 시간을 진행시키지 않습니다.
Box / cylinder / sphere / ellipsoid를 RViz primitive로 옮기고, capsule은 같은 크기의
원기둥과 양 끝 구로 표현합니다. 현재 원본 두 드론에 있는 모든 geom 형식을 지원합니다.
새로 추가한 mesh/heightfield 등 미지원 형식은 경고에 이름을 남기며 임의의 상자로 대체하지 않습니다.
RGBA는 MuJoCo의 native scene에서 해석한 값을 사용하지만 RViz와 MuJoCo의 조명·반사·그림자
렌더러가 다르므로 화면의 음영까지 픽셀 단위로 같지는 않습니다.
XML을 수정했다면 `hih_plant`를 rebuild/source하고 visualizer를 재시작하면 새 형상이 반영됩니다.
CoP/렌치 overlay의 원점·판 크기 파라미터는 observer의 좌표계와 별도로 일치시켜야 합니다.

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

## 7. 토픽

`{role}`은 `leader` 또는 `follower`, `{body}`는 `leader/follower/payload`입니다.

| 구독 입력 | 타입 | 필수 여부 |
| --- | --- | --- |
| `/{role}/minitrone/state` | hih_interfaces/MinitroneState | measured 드론 표시 |
| `/hih/ground_truth/payload/odometry` | nav_msgs/Odometry | 업데이트한 원본 plant의 payload pose/twist |
| `/hih/ground_truth/payload/acceleration` | geometry_msgs/AccelStamped | 업데이트한 원본 plant의 payload World 가속도 |
| `/hih/ground_truth/payload/com` | geometry_msgs/PointStamped | 실제 COM World 위치, COM 표시용 선택 입력 |
| `/hih/disturbance/payload/com_visualization_enabled` | std_msgs/Bool | payload disturbance 노드의 실행 heartbeat, true일 때만 COM 표시 |
| `/hih/ground_truth/{role}/odometry` | nav_msgs/Odometry | drone GT 모드일 때 외부 입력 필요 |
| `/hih/ground_truth/{role}/acceleration` | geometry_msgs/AccelStamped | 선택 drone GT 가속도; 외부 입력 필요 |
| `/hih/ground_truth/{role}/joint_states` | sensor_msgs/JointState | GT 모드의 네 servo position [rad] |
| `/{role}/minitrone/cmd` | hih_interfaces/Cmd | upstream 경로, 선택 |
| `/{role}/minitrone/cmd_admittance` | hih_interfaces/Cmd | admittance 출력 경로, 선택 |
| `/{role}/minitrone/cop_hat` | hih_interfaces/CenterOfPressure | MoB CoP, 선택 |
| `/{role}/minitrone/cop_solver_real` | hih_interfaces/CenterOfPressure | Solver CoP, 선택 |
| `/{role}/minitrone/external_wrench_hat_second_order` | hih_interfaces/Wrench | 추정 렌치, 선택 |
| `/{role}/minitrone/contact_wrench_real` | hih_interfaces/Wrench | Solver 렌치, 선택 |
| `/{role}/minitrone/contact_diagnostics` | hih_interfaces/ContactDiagnostics | 선택 접촉점; 외부 입력 필요 |
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

이 패키지는 위 구독 입력을 발행하지 않습니다. Payload GT는 원본 plant가 출력하며,
선택 drone GT/contact diagnostics를 제공하는 publisher가 없으면 해당 항목은 표시하지 않습니다.
이 표의 경로는 미래의 전체 planned trajectory가 아니라 지금까지 발행된 명령의 이력입니다.
controller 내부에서 합산되는 모든 CoP/attitude offset을 포함한 최종 desired pose라고 가정하지 않습니다.

원래 custom 메시지에는 Header가 없으므로 수신 시각으로 stamp를 추가합니다.
상태/렌치/CoP의 엄밀한 sample synchronization이나 latency 보정은 수행하지 않습니다.
표시용 odometry covariance는 새로 추정하지 않으며 기본 0 배열에 불확실성 평가 의미를 부여하지 않습니다.
같은 ROS 시간 기준을 사용해야 합니다. 기본 simulator는 /clock을 발행하지 않으므로
일반 실행은 `use_sim_time=false`를 사용합니다.

## 8. rosbag 재생

원본 headerless 입력과 원본 plant의 payload GT를 함께 기록합니다.
작은 예시는 다음과 같습니다.

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

## 9. 문제가 있을 때

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
| payload만 없음 | 원본 simulator 업데이트·hih_plant 재빌드·plant 재시작 및 enable_payload_telemetry=true 확인 |
| CoP 점 없음 | 해당 source 발행, valid=true, normal_force>0 확인 |
| 안전 사각형 없음 | CoP 노드 실행/이름/parameter service 확인; manual 기본은 숨김 |
| 정지인데 가속도가 약 9.81 | acceleration_mode가 sensor model 모드와 맞는지 확인 |
| GT 가속도 없음 | 외부 World AccelStamped publisher 또는 설정한 입력 토픽 확인 |
| 과거 화살표가 없어짐 | stale timeout 또는 invalid 데이터에 따른 삭제가 정상 |
| rviz2 package 없음 | ros-humble-rviz2 설치 또는 `rviz:=false`로 데이터 출력만 실행 |
| `MJCF model unavailable` | `hih_plant` source, MuJoCo 모듈, `model_xml_path` 확인; 상태 overlay는 계속 동작 |
| 모터/서보만 없음 | state.servo 또는 GT joint_states 수신/stale 여부 확인 |

geometry/topic/frame이 원본 simulator에서 바뀌면 YAML도 맞추세요. `world_frame` 이름은
RViz Fixed Frame과 외부 GT 입력의 frame_id에도 일치해야 합니다.
이름 변경 자체가 좌표 회전을 수행하는 것은 아닙니다.

## 10. 검증과 제한

ROS 없는 환경에서 실행 가능한 검사:

```bash
python3 -m pip install pytest numpy PyYAML mujoco==3.3.7
python3 -m pytest -q test
```

검사는 frame math, facing-drone CoP 부호, tilted hover의 specific-force 변환,
bounded history/reset, 실제 visualizer callback의 invalid/stale 삭제와 메시지 원점,
조회 전용 safety query를 확인합니다. 전체 출력 토픽과 서비스 타입, launch/entry point를
검사하여 제어 입력 발행 및 plant/제어기 실행 경로가 없는지도 확인합니다.
설치된 `plant.payload_telemetry`가 있으면 실제 payload publisher helper의 메시지를 받아
3D 상자 모델·TF·Path·속도·가속도까지 이어지는 연결도 검사합니다. 두 패키지의 소스가 같은
Python 경로에 있거나 simulator workspace를 source해야 이 연결 검사를 실행합니다.
MJCF 형상 검사는 native material/group 선택, capsule 끝점, 실제 servo FK, World 자세 결합과
sensor noise 아래 geometry cache의 메모리 제한을 확인합니다. 원본 Cooperation.xml 기준으로도
드론별 visible geom 48개(각각 RViz primitive 52개)의 위치·회전·크기 284개 조건을 확인했습니다.
ROS transport 대체물은 `test/ros_stubs.py` 안에서만 사용합니다.
MuJoCo가 없으면 그 라이브러리를 사용하는 검사만 skip합니다.

개발 환경에서는 ROS 2/DDS 및 RViz GUI가 설치되어 있지 않아 전체 colcon build,
실제 simulator와의 ROS topic 연결 및 RViz 렌더링은 아직 실행 검증하지 못했습니다.
배포 후 3번 시각화 launch와 4번 ROS graph 조회로 화면/연결을 확인하세요.

## 11. 저장소와 업데이트

이 패키지는 https://github.com/INTGER-PARK/HandinHand_rviz 에서 별도로 관리합니다.
기존 `HandinHand_sim`과 독립된 저장소이며, simulator 설정 변경을 자동으로 가져오지 않습니다.

```bash
cd ~/ros2_project/hih_visualization_ws/src/HandinHand_rviz
git pull --ff-only
cd ~/ros2_project/hih_visualization_ws
# 0.1.0을 설치한 적이 있다면 별도 visualization workspace의 기존 설치 산출물을
# 먼저 제거합니다. 아래 두 경로는 기본 isolated install 기준이며 소스 폴더가 아닙니다.
rm -rf build/hih_rviz install/hih_rviz
colcon build --symlink-install --packages-select hih_rviz
source install/setup.bash
```

시뮬레이터 메시지·XML·좌표계가 변경되면 이 패키지의 입력 규약과 YAML도 확인하세요.
Payload 표시를 추가하려면 원본 simulator도 2번 안내대로 업데이트·재빌드하고 plant를 재시작하세요.
기존 `simulation.launch.py`/`demo.launch.py`를 실행 중이었다면 해당 프로세스를 종료하고,
원래 HandinHand_sim launch를 실행한 뒤 `visualization.launch.py`만 추가로 실행하세요.

## 참고

- ROS Marker 형식: https://github.com/ros2/common_interfaces/blob/humble/visualization_msgs/msg/Marker.msg
- ROS Odometry 좌표 규약: https://github.com/ros2/common_interfaces/blob/humble/nav_msgs/msg/Odometry.msg
- MuJoCo kinematics: https://mujoco.readthedocs.io/en/3.3.7/APIreference/APIfunctions.html#mj-kinematics
