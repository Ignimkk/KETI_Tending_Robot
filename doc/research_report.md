# Tending 로봇 비전 검사 시스템 — 연구·개발 상세 보고서 (Research Report)

> 대상: 엔드밀(End-Mill) 측면 360° 비전 검사용 TM5 로봇 제어 시스템
> 미들웨어: ROS 2 Humble / 로봇: Techman TM5-900 / 핵심 드라이버: `tmr_ros2`
> 작성 범위: 시스템 SETUP, 개발 계획, 구현 패키지·모듈(기능별), 시나리오별 제어 방식, CLI/코드, 검증 결과, 향후 계획
> 본 문서는 `PROJECT_PLAN.md`(계획 정본)를 보완하는 **구현 상세 보고서**이다.

---

## 목차

1. 문서 개요와 목적
2. 프로젝트 배경과 과제 정의
3. 아키텍처 핵심 원칙 — `tmr_ros2`는 코어, `tending_*`는 보충
4. 우리가 세운 계획 (의사결정 · 우선순위)
5. 개발 환경 및 SETUP 상세 절차 (CLI 포함)
6. `tmr_ros2` 통신 구조 심층 분석
7. 좌표계 · 단위 규약
8. 구현 패키지 및 모듈 — 기능별 상세
9. 시나리오별 제어 방식 상세
10. 검사 파이프라인 (MVP)
11. 수동 티칭 유틸리티와 워크플로
12. 무하드웨어 검증 방법과 결과
13. 안전 설계
14. 실행 레시피 모음 (CLI Cookbook)
15. 향후 계획 (P1 / P2 / 다중 CNC)
16. 부록 — 파라미터 레퍼런스 · 파일 인덱스 · 트러블슈팅

---

## 1. 문서 개요와 목적

본 보고서는 "TM5 로봇 기반 엔드밀 측면 비전 검사 시스템"의 로봇 제어·인터페이싱 계층을 어떻게 설계·구현했는지를 상세히 기록한다. 특히 다음을 다룬다.

- **SETUP**: 아무것도 없는 상태에서 이 시스템을 빌드·구동하기까지의 전 과정.
- **계획**: 과제를 수행하기 위해 세운 의사결정과 개발 우선순위.
- **구현물**: 우리가 새로 만든 ROS 2 패키지와 모듈을 기능 단위로 분해해 설명.
- **제어 방식**: 시나리오 1(손목 장착)에서 임의 좌표의 툴을 검사하기 위한 3가지 제어 방식과 그 트레이드오프.

카메라는 아직 미부착 상태이므로 본 단계의 초점은 **로봇 단독 제어·인터페이싱**과 **무하드웨어 검증 체계**이며, 카메라 관련 요소는 인터페이스/스텁으로만 준비되어 있다.

---

## 2. 프로젝트 배경과 과제 정의

### 2.1 물리적 구성

바닥에 고정된 **TM5 로봇**이 상부 CNC 척에 매달린 **엔드밀**에 아래에서 접근한다. 엔드밀의 스핀들 축(수직, 아래로 매달림)을 기준으로 측면 360°를 촬영해 플루트 형상·절삭날 마모·치핑·스크래치 등 결함을 검사한다.

### 2.2 두 가지 카메라 장착 시나리오

- **시나리오 1 (손목 직접 장착, 90°)**: 카메라를 플랜지에 직결하고 손목을 ~90° 틀어 광축이 엔드밀 측면을 향하게 한다. 360° 커버를 위해 **로봇이 툴 축을 중심으로 궤도(orbit) 이동**한다. → 본 단계의 우선 구현 대상.
- **시나리오 2 (L-브라켓)**: 측방 오프셋 브래킷 끝에 카메라를 달고 EEF 축을 스핀들 축과 정렬한 뒤 **J6만 회전**해 스윕한다. → 후속.

### 2.3 3가지 로봇 제어 방식(요구)

1. ROS 2 MoveIt 2 기반 경로계획
2. 규칙기반/하드코딩
3. 다중 CNC 재사용 제어 방식(선정 대상)

과제는 이 세 방식을 구현·비교하고, **1대 → 3대 CNC로 확장 가능한 재사용 제어 방식**을 도출하는 것이다.

### 2.4 확정된 프로젝트 결정

| 항목 | 결정 |
|---|---|
| 카메라 | 외부 산업용(USB3/GigE) + 링라이트 (부착 예정, 현재 stub) |
| 신규 코드 언어 | C++ (rclcpp) |
| MVP 우선 시나리오 | 시나리오 1 (손목 직접 장착) |
| 시뮬레이션 | 최소한, Gazebo 미사용, 실물/공식 가상로봇 위주 |
| 외부 GUI | C# `TendingSystem` 브리지 (후순위 P2) |

---

## 3. 아키텍처 핵심 원칙 — `tmr_ros2`는 코어, `tending_*`는 보충

프로젝트의 최상위 설계 원칙이다.

- **`tmr_ros2` = 코어.** 로봇 **bringup**, **MoveIt 2**, **실물 연동**은 `tmr_ros2`가 제공하는 공식 런치를 **그대로** 사용한다. 이를 재구현하지 않는다.
  - 드라이버: `tm_driver/launch/tm_bringup.launch.py`
  - MoveIt + RViz: `tm5-900_moveit_config/launch/tm5-900_run_move_group.launch.py`
- **`tending_*` = 보충.** `tmr_ros2`에 없는 것만 만든다: 검사 제어 로직, 검사 인터페이스(msg/srv/action), 카메라, 데이터 기록, 그리고 이들을 잇는 글루.
- **`tmr_ros2` 내부 파일은 생성/수정하지 않는다.** 서비스/토픽/액션/xacro/MoveIt config는 의존성으로만 참조한다.
- **저수준은 상위에 의존하지 않는다.** (bringup 계층이 control 계층에 의존 금지)
- **Gazebo 미사용.** 무하드웨어 로봇은 `tmr_ros2` 내장 가상(fake) 로봇을 쓴다(§6.4).

이 원칙에 따라, 초기에 만들었던 자체 `fake_tm_driver`/`display.launch.py`는 제거하고 `tmr_ros2`의 가상 로봇으로 대체했다.

---

## 4. 우리가 세운 계획 (의사결정 · 우선순위)

### 4.1 접근 전략

과제의 요구를 다음 순서로 분해했다.

1. **워크스페이스·`tmr_ros2` 구조 분석** → 재사용 가능한 서비스/토픽/액션/MoveIt config/description 확정.
2. **통신 인터페이스 확정** → TMSCT(명령)/TMSVR(피드백) 채널, `set_positions`/`send_script`/`feedback_states` 파악.
3. **보충 계층 설계** → 검사 인터페이스 → 제어(rule/moveit/hybrid) → 카메라/데이터 → 티칭 유틸 순으로 축조.
4. **무하드웨어 검증 체계** → 오프라인 dry_run + 단위테스트 + `tmr_ros2` 가상 로봇.

### 4.2 개발 우선순위 (P0 / P1 / P2)

- **P0 (지금, 카메라 불필요)**: 로봇 통신·상태, 좌표/TCP, 시나리오1 3가지 제어 방식, 안전, 데이터 동기 파이프라인 구조, 티칭 유틸. 카메라는 stub.
- **P1 (카메라 부착 후, ≈2026-08-10)**: 카메라 실드라이버, 핸드아이 캘리브레이션, 실이미지 촬영·단일 CNC 실험, 3대 확장, 시나리오 비교.
- **P2 (후순위)**: C# `TendingSystem` 브리지.

### 4.3 3번째 제어 방식에 대한 결론

**태스크프레임 기반 재사용 모션 + 사용자 티칭 기준점 + 기계별 YAML 설정(하이브리드)** 을 3번째 방식으로 채택했다. 이유는 §9.4에서 상술한다. 요지는 "검사 궤적이 본질적으로 툴 축(task frame) 기준의 원/등각 스윕이므로, 축 프레임만 바뀌면 동일 파라메트릭 궤적을 재사용 → 3대 확장 시 코드 무변경, YAML+티칭만 교체" 이다.

---

## 5. 개발 환경 및 SETUP 상세 절차

> 이 절만 따라 하면 빈 우분투에서 본 시스템을 빌드·구동할 수 있도록 구성했다.

### 5.1 전제 환경

- Ubuntu 22.04
- ROS 2 **Humble**
- 빌드: `colcon`, RMW: **Cyclone DDS** 권장
- 로봇: TM5-900 (실물) 또는 `tmr_ros2` 내장 가상 로봇

### 5.2 ROS 2 및 의존 패키지 설치

```bash
# ROS 2 Humble (데스크톱)
sudo apt update
sudo apt install -y ros-humble-desktop

# 빌드/개발 도구
sudo apt install -y python3-colcon-common-extensions python3-rosdep build-essential

# MoveIt 2 + 컨트롤러 + 계획기 (tmr_ros2 MoveIt 사용)
sudo apt install -y \
  ros-humble-moveit \
  ros-humble-controller-manager \
  ros-humble-joint-trajectory-controller \
  ros-humble-joint-state-broadcaster \
  ros-humble-chomp-motion-planner \
  ros-humble-pilz-industrial-motion-planner

# Cyclone DDS (권장 RMW)
sudo apt install -y ros-humble-rmw-cyclonedds-cpp
echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc

# tf2 / eigen (tending 유틸이 사용)
sudo apt install -y ros-humble-tf2-ros ros-humble-tf2-eigen ros-humble-tf2-geometry-msgs
```

### 5.3 워크스페이스 구성

```bash
# 워크스페이스 (이미 존재)
cd ~/dev_ws/prject/NCC/Tending_ws
ls src
#  tmr_ros2/          ← 코어 드라이버 스택 (Techman 공식, humble 브랜치)
#  tending_*/         ← 우리가 만든 보충 패키지들
#  doc/  command.txt  ← 문서/명령 모음
```

`src/` 구성 요약:

```
src/
├── tmr_ros2/                  # 코어 (수정 금지)
│   ├── tm_driver/             # 드라이버 노드 (TMSCT/TMSVR)
│   ├── tm_msgs/               # TM 인터페이스 (SetPositions, SendScript, FeedbackState...)
│   ├── tm_description/        # URDF/xacro/meshes
│   ├── tm_moveit/tm5-900_moveit_config/   # MoveIt 2 config
│   └── ...
├── tending_interfaces/        # 검사용 msg/srv/action
├── tending_control/           # 검사 제어 (rule/hybrid) + 상태기계 + 티칭 유틸 + 검사 launch/config
├── tending_moveit/            # 시나리오1 MoveIt 제어 (충돌회피)
├── tending_camera/            # 카메라(stub) capture_image
├── tending_data/              # 검사 데이터 저장(recorder)
├── tending_bringup/           # (골격) 카메라 부착 셀 bringup 예정
├── tending_description/       # (스캐폴드) 카메라/브라켓/CNC 확장 description
├── tending_calibration/       # (스캐폴드 P1) 핸드아이 캘리브
├── tending_bridge/            # (스캐폴드 P2) C# TendingSystem 브리지
├── doc/{PROJECT_PLAN.md, README.md, research_report.md}
└── command.txt
```

### 5.4 rosdep + 빌드

```bash
cd ~/dev_ws/prject/NCC/Tending_ws
sudo rosdep init 2>/dev/null; rosdep update
rosdep install --from-paths src --ignore-src -r -y

source /opt/ros/humble/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

빌드 순서(의존성): `tending_interfaces` → `tending_control` → `tending_moveit`/`tending_camera`/`tending_data` → (기타). `tending_bringup`은 `tending_control`에 의존하지 않는 **저수준**으로 유지된다.

### 5.5 TMflow(로봇 컨트롤러) 준비 — 실물 사용 시

`tm_driver`가 로봇에 연결되려면 TMflow에서 다음이 활성화돼야 한다.

1. **Ethernet Slave**(TMSVR, 포트 5891) 활성 — 상태 피드백.
2. **Listen Node**(TMSCT, 포트 5890)를 포함한 프로젝트를 작성하고 **실행(Play)** 상태로 둔다 — 외부 스크립트 명령 수신.
3. 로봇 IP 확인(예: `192.168.0.10`), 검사 PC와 동일 서브넷·방화벽 허용.

### 5.6 단위테스트 (로봇 불필요)

```bash
colcon test --packages-select tending_control
colcon test-result --all --verbose
# pose_utils gtest 3종: 검사거리/look-at, TM 왕복(round-trip), 등각 분할
```

### 5.7 SETUP 검증 체크리스트

```bash
# 가상 로봇 + MoveIt + RViz 기동 (robot_ip 생략 = 내장 fake)
ros2 launch tm5-900_moveit_config tm5-900_run_move_group.launch.py

# 다른 터미널: 상태/서비스 확인
ros2 topic list | grep -E 'feedback_states|joint_states|tool_pose'
ros2 service list | grep -E 'set_positions|send_script|set_event'
ros2 node list        # /move_group, /tm_driver_node, /robot_state_publisher ...
```

정상이면 RViz에 TM5-900이 표시되고 위 토픽/서비스가 노출된다.

---

## 6. `tmr_ros2` 통신 구조 심층 분석

### 6.1 두 개의 소켓 채널

| 채널 | 역할 | 포트 | 클래스 |
|---|---|---|---|
| **TMSCT (Listen Node)** | 명령(스크립트/모션/이벤트/IO) | 5890 | `TmSctRos2` |
| **TMSVR (Ethernet Slave)** | 상태 피드백(조인트/포즈/에러) | 5891 | `TmSvrRos2` |

두 채널은 하나의 노드(`tm_driver_node`)에 컴포지션된다.

### 6.2 우리가 사용하는 서비스 (명령)

- `set_positions` (`tm_msgs/srv/SetPositions`): 구조화된 모션.
  - `motion_type`: `PTP_J=1`(관절), `PTP_T=2`(Cartesian PTP), `LINE_T=4`(Cartesian 직선), `CIRC_T=6`, `PLINE_T=8`
  - `positions`: 관절이면 `[j1..j6]`(rad), Cartesian이면 `[x,y,z,rx,ry,rz]` **(m, rad)** — 드라이버가 내부에서 mm/deg로 변환
  - `velocity`(관절 rad/s ≤ π, 또는 Cartesian m/s), `acc_time`(ms), `blend_percentage`, `fine_goal`
- `send_script` (`SendScript`): 임의 TM 스크립트 문자열 전송(`PTP(...)`, `Line(...)`, `ChangeTCP(...)`, `QueueTag(...)` 등)
- `set_event` (`SetEvent`): `TAG/WAIT_TAG/STOP(=11)/PAUSE/RESUME/EXIT` — 동기화·정지
- `set_io` (`SetIO`): 컨트롤박스/엔드이펙터 DO/AO (링라이트 트리거 후보)

### 6.3 우리가 사용하는 상태 (피드백)

- 토픽 `feedback_states` (`tm_msgs/msg/FeedbackState`): `joint_pos/vel/tor`, `tool0_pose`(플랜지), `tool_pose`(TCP), 연결 플래그(`is_svr_connected/is_sct_connected`), 안전 플래그(`e_stop/safetyguard_a/robot_error`), `error_code/error_content`
- 토픽 `joint_states` (`sensor_msgs/JointState`) — RViz/RSP FK 입력
- 토픽 `tool_pose` (`geometry_msgs/PoseStamped`) — base 기준 TCP
- 액션 `follow_joint_trajectory` (`control_msgs/action/FollowJointTrajectory`) — MoveIt 실행 경로

### 6.4 ★ 내장 가상(fake) 로봇

`tm_driver`의 MoveIt 컴포지션(`tm_ros2_composition_moveit.cpp`)은 실행 인자에 `robot_ip:=`가 없으면 **`is_fake=true`** 로 동작한다(로그 `"ip is not found, use fake robot"`). 즉:

```bash
# 가상 로봇 (실물 없이 RViz 동작 확인)
ros2 launch tm5-900_moveit_config tm5-900_run_move_group.launch.py
# 실물
ros2 launch tm5-900_moveit_config tm5-900_run_move_group.launch.py robot_ip:=192.168.0.10
```

이 가상 로봇은 `move_group`+RViz + `/set_positions`·`/send_script`·`/feedback_states`·`/joint_states` + `FollowJointTrajectory`를 모두 제공한다. `TmRos2SctMoveit : public TmSctRos2` 상속이므로 `set_positions`까지 포함된다. **우리 `tending_moveit` 검사 노드가 이 가상 로봇에서 계획·실행되어 RViz에서 로봇이 궤도를 도는 것을 확인**했다(§12). 이 덕분에 자체 fake는 불필요해 제거했다.

> 주의: 가상 로봇은 MoveIt(FollowJointTrajectory) 실행으로 관절이 움직인다. 반면 `set_positions`가 fake에서 관절을 실제로 구동하는지는 미검증이므로, 무하드웨어 시각화는 MoveIt 경로를 기준으로 한다. 또한 fake는 `tool_pose`(Cartesian 피드백)를 계산하지 않아(§12.3) EE 포즈는 TF에서 읽는다.

### 6.5 단위·회전 규약 (매우 중요)

- 위치: **meter**, 회전: **tf2 `setRPY(rx,ry,rz)` = Rz·Ry·Rx**
- `tool_pose` 토픽의 quaternion과 `SetPositions PTP_T`의 `[rx,ry,rz]`가 **동일 규약**
- 드라이버 `mmdeg_pose()`가 `[m,rad] → [mm,deg]` 변환을 담당하므로, 우리 코드는 항상 **m/rad**로 다룬다

---

## 7. 좌표계 · 단위 규약

### 7.1 프레임 트리(개념)

```
world ── base(=base_link) ──(6 관절)── flange(tool0) ── [TCP=카메라 광학, 캘리브 후]
              │
              └─ (검사 대상) cnc_frame ── endmill_axis(스핀들축) ── endmill_tip
```

- MoveIt 그룹 `tmr_arm`: 체인 `base → flange` (SRDF).
- rule 모드: 툴 축을 **base 기준**으로 직접 지정.
- hybrid 모드: 툴 축을 **cnc_frame(태스크 프레임) 기준**으로 지정 후 base로 변환.

### 7.2 변환 유틸 (pose_utils)

`tending_control/pose_utils`가 모든 변환의 단일 소스이다.

| 함수 | 역할 |
|---|---|
| `scenario1ViewPose(axis, d, θ, flip_up)` | 시나리오1 궤도 한 뷰의 base→TCP 변환 |
| `scenario1Orbit(...)`, `orbitAngles(...)` | 궤도 전체 뷰 목록/각도 |
| `toTmCartesian(T)` | Eigen → `[x,y,z,rx,ry,rz]`(m,rad, SetPositions용) |
| `toPoseMsg/fromPoseMsg` | Eigen ↔ `geometry_msgs::Pose` |
| `fromTmCartesian(xyzrpy)` | TM 6-value → Eigen (피드백 파싱/티칭) |
| `prettyPose(T)` | `"xyz=[..] m, rpy=[..] deg"` 사람용 출력 |

---

## 8. 구현 패키지 및 모듈 — 기능별 상세

우리가 만든 패키지는 모두 `src/` 아래 `tmr_ros2` **외부**에 있으며, 총 9개다. 기능군으로 묶으면:

- **인터페이스**: `tending_interfaces`
- **제어**: `tending_control`(rule/hybrid + 상태기계 + 유틸), `tending_moveit`(MoveIt)
- **센싱/데이터**: `tending_camera`, `tending_data`
- **브링업/기술**: `tending_bringup`(골격), `tending_description`(스캐폴드)
- **후속(스캐폴드)**: `tending_calibration`(P1), `tending_bridge`(P2)

### 8.1 `tending_interfaces` — 검사 인터페이스

`tmr_ros2`에 없는 "검사 오케스트레이션" 전용 타입만 최소로 정의한다.

- `action/RunInspection.action` — 검사 실행: goal(`machine_id, scenario, angle_start/end, num_views, inspect_distance`), result(`success, dataset_path, image_count, message`), feedback(`progress, captured, phase`)
- `srv/CaptureImage.srv` — 촬영 트리거: req(`label, view_index`) → res(`ok, image_path, stamp`)
- `srv/GoToNamedPose.srv` — 명명 포즈 이동: req(`pose_name, velocity`) → res(`ok, message`)
- `msg/InspectionSample.msg` — 촬영 순간 동기 샘플: `header, view_index, angle, joint_pos[], tcp_pose(Pose), image_path`

### 8.2 `tending_control` — 검사 제어의 중심

가장 큰 패키지. 라이브러리 + 3개 실행 노드 + 검사 launch/config를 소유한다.

#### 8.2.1 라이브러리 `tending_control_core`

- **`pose_utils`** (§7.2): 궤도 기하 + 좌표/단위 변환. 다운스트림(`tending_moveit` 등)에서 재사용하도록 export.
- **`robot_io_bridge`**: `tm_driver`를 감싼 얇은 C++ 클라이언트 계층. 상위가 TM 세부를 직접 다루지 않게 한다.

`RobotIoBridge` 공개 API:

```cpp
bool wait_for_services(timeout);
bool have_state();  bool connected();  bool safety_ok(std::string& reason);
std::vector<double> joint_pos();
geometry_msgs::msg::Pose tcp_pose();     // base→TCP (tool_pose 토픽)
geometry_msgs::msg::Pose flange_pose();  // base→flange (feedback tool0_pose)
// 모션 (도달까지 폴링 대기)
bool move_ptp_joint(q, vel, acc_ms, tol_rad, timeout);                 // PTP_J
bool move_ptp_tool (xyzrpy, vel, acc_ms, pos_tol, ang_tol, timeout);  // PTP_T
bool move_line_tool(xyzrpy, vel, acc_ms, pos_tol, ang_tol, timeout);  // LINE_T
bool send_script(id, script);
bool stop();                              // set_event STOP
```

핵심 설계 포인트:
- **도달 판정**: 관절 이동은 `feedback_states.joint_pos`를, Cartesian 이동은 `tool_pose`를 폴링해 목표 근방(위치/자세 허용오차)까지 대기.
- **안전 인터록**: 이동 폴링 루프마다 `safety_ok()`를 확인해 `e_stop/safetyguard/robot_error/통신단절` 시 즉시 `stop()`.
- **동시성**: 구독/서비스 클라이언트를 Reentrant 콜백그룹에 두고 상위 노드는 MultiThreadedExecutor로 스핀 → 액션/모션 스레드가 대기 중에도 상태 갱신.

Cartesian 도달 판정 예(회전 오차는 축각으로 계산):

```cpp
const Eigen::Isometry3d cur = fromPoseMsg(tcp_pose());
double dp = (cur.translation() - target.translation()).norm();
Eigen::AngleAxisd aa(cur.rotation().transpose() * target.rotation());
double da = std::fabs(aa.angle());
if (dp <= pos_tol_m && da <= ang_tol_rad) return true;   // 도달
```

#### 8.2.2 노드 `scenario1_inspect` — 시나리오1 rule/hybrid 제어

- `control_mode`(rule|hybrid)와 `dry_run`으로 동작을 고른다.
- 궤도 생성은 `pose_utils::scenario1Orbit`으로 공통, 실행만 방식별로 다르다.
- 상세는 §9.

#### 8.2.3 노드 `inspection_manager` — 검사 상태기계 + 액션

- 상태기계: `INIT → APPROACH → INSPECT → RETREAT → HOME`.
- `RunInspection` 액션 서버, `GoToNamedPose` 서비스.
- 각 검사각에서 정지→`CaptureImage` 트리거→촬영 순간의 관절/TCP/각도/이미지경로를 `InspectionSample`로 게시.
- MVP의 rule 스윕은 "inspect 기준 포즈에서 sweep 관절(기본 J6) 증분"으로 뷰를 생성(시나리오2 직접 대응, 시나리오1은 후속 태스크프레임 전략으로 확장).

#### 8.2.4 노드 `ee_pose_query` — 수동 티칭용 EE 추적 유틸

- §11 참조. **TF(base→flange)** + `/joint_states` 기반, 1Hz로 관절값·EE 포즈·복붙용 형식 출력.

#### 8.2.5 config

- `config/poses.yaml` — `inspection_manager`의 명명 포즈(home/init/approach/inspect/retreat, 관절값) + 모션 파라미터. **자리표시자**(티칭 필요).
- `config/scenario1.yaml` — `scenario1_inspect`(rule/hybrid)와 `scenario1_inspect_moveit` 파라미터(§16.1).
- `config/frames/mc_a.yaml` — 기계별(CNC A) cnc_frame/endmill_axis/approach_dir/safe_region/inspect_distance.

### 8.3 `tending_moveit` — 시나리오1 MoveIt 제어 (충돌회피)

- 노드 `scenario1_inspect_moveit`: `pose_utils`로 궤도(base→flange 목표)를 만들고 `MoveGroupInterface`로 각 뷰를 계획·실행.
- 계획기(`ompl`/`chomp`/`pilz_industrial_motion_planner`)와 `planner_id`를 파라미터로 선택.
- launch `scenario1_moveit.launch.py`: **`run_move_group`과 동일 방식**으로 robot_description(=`tm_description/xacro/tm5-900.urdf.xacro`)·SRDF·kinematics를 주입한다(§9.2의 교훈: `MoveItConfigsBuilder`는 링크명 불일치로 `tmr_arm` 그룹이 비어 사용 불가).

### 8.4 `tending_camera` — 카메라(stub)

- 노드 `camera_node`: `capture_image` 서비스 제공.
- `use_stub:=true`(카메라 미부착): 합성 PPM 이미지를 `output_dir/label/view_XXXX.ppm`에 저장하고 경로/타임스탬프 반환.
- 인터페이스를 먼저 고정했으므로 카메라 부착 후 **동일 서비스로 실드라이버만 교체**하면 된다.

### 8.5 `tending_data` — 검사 데이터 저장

- 노드 `dataset_recorder`: `inspection_sample` 구독 → 뷰별 `view_XXXX.json` 사이드카 + `manifest.jsonl`(append) 저장.
- 동기화 방식(P0, stop-and-shoot): 매니저가 정지 순간의 상태 스냅샷을 담아 게시하므로 결정적으로 영속화만 한다(연속 모션 시 message_filters로 대체 가능).

### 8.6 `tending_bringup` — (골격) 카메라 부착 셀 bringup

- 현재 골격만(패키지 존재). 향후 `tm_description`의 tm5-900 xacro에 **카메라+링라이트**를 부착한 확장 셀(`tending_description`)을 얹어 bringup하는 런치를 담을 예정.
- 로봇 자체 bringup/MoveIt/RViz는 `tmr_ros2`를 그대로 사용(원칙 §3).

### 8.7 `tending_description` / `tending_calibration` / `tending_bridge` (스캐폴드)

- `tending_description`: TM5 + 카메라/브라켓 마운트 + CNC/척/엔드밀 충돌모델 xacro (`tending_cell.xacro`에 TODO 골격).
- `tending_calibration` (P1): 핸드아이(eye-in-hand) 캘리브레이션 계획(`cv::calibrateHandEye` → `flange→camera`).
- `tending_bridge` (P2): C# `TendingSystem` GUI로 검사 결과 전송(경계 노드, TCP/JSON 또는 rosbridge).

---

## 9. 시나리오별 제어 방식 상세

### 9.1 시나리오 1의 궤도 기하 (모든 방식 공통)

시나리오 1은 카메라 광축이 툴(엔드밀) 축에 **수직**이다. 카메라(=TCP)가 툴 축을 중심으로 반경 `d`(검사거리) 원을 그리며 각도 θ만큼 궤도 이동하고, 광축은 항상 축을 향한다.

`scenario1ViewPose`의 계산(요지):

```cpp
a  = axis.dir.normalized();                       // 툴 축 방향
u0 = perpendicular_to(a);  v0 = a × u0;           // 축에 수직인 기준면
r  = cos(θ)·u0 + sin(θ)·v0;                       // θ 에서의 반경 방향
pos = axis.point + d · r;                         // 카메라 위치
z_cam = -r;                                       // 광축은 축을 향함
y_cam = (±a) 를 z 에 직교화;  x_cam = y_cam × z_cam;
R = [x_cam | y_cam | z_cam];  T = {R, pos};       // base→TCP
```

`orbitAngles`는 `angle_start..angle_end`를 `num_views`로 등각 분할하되, 완전 360°(≈2π)면 끝점 중복을 피해 `step = range/num_views`를 쓴다.

이 기하는 **세 방식이 모두 공유**한다(단일 소스 `pose_utils`). 방식의 차이는 "이 목표 포즈들을 **어떻게 실행하는가**"이다.

### 9.2 제어 방식 (2) — MoveIt 2 (충돌회피)

- 노드 `tending_moveit::scenario1_inspect_moveit`.
- 각 뷰 포즈를 `MoveGroupInterface::setPoseTarget` → `plan()` → `execute()`. OMPL/CHOMP/Pilz 선택.
- **장점**: 충돌회피·도달성 검증·경로 최적화. 상부 척/CNC를 PlanningScene에 넣으면 자동 회피.
- **단점**: `move_group` 기동·설정 필요, 계획 비결정성(재현성 다소 낮음), 셋업 비용.
- **핵심 교훈**: 클라이언트의 robot_description을 `MoveItConfigsBuilder`로 만들면 URDF/SRDF 링크명 불일치로 `tmr_arm` 그룹이 비어 `plan()`이 실패한다. **반드시 `run_move_group`과 동일하게** `tm_description/xacro/tm5-900.urdf.xacro` + `config/tm5-900.srdf` + `config/kinematics.yaml`을 주입해야 한다(`scenario1_moveit.launch.py`가 이를 수행).

실행:

```bash
# 터미널 A: move_group (가상 로봇이면 robot_ip 생략)
ros2 launch tm5-900_moveit_config tm5-900_run_move_group.launch.py
# 터미널 B: MoveIt 검사
ros2 launch tending_moveit scenario1_moveit.launch.py
```

### 9.3 제어 방식 (1) — 규칙기반/하드코딩 (충돌회피 없음)

- 노드 `tending_control::scenario1_inspect`, `control_mode:=rule`.
- 툴 축을 **base 좌표로 직접 지정**(`axis_point`/`axis_dir`), 궤도 포즈를 open-loop Cartesian 명령(`move_ptp_tool`=PTP_T 또는 `move_line_tool`=LINE_T)으로 전송.
- **장점**: 가장 단순, tm_driver 직접 명령이라 실물 배포 간단, 반복성 높음.
- **단점**: 충돌회피·도달성 검증 **없음** → 미검증 좌표에 위험. 환경 변화에 취약.
- `dry_run:=true`면 이동 없이 궤도 포즈만 출력(오프라인 검증).

실행:

```bash
# 오프라인 궤도 미리보기
ros2 launch tending_control scenario1_inspect.launch.py control_mode:=rule dry_run:=true
# 실물(먼저 tmr_ros2 로 bringup)
ros2 launch tending_control scenario1_inspect.launch.py control_mode:=rule dry_run:=false
```

### 9.4 제어 방식 (3) — 하이브리드(태스크프레임 + YAML) ★ 권장

- 노드 동일(`scenario1_inspect`), `control_mode:=hybrid`.
- 툴 축을 **cnc_frame(태스크 프레임) 기준**으로 정의하고 YAML로 외부화. 궤도를 태스크 프레임 상대로 생성한 뒤 base로 변환:

```cpp
// axis_point/dir 를 cnc_frame 기준으로 해석 → base 로 변환
Eigen::Isometry3d T = fromTmCartesian({cnc.xyz..., cnc.rpy...});
axis.point = T * pt;         axis.dir = T.rotation() * dir;
```

- 각 뷰를 **safe_region 박스로 검증**한 뒤 실행(룰베이스에 없는 보호):

```cpp
if (control_mode_=="hybrid" && !in_safe_region(p)) { stop(); return; } // 이탈 시 중단
```

- **장점**: 기계 교체 시 **YAML(cnc_frame/축/안전영역)만 바꾸면 재사용**(코드 무변경). 티칭+상대좌표라 실물 배포 단순, 안전영역 보호. 필요 구간만 MoveIt Cartesian/Pilz 결합 가능.
- **왜 3번째 방식으로 채택했나**: 요구는 "모든 CNC 자동 인식"이 아니라, 사용자가 기준프레임·툴위치·접근방향·안전영역을 제공하면 시스템이 작업을 생성·적응하는 것이다. 검사 궤적이 축(task frame) 기준 원/등각 스윕이므로 축 프레임만 바뀌면 동일 파라메트릭 궤적이 재사용된다 → **1대→3대 확장 시 코드 무변경, YAML+티칭만 교체**. ①단독은 셋업 과다, ②단독은 기계변경 적응 불가 → 하이브리드가 실현성·일반성·안전의 균형점.

### 9.5 세 방식 비교표

| 기준 | ① MoveIt | ② 룰베이스 | ③ 하이브리드 ★ |
|---|---|---|---|
| 개발 복잡도 | 높음 | 낮음 | 중간 |
| 충돌회피 | ✅ | ❌ | 부분(safe_region + 옵션 MoveIt) |
| 안전성 | 높음 | 고정환경만 | 높음(안전영역·티칭) |
| 반복성 | 높음(계획 비결정성) | 매우 높음 | 높음 |
| 기계변경 적응 | 중간 | 매우 낮음 | **높음(YAML 교체)** |
| 경로 유연성 | 매우 높음 | 낮음 | 중간~높음 |
| 실물 배포 | 중간 | 높음 | 높음 |
| 무하드웨어 검증 | 가상로봇 계획·실행 | dry_run 포즈 | dry_run + safe_region |
| 권고 | 충돌구간 검증 | MVP/기준선 | **3번째 방식 채택** |

세 방식은 배타적이지 않다. 궤도 생성(`pose_utils`)을 공유하고 실행 계층만 다르므로, 동일 검사 시퀀스를 세 방식으로 실행·비교할 수 있다.

### 9.6 시나리오 2 (계획)

L-브라켓 + EEF축∥스핀들축 정렬 후 **J6만 360° 회전**. `inspection_manager`의 sweep-joint 방식이 직접 대응한다(현재 MVP 골격). 상세 구현은 후속.

---

## 10. 검사 파이프라인 (MVP)

카메라 stub로 로봇 단독 엔드투엔드 흐름을 검증한다.

- 노드: `camera_node`(stub) + `dataset_recorder` + `inspection_manager`
- launch: `tending_control/mvp_inspect.launch.py`
- 흐름: `RunInspection` 액션 → INIT→APPROACH→INSPECT(뷰별 정지→촬영→샘플 게시)→RETREAT→HOME → result(데이터셋 경로)

```bash
# 1) 로봇 bringup (가상/실물)
ros2 launch tm5-900_moveit_config tm5-900_run_move_group.launch.py
# 2) 검사 노드
ros2 launch tending_control mvp_inspect.launch.py
# 3) 검사 실행
ros2 action send_goal /run_inspection tending_interfaces/action/RunInspection \
  "{machine_id: 'mc_a', scenario: 1, angle_start: 0.0, angle_end: 6.283, num_views: 8, inspect_distance: 0.1}" --feedback
# 4) 결과: /tmp/tending_dataset/mc_a/ 에 이미지(stub) + view_*.json + manifest.jsonl
```

---

## 11. 수동 티칭 유틸리티와 워크플로

### 11.1 목적

rule/hybrid 제어의 **목표 지점**(축점/포즈/관절)을 실측으로 채집하기 위해, MoveIt RViz의 **interactive marker로 로봇을 수동으로 끌어 옮기며** 그 순간의 EE 포즈·관절값을 1초마다 정리된 형태로 출력한다.

### 11.2 `ee_pose_query` 설계

- **포즈 소스 = TF(base→ee_frame)**. `robot_state_publisher`가 `joint_states`로부터 publish하는 TF를 쓰므로 **RViz 표시와 정확히 일치**하고, `tool_pose`를 계산하지 않는 **가상 로봇에서도 유효**하다.
- **관절 = /joint_states** (이름 매칭으로 `joint_1..6` 순서 보장).
- 1Hz(기본) 출력. 복붙용 형식 2종 제공.

### 11.3 워크플로

```bash
# 터미널 A: 가상 로봇 + RViz (MotionPlanning 의 interactive marker 로 드래그→Plan&Execute)
ros2 launch tm5-900_moveit_config tm5-900_run_move_group.launch.py
# 터미널 B: 1초마다 EE 상태 출력
ros2 run tending_control ee_pose_query
#   옵션: -p rate_hz:=2.0  -p once:=true  -p ee_frame:=flange  -p base_frame:=base
```

출력 예(가상 로봇 home):

```
────────── EE state: 'base' → 'flange' ──────────
 joints[rad] : [+0.0000, +0.0000, +0.0000, +0.0000, +0.0000, +0.0000]
 joints[deg] : [  +0.00,   +0.00,   +0.00,   +0.00,   +0.00,   +0.00]
 EE pose     : xyz=[0.0000, -0.2354, 1.0917] m, rpy=[90.0000, 0.0000, 0.0000] deg
 ── rule 목표 복붙용 (m, rad) ──
  pose[x,y,z,rx,ry,rz] : [+0.0000, -0.2354, +1.0917, +1.5708, +0.0000, +0.0000]
  axis_point [x,y,z]   : [+0.0000, -0.2354, +1.0917]
  poses.<name> (joints): [+0.0000, +0.0000, +0.0000, +0.0000, +0.0000, +0.0000]
──────────────────────────────────────────────────
```

- `pose[...]` → `scenario1.yaml`의 `axis_point`/pose로 붙여넣기
- `poses.<name>(joints)` → `poses.yaml`의 명명 포즈로 붙여넣기

---

## 12. 무하드웨어 검증 방법과 결과

### 12.1 오프라인 (로봇 불필요)

- **`scenario1_inspect dry_run:=true`**: rule/hybrid 궤도 포즈 계산·출력. hybrid는 safe_region 이탈 뷰를 `[!! safe_region 이탈]`로 표시.
- **`pose_utils` gtest 3종**: 검사거리/look-at, TM 왕복, 등각 분할 → 통과.

### 12.2 가상 로봇 (MoveIt 경로)

`run_move_group`(robot_ip 생략) + `tending_moveit/scenario1_moveit.launch.py`:
- OMPL로 view 0..N 계획·실행 성공.
- `joint_states`가 `[0,0,0,0,0,0]` → `[-1.49, -1.11, 0.36, 2.64, 0.06, 1.25]`로 이동 → **RViz에서 로봇이 궤도를 도는 것 확인**.

### 12.3 검증 중 발견·해결한 이슈

- **fake의 tool_pose 미계산**: 가상 로봇은 Cartesian 피드백(`tool_pose`)을 계산하지 않아 초기 EE 유틸이 쓰레기값(1e22)을 냈다 → 유틸을 **TF 기반**으로 전환해 해결.
- **좀비 프로세스 오염**: 반복 테스트에서 종료되지 않은 `ee_pose_query`가 같은 로그 파일에 겹쳐 써 구버전 출력이 보였다 → 명시적 정리로 해결.
- **MoveIt tmr_arm 빈 그룹**: `MoveItConfigsBuilder` 사용 시 링크명 불일치 → `run_move_group` 방식 description 주입으로 해결.

---

## 13. 안전 설계

- **인터록**: 모든 이동 폴링 루프에서 `safety_ok()` 확인 → `e_stop/safetyguard/robot_error/통신단절` 시 즉시 `stop()`(set_event STOP).
- **사전 점검**: 실제 이동 전 서비스 준비·첫 피드백 수신·안전 상태 확인(레이스 방지 대기 포함).
- **안전영역(hybrid)**: 각 목표 뷰를 safe_region 박스로 검증, 이탈 시 중단.
- **저속 원칙**: 첫 실행은 반드시 저속. 상부 척 충돌 여유 확인.
- **충돌회피가 필요한 미검증 좌표**는 rule 대신 hybrid(safe_region) 또는 MoveIt로 먼저 확인.
- **즉시 정지 CLI**: `ros2 service call /set_event tm_msgs/srv/SetEvent "{func: 11, arg0: 0, arg1: 0}"`

---

## 14. 실행 레시피 모음 (CLI Cookbook)

```bash
# [환경]
source /opt/ros/humble/setup.bash
cd ~/dev_ws/prject/NCC/Tending_ws && source install/setup.bash

# [빌드]
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# [로봇 bringup: 가상]
ros2 launch tm5-900_moveit_config tm5-900_run_move_group.launch.py
# [로봇 bringup: 실물]
ros2 launch tm5-900_moveit_config tm5-900_run_move_group.launch.py robot_ip:=192.168.0.10
# [드라이버만: 실물]
ros2 launch tm_driver tm_bringup.launch.py 192.168.0.10

# [티칭 유틸]
ros2 run tending_control ee_pose_query

# [시나리오1 rule/hybrid]
ros2 launch tending_control scenario1_inspect.launch.py control_mode:=rule   dry_run:=true    # 오프라인
ros2 launch tending_control scenario1_inspect.launch.py control_mode:=hybrid dry_run:=false   # 실물

# [시나리오1 MoveIt]
ros2 launch tending_moveit scenario1_moveit.launch.py
#   계획기 변경: scenario1.yaml 의 planning_pipeline(ompl|chomp|pilz_industrial_motion_planner)

# [MVP 파이프라인]
ros2 launch tending_control mvp_inspect.launch.py
ros2 action send_goal /run_inspection tending_interfaces/action/RunInspection \
  "{machine_id: 'mc_a', scenario: 1, angle_start: 0.0, angle_end: 6.283, num_views: 8, inspect_distance: 0.1}" --feedback

# [안전/상태]
ros2 topic echo /feedback_states --field e_stop
ros2 service call /set_event tm_msgs/srv/SetEvent "{func: 11, arg0: 0, arg1: 0}"   # 즉시 정지
```

> 전체·최신 명령은 `src/command.txt` 참조.

---

## 15. 향후 계획 (P1 / P2 / 다중 CNC)

### 15.1 P1 — 카메라 부착 후

1. **카메라 실드라이버**: `tending_camera`를 `use_stub:=false`로 전환(산업용 카메라 SDK 래핑, `image_raw`).
2. **핸드아이 캘리브레이션**(`tending_calibration`): ChArUco 타깃, 여러 포즈에서 `cv::calibrateHandEye` → `flange→camera`. TCP를 카메라 광학중심에 설정.
3. **실이미지 촬영·단일 CNC 실험**: 검사각별 이미지-포즈 동기 저장, 결함 가시성/반복성 측정.
4. **시나리오 비교**: 시나리오1 vs 2를 간섭·도달공간·거리일관성·이미지품질·모션복잡도·캘리브난이도·안전·반복성·기계적응성으로 정량 비교.

### 15.2 다중 CNC 확장 (하이브리드 방식)

1. 단일 CNC: 룰/하이브리드로 티칭.
2. 파라미터화: 고정 포즈를 cnc_frame/endmill_axis 기준 상대좌표로.
3. 설정 외부화: `config/frames/{mc_a,mc_b,mc_c}.yaml`, `machine_id` 런타임 선택.
4. 3대 운영: **코드 무변경**, YAML 3개 + 각 기계 티칭으로 확장. 충돌 우려 구간만 MoveIt 결합.

### 15.3 P2 — TendingSystem(C#) 브리지

`tending_bridge`: 검사 결과/상태/데이터셋 경로를 C# GUI로 전송. 1차 권장 TCP/JSON(의존성 최소), 대안 rosbridge websocket.

### 15.4 tending_bringup 채우기

카메라 부착 시 `tending_description`에 카메라+링라이트+CNC 충돌모델 xacro를 완성하고, `tending_bringup`에 확장 셀 bringup 런치를 추가.

---

## 16. 부록

### 16.1 파라미터 레퍼런스

**`scenario1_inspect` (rule/hybrid)** — `config/scenario1.yaml`

| 파라미터 | 기본 | 설명 |
|---|---|---|
| `control_mode` | rule | rule \| hybrid |
| `dry_run` | true | true=포즈만 출력 |
| `use_line` | false | true=LINE_T(직선), false=PTP_T |
| `axis_point` | [0.4,0,0.4] | rule=base / hybrid=cnc_frame 기준 축점 |
| `axis_dir` | [0,0,-1] | 축 방향 |
| `inspect_distance` | 0.10 | 검사거리(m) |
| `angle_start/end` | 0 / 2π | 궤도 각도 범위 |
| `num_views` | 12 | 등각 분할 수 |
| `velocity` / `line_velocity` | 0.3 / 0.05 | PTP_T(rad/s상당) / LINE_T(m/s) |
| `pos_tol_m` / `ang_tol_rad` | 0.003 / 0.02 | 도달 허용오차 |
| `move_timeout_s` / `settle_ms` | 30 / 300 | 이동 타임아웃 / 정지 안정화 |
| `cnc_frame.xyz/rpy` | 0 | hybrid 태스크 프레임 |
| `safe_region.min/max_xyz` | 박스 | hybrid 안전영역 |

**`scenario1_inspect_moveit`** — 동일 파일

| 파라미터 | 기본 | 설명 |
|---|---|---|
| `group` | tmr_arm | MoveIt 그룹 |
| `planning_pipeline` | ompl | ompl \| chomp \| pilz_industrial_motion_planner |
| `planner_id` | RRTConnectkConfigDefault | 계획기 ID |
| `reference_frame` / `ee_link` | base / flange | 포즈 기준/EE 링크 |
| `velocity_scaling`/`accel_scaling` | 0.1 | 속도/가속 스케일 |
| `planning_time` | 5.0 | 계획 시간(s) |
| (궤도) `axis_point/dir, inspect_distance, angle_*, num_views` | — | rule과 동일 의미 |

**`ee_pose_query`**: `once`(false), `rate_hz`(1.0), `base_frame`(base), `ee_frame`(flange), `joint_order`.

### 16.2 파일 인덱스 (신규 구현물)

```
tending_interfaces/{action/RunInspection.action, srv/CaptureImage.srv, srv/GoToNamedPose.srv, msg/InspectionSample.msg}
tending_control/include/tending_control/{pose_utils.hpp, robot_io_bridge.hpp}
tending_control/src/{pose_utils.cpp, robot_io_bridge.cpp, scenario1_inspect.cpp, inspection_manager.cpp, ee_pose_query.cpp}
tending_control/test/test_pose_utils.cpp
tending_control/config/{poses.yaml, scenario1.yaml, frames/mc_a.yaml}
tending_control/launch/{scenario1_inspect.launch.py, mvp_inspect.launch.py}
tending_moveit/src/scenario1_inspect_moveit.cpp
tending_moveit/launch/scenario1_moveit.launch.py
tending_camera/src/camera_node.cpp
tending_data/src/dataset_recorder.cpp
tending_description/urdf/tending_cell.xacro
doc/{PROJECT_PLAN.md, README.md, research_report.md}  command.txt
```

### 16.3 트러블슈팅

| 증상 | 원인 | 해결 |
|---|---|---|
| `Group 'tmr_arm' was not found` | 클라이언트 robot_description 불일치 | `scenario1_moveit.launch.py`처럼 `tm_description` xacro + config SRDF 주입 |
| EE 유틸이 1e22 등 쓰레기값 | fake가 tool_pose 미계산 | TF(base→flange) 기반 유틸 사용(기본) |
| 구버전 출력이 계속 보임 | 이전 테스트 프로세스 잔존 | `pkill -9 -f <node>` 로 정리 후 재실행 |
| `tm_driver 서비스 미검출` | 로봇 미bringup | `run_move_group`/`tm_bringup`으로 먼저 bringup |
| `set_positions 거부` | IK 실패/미연결/도달불가 | 좌표 재확인, hybrid safe_region/MoveIt로 사전 검증 |
| 재빌드 overlay 경고 | 동일 패키지 중복 감지 | `--allow-overriding <pkg>` |

### 16.4 용어

- **TCP**: Tool Center Point(공구 중심점) — 카메라 광학중심(캘리브 후).
- **TMSCT/TMSVR**: TM 외부 스크립트(명령)/이더넷 슬레이브(피드백) 프로토콜.
- **태스크 프레임**: 검사 대상(엔드밀 축)을 원점으로 하는 좌표계(cnc_frame). 하이브리드 재사용의 핵심.

---

*본 보고서는 구현 진행에 따라 갱신한다. 계획 정본은 `PROJECT_PLAN.md`, 실행 명령은 `command.txt`.*
