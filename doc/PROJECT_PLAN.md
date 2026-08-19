# 엔드밀 로봇 비전 검사 시스템 — 프로젝트 실행 계획 (통합 문서)

> 본 문서는 프로젝트의 **단일 통합 문서**입니다. 이후 구현이 진행되면서 이 문서를 계속 수정·확장합니다.
> 워크스페이스 내 `src/doc/PROJECT_PLAN.md` 와 항상 동일하게 유지합니다(정본 미러). 실행 명령은 `src/command.txt`.

---

## Context (배경 · 목표 · 결정 사항)

**문제/필요성.** 바닥에 고정된 TM5 로봇이 상부 CNC 척에 매달린 엔드밀에 아래에서 접근하여, 측면 360°를 촬영해 플루트 형상·절삭날 마모·치핑·스크래치 등 결함을 검사한다. 핵심 목표는 "카메라를 로봇에 다는 것"이 아니라 **동일 검사 작업에 대해 두 가지 카메라 장착 구성(시나리오 1: 손목 직접 장착 90° / 시나리오 2: L-브라켓)을 비교**하고, **3가지 로봇 제어 방식**을 구현·비교하여 **1대 → 3대 CNC로 확장 가능한 재사용 제어 방식**을 도출하는 것이다.

**사용자 확정 결정 (2026-07-27).**
| 항목 | 결정 |
|---|---|
| 카메라 하드웨어 | **외부 산업용 카메라(USB3/GigE) + 링라이트**, 별도 ROS 2 카메라 드라이버 패키지 |
| 신규 패키지 구현 언어 | **C++ (rclcpp)** — 오케스트레이션/제어 |
| MVP 우선 시나리오 | **시나리오 1 (손목 직접 장착, 90°)** |
| 시뮬레이션 범위 | **최소한 (RViz/MoveIt 경량 사용), 실물 TM5 위주** |
| 카메라 도입 시점 | **~2주 뒤(≈2026-08-10) 배송·HW 부착 예정.** 그 전까지는 **구조/인터페이스만 구축, 물리 카메라 테스트는 유예**(스텁·목 모드로 개발) |
| 외부 GUI 연동 | **후순위.** 추후 C# 기반 **`TendingSystem` GUI**로 검사 데이터 전송 필요 → 전용 브리지(`tending_bridge`) 계획에 포함 |

**개발 단계 분류 (우선순위).**
- **P0 (지금 진행, 카메라 불필요):** 로봇 통신·상태(단계 3), 안전 포즈(4), 프레임/TCP(5), 시나리오1 기구·모션 구조(7), 궤적·트리거 인터페이스(8), 3제어방식(9), 안전(10), 시뮬 검증(11), 데이터 동기 파이프라인 구조(12) — **카메라는 스텁으로 대체**하여 엔드투엔드 흐름까지 검증.
- **P1 (카메라 부착 후):** 카메라 드라이버 실연동, 핸드아이 캘리브(6), 실이미지 촬영·1대 CNC 실험(13), 3대 확장(14), 시나리오 비교(15).
- **P2 (후순위):** `TendingSystem`(C#) 브리지(단계 16).

**패키지 관리 규칙 (엄수).**
- 신규 코드 파일은 **절대 `tmr_ros2/` 내부에 생성하지 않는다.**
- 신규 기능은 모두 **워크스페이스 `src/` 아래 신규 패키지**로 생성한다 (`tmr_ros2/`의 형제 디렉터리).
- `tmr_ros2`는 **읽기 전용 의존성**으로만 사용(서비스/토픽/액션 클라이언트, xacro include, MoveIt config 재사용). 기존 코드는 삭제·대규모 수정하지 않는다.

---

## 0. 워크스페이스 분석 결과 (Task 1 완료분)

- 워크스페이스: `/home/mk/dev_ws/prject/NCC/Tending_ws/src`
- `src` 내 실질 콘텐츠는 `tmr_ros2/` 하나 (TechmanRobotInc/tmr_ros2, **humble** 브랜치). 빌드: colcon, ROS 2 **Humble**, 권장 RMW: Cyclone DDS.

**`tmr_ros2` 핵심 패키지 및 재사용 포인트**

| 패키지 | 재사용 방식 |
|---|---|
| `tm_driver` (C++) | 드라이버 노드. TMSCT(Listen Node, 5890)=명령 / TMSVR(Ethernet Slave, 5891)=피드백. 우리가 **서비스 클라이언트/토픽 구독**으로 사용. 진입: `launch/tm_bringup.launch.py` (`robot_ip:=`) |
| `tm_msgs` | **명령/상태 인터페이스 재사용** (아래 표) |
| `tm_description` | TM5 URDF/xacro 매크로 (`xacro/macro.tm5-900-nominal.urdf.xacro` 등) — 신규 description에서 xacro include |
| `tm_moveit/tm5-900_moveit_config` (외 tm5-700/tm5x) | **MoveIt 2 config 재사용**. `launch/tm5-900_run_move_group.launch.py`는 tm_driver까지 함께 기동 |
| `demo` (C++) | 서비스 호출 참조 구현: `demo_send_script.cpp`, `demo_set_positions.cpp`, `demo_set_event.cpp`, `get_status_demo_src/demo_get_feedback.cpp` |
| `custom_package` (C++/OpenCV) | TMvision 이미지 수신 예시 — 외부 카메라 채택으로 직접 재사용은 안 하나 이미지 표시 패턴 참조 |

**재사용할 `tm_msgs` 인터페이스 (신규 인터페이스 최소화)**

- 서비스 `send_script` (`tm_msgs/srv/SendScript`: `string id, string script → bool ok`) — 임의 TM 스크립트(PTP/Line/QueueTag/ChangeTCP 등) 전송.
- 서비스 `set_positions` (`tm_msgs/srv/SetPositions`: `int8 motion_type{PTP_J=1,PTP_T=2,LINE_T=4,CIRC_T=6,PLINE_T=8}, float64[] positions, float64 velocity, float64 acc_time, int32 blend_percentage, bool fine_goal → bool ok`) — 구조화된 PTP/Line.
- 서비스 `set_event` (`tm_msgs/srv/SetEvent`: `func{TAG=1,WAIT_TAG=2,STOP=11,PAUSE=12,RESUME=13,EXIT=-1}, arg0, arg1`) — QueueTag 동기화 · 정지/일시정지.
- 서비스 `set_io` (`tm_msgs/srv/SetIO`) — 엔드이펙터/컨트롤박스 DO/AO (링라이트 트리거 후보).
- 서비스 `connect_tmsvr`/`connect_tmsct` (`tm_msgs/srv/ConnectTM`) — 소켓 연결·재연결 관리.
- 토픽 `feedback_states` (`tm_msgs/msg/FeedbackState`) — `joint_pos/vel/tor`, `tool0_pose`(플랜지), `tool_pose`(TCP), `tcp_speed/force`, 연결 플래그 `is_svr_connected/is_sct_connected`, 안전 플래그 `e_stop/safetyguard_a/robot_error/project_run/pause`, `error_code/error_content`, IO 배열.
- 토픽 `joint_states` (`sensor_msgs/JointState`), `tool_pose` (`geometry_msgs/PoseStamped`).
- 액션(MoveIt 경로): 드라이버가 `control_msgs/action/FollowJointTrajectory` 서버 제공 (`tm_ros2_movit_sct.cpp`).

**분석 결론:** 로봇 명령·상태 인터페이스는 이미 완비되어 있어 **신규 인터페이스는 "검사 오케스트레이션" 계층에만** 필요하다.

---

## A. 권장 시스템 아키텍처

### A.1 하드웨어 구성
```
[상부 CNC 척] ── 엔드밀(스핀들 축, 아래로 매달림)
        ▲ 아래에서 접근
[TM5 로봇] ─ 플랜지(tool0)
   ├─ 시나리오 1: 플랜지에 카메라 직결 (광축 ⟂ 스핀들축, 손목 90° 회전)
   └─ 시나리오 2: L-브라켓(측방 오프셋) 끝에 카메라+링라이트, EEF축∥스핀들축, J6 회전
[외부 산업용 카메라(USB3/GigE) + 링라이트] → 검사 PC
[검사 PC] ─ ROS 2 Humble, tm_driver ↔ TM5 (Ethernet 5890/5891)
```

### A.2 소프트웨어 노드 구성 (신규 패키지는 모두 `src/` 아래, `tmr_ros2` 외부)

```
 tm_driver (기존, 재사용)
   ├─(sub) feedback_states / joint_states / tool_pose
   └─(srv) send_script / set_positions / set_event / set_io / connect_*
        ▲                                   │
   상태 구독                            명령 호출
        │                                   ▼
 ┌─────────────────────── tending_control (신규, rclcpp) ───────────────────────┐
 │  robot_io_bridge     : tm_driver 서비스/토픽을 래핑한 얇은 C++ 클라이언트 계층 │
 │  motion_executor     : PTP/Line/Cartesian orbit · QueueTag 동기 실행         │
 │  inspection_manager  : 상태기계(INIT→APPROACH→INSPECT→RETREAT→HOME),          │
 │                        RunInspection 액션 서버, 제어방식 3종 플러그인 선택      │
 │  safety_monitor      : e_stop/safeguard/error/연결손실 감시 → 즉시 STOP        │
 └───────────────────────────────────────────────────────────────────────────┘
        │ CaptureImage.srv (트리거)        ▲ 이미지+메타
        ▼                                  │
 ┌───── tending_camera (신규) ─────┐   ┌───── tending_data (신규) ─────┐
 │ 산업용 카메라 드라이버 래핑    │   │ 이미지 ↔ pose/joint/ts/각도 │
 │ (image_raw), 링라이트 제어,   │   │ 동기화(message_filters),    │
 │ SW/HW 트리거 CaptureImage srv │   │ 디스크 저장(+JSON 사이드카) │
 │ ※ 카메라 부착 전엔 stub 모드  │   └─────────────┬───────────────┘
 │   (합성/저장 이미지 반환)     │                 │ 검사 결과·데이터셋
 └──────────────────────────────┘                 ▼
                                    ┌──── tending_bridge (신규, P2 후순위) ────┐
                                    │ TendingSystem(C#) GUI로 검사 데이터   │
                                    │ 전송. 경계 노드(TCP/JSON 또는         │
                                    │ rosbridge websocket). 결과·상태·경로  │
                                    └───────────────────────────────────────┘

 tending_description : TM5 + 카메라/브라켓 마운트 + CNC/척/엔드밀 충돌모델 (xacro, tm_description include)
 tending_calibration : 핸드아이(eye-in-hand) 캘리브레이션 루틴 · TCP 설정 (P1, 카메라 부착 후)
 tending_bringup     : launch/config(YAML: 기계별 파라미터·티칭 포즈), RViz, 통합 문서
 tending_interfaces  : 신규 msg/srv/action (검사 오케스트레이션 전용)
 tending_bridge      : TendingSystem(C#) 연동 브리지 (P2 후순위)
```

**카메라 스텁 전략 (P0).** `tending_camera`는 `CaptureImage.srv` 인터페이스를 먼저 확정하고, `use_stub:=true` 파라미터로 **합성/사전저장 이미지를 반환**하는 목 구현을 제공한다. 오케스트레이션·데이터 동기·안전·모션 전 흐름을 카메라 없이 엔드투엔드 검증하고, 카메라 부착(≈2026-08-10) 후 동일 인터페이스로 실드라이버만 교체한다.

### A.3 신규 인터페이스 (`tending_interfaces`, 최소 집합)
- `action/RunInspection.action` — goal: `string machine_id, uint8 scenario, float64 angle_start/angle_end, uint16 num_views, float64 inspect_distance` / feedback: `float64 progress, uint16 captured` / result: `bool success, string dataset_path, uint16 image_count`.
- `srv/CaptureImage.srv` — req: `string label, uint32 view_index` / res: `bool ok, string image_path, builtin_interfaces/Time stamp`.
- `srv/GoToNamedPose.srv` — req: `string pose_name`(home/init/approach/retreat) / res: `bool ok`.
- `msg/InspectionSample.msg` — `Header header, uint32 view_index, float64 angle, float64[] joint_pos, geometry_msgs/Pose tcp_pose, string image_path`.
- 기계별 설정은 인터페이스가 아니라 **YAML**(아래 D/로드맵 참조).

### A.4 데이터·명령 흐름
1. 사용자 → `RunInspection` 액션(기계 id·시나리오·각도·뷰 수·거리).
2. `inspection_manager` → YAML에서 기계 프레임/티칭 포즈/안전영역 로드 → TF로 `cnc_frame`·`endmill_axis` 게시.
3. INIT→HOME→APPROACH 이동(`set_positions`/`send_script`), 매 스텝 `safety_monitor` 및 워크스페이스 박스 검사.
4. INSPECT: 각 검사각에서 정지(MVP: stop-and-shoot) → `CaptureImage` 트리거(+링라이트) → `tending_data`가 이미지와 그 순간 `joint_states`/`tool_pose`/timestamp/각도 동기 저장.
5. 완료 후 RETREAT→HOME, result 반환(데이터셋 경로).

---

## B. 제어 방식 비교표

| 기준 | ① MoveIt 2 경로계획 | ② 규칙기반/하드코딩 | ③ **태스크프레임 + 기계별 YAML(하이브리드)** ★권장 |
|---|---|---|---|
| 개발 복잡도 | 높음(MoveGroup/충돌환경/컨트롤러) | 낮음(스크립트/고정 포즈) | 중간 |
| 안전성 | 높음(충돌회피·계획검증) | 고정환경에선 높음, 환경변화엔 취약 | 높음(안전영역·티칭 기준·상대이동) |
| 반복성 | 높음(단, 계획 비결정성 있음) | 매우 높음(동일 궤적) | 높음 |
| 기계변경 적응 | 중간(장면·목표 재설정 필요) | 매우 낮음(재작성 필요) | **높음(YAML+티칭만 교체)** |
| 경로 유연성 | 매우 높음(임의 형상 회피) | 낮음 | 중간~높음(Cartesian/Pilz 결합) |
| 실물 배포 용이성 | 중간(설정 부담) | 높음 | 높음(tm_driver 직접 명령) |
| R&D 범위 적합성 | 비교연구용으로 필수 | 시나리오1 MVP·기준선으로 필수 | 3대 CNC 확장 목표에 최적 |
| 최종 권고 | 시나리오1 궤도경로·충돌회피 검증에 채택 | MVP/단일 CNC 기준선으로 채택 | **3번째 방식으로 최종 채택** |

> 세 방식은 배타적이지 않다. `motion_executor`에 **전략 플러그인**(rule / moveit / task_frame)으로 넣어 동일 검사 시퀀스를 세 방식으로 실행·비교한다.

---

## C. 단계별 개발 로드맵

각 단계: 목표 / 세부 작업 / ROS 2 노드·패키지 / 생성·수정 파일 / 검증 / 완료기준 / 주요 리스크.

### 단계 1 — 워크스페이스·tmr_ros2 구조 분석 ✅(본 문서 0장)
완료기준: 재사용 인터페이스·MoveIt config·description 목록 확정. 리스크: 없음.

### 단계 2 — TM 외부통신·Listen Node 조사 ✅
목표: TMSCT/TMSVR 채널·서비스·토픽 파악(0장). 검증: `demo_send_script`, `demo_get_feedback` 개념 확인. 완료기준: 명령/상태 인터페이스 확정.

### 단계 3 — 상태 피드백 & 원격명령 연결 (신규 `tending_control`의 `robot_io_bridge`)
- 세부: `tm_bringup.launch.py`로 tm_driver 기동(`robot_ip:=`) → `feedback_states`/`joint_states` 구독, `set_positions`/`send_script`/`set_event` 클라이언트 래핑, 재연결(`connect_tmsct/tmsvr`).
- 파일: `tending_control/src/robot_io_bridge.cpp`, `include/tending_control/robot_io_bridge.hpp`, `tending_bringup/launch/driver.launch.py`(tm_bringup include).
- 검증: 실물 TM5에서 `feedback_states.is_sct/svr_connected==true`, 소량 조인트 조그 성공. 완료기준: 상태 수신+안전한 단일 PTP 왕복. 리스크: TMflow에서 Listen Node/Ethernet Slave·프로젝트 실행 설정 필요, IP/방화벽.

### 단계 4 — 안전 포즈 정의 (init/approach/inspect/retreat/home)
- 세부: TMflow 수동 티칭으로 관절값 취득 → `tending_bringup/config/poses.yaml`. `GoToNamedPose` 서비스.
- 파일: `tending_bringup/config/poses.yaml`, `tending_control/src/inspection_manager.cpp`(포즈 이동), `tending_interfaces/srv/GoToNamedPose.srv`.
- 검증: 각 명명 포즈 왕복 이동 반복성 확인. 완료기준: 5개 포즈 안전 이동. 리스크: 상부 척과의 근접 충돌 여유.

### 단계 5 — 좌표 프레임·TCP 설계
- 프레임: `world→tm_base→...→flange(tool0)→tcp`; `world→cnc_frame→chuck→endmill_base→endmill_axis(스핀들축)→endmill_tip`; `flange→camera_optical`.
- 세부: `static_transform`/URDF로 프레임 게시, `cnc_frame`은 기계별 YAML로 파라미터화, TCP는 `send_script`의 `ChangeTCP`/`ChangeBase` 또는 TMflow TCP.
- 파일: `tending_description/urdf/tending_cell.xacro`(tm_description include + 마운트/CNC), `tending_bringup/config/frames/<machine>.yaml`, `tending_control` TF 브로드캐스트.
- 검증: RViz에서 프레임 정합, `tool_pose`와 TF 일치. 완료기준: 엔드밀 축 기준 좌표계 확립. 리스크: 축 정의 부호/오리엔테이션 혼동.

### 단계 6 — TCP·핸드아이(eye-in-hand) 캘리브레이션 (`tending_calibration`)  ⏳P1(카메라 부착 후)
- 세부: 체스보드/ChArUco 타깃, 여러 포즈에서 이미지+플랜지 포즈 수집 → `cv::calibrateHandEye`로 `flange→camera` 산출. TCP를 카메라 광학중심에 설정.
- 파일: `tending_calibration/src/handeye_collect.cpp`, `handeye_solve.cpp`, `tending_bringup/config/handeye.yaml`(결과).
- 검증: 재투영오차 임계·독립 포즈 검증. 완료기준: `flange→camera` 변환 저장·로드. 리스크: 외부 카메라 내부파라미터 캘리브 선행 필요, 조명 반사.

### 단계 7 — 두 시나리오 기구·모션 설계 (`tending_description` + `tending_control`)
- 시나리오1(★MVP 우선): 플랜지 카메라 직결, 손목 ~90° 회전으로 광축이 엔드밀 측면 향함. 360° 커버 위해 **엔드밀 축을 중심으로 로봇이 궤도(orbit) 이동**하며 거리·오리엔테이션 유지(Cartesian/궤도 경로).
- 시나리오2: L-브라켓 측방 오프셋, EEF축∥스핀들축 정렬 후 **J6만 360° 회전**해 카메라가 축 둘레를 스윕.
- 파일: `tending_description/urdf/mount_scenario1.xacro`, `mount_scenario2.xacro`, `tending_control/src/motion_executor.cpp`.
- 검증: RViz/MoveIt에서 도달성·간섭 시뮬 후 실물 소각도. 완료기준: 두 구성 모션 프로파일 정의. 리스크: 시나리오1 궤도경로의 도달성·손목 특이점, 시나리오2 브라켓 강성/오프셋 정확도.

### 단계 8 — 360° 검사 궤적 & 이미지 트리거  (P0: 스텁 트리거로 흐름 검증)
- 세부: 검사각 N등분 → 각 포즈 계산(태스크프레임 기준). MVP=stop-and-shoot(정지→`CaptureImage`), 확장=연속모션+`QueueTag/WaitQueueTag`(set_event) 또는 시간·포즈 임계 트리거.
- 파일: `tending_control/src/trajectory_generator.*`, `tending_camera/src/capture_service.cpp`(stub 포함).
- 검증: 균등 각도·거리 일관성, N회 트리거(스텁 이미지). 완료기준: 지정 각도범위 자동 트리거·저장. 리스크: 트리거-모션 지연 동기. **실이미지 품질 검증은 P1으로 유예.**

### 단계 9 — 3가지 제어 방식 구현·비교
- 세부: `motion_executor` 전략 플러그인 — (a)rule/하드코딩(`send_script` 고정 시퀀스), (b)MoveIt(MoveGroup/Cartesian, tm5-900_moveit_config 재사용), (c)태스크프레임+YAML(상대 좌표 궤적생성, 필요시 Pilz/Cartesian 결합).
- 파일: `tending_control/src/strategy_rule.cpp`, `strategy_moveit.cpp`, `strategy_taskframe.cpp` + 공통 `motion_strategy.hpp`.
- 검증: 동일 검사 시퀀스를 3방식으로 실행, B표 지표 측정. 완료기준: 3방식 동작·정량 비교표. 리스크: MoveIt 컨트롤러/충돌환경 셋업 비용(범위 최소화 결정 반영).

### 단계 10 — 안전·예외 처리 (`safety_monitor`)
- 세부: `feedback_states`의 `e_stop/safetyguard_a/robot_error/error_code` 감시→`set_event(STOP)`; SW 관절/작업공간 한계 사전검사; `is_sct/svr_connected` 손실 시 정지·재연결(`connect_*`); 액션 취소·타임아웃.
- 파일: `tending_control/src/safety_monitor.cpp`, `tending_bringup/config/limits.yaml`.
- 검증: e-stop/케이블 분리/한계초과 주입 테스트. 완료기준: 모든 결함에서 안전 정지·복구. 리스크: 상부 척 충돌은 물리 방지책(저속·여유·시뮬 선검증) 병행.

### 단계 11 — 시뮬/오프라인 검증 (경량)
- 세부: RViz+MoveIt로 도달성·충돌 점검(tm5-900_moveit_config 재사용). Gazebo는 선택. 실물 전 저속 드라이런.
- 파일: `tending_bringup/launch/rviz_check.launch.py`, `tending_bringup/rviz/*.rviz`.
- 검증: 계획된 궤적 충돌 0. 완료기준: 실물 투입 전 승인. 리스크: 시뮬-실물 캘리브 오차.

### 단계 12 — 이미지↔포즈/조인트/타임스탬프/각도 동기 (`tending_data`)
- 세부: `message_filters` ApproximateTime 또는 트리거 시점 상태 스냅샷 → 이미지+JSON 사이드카(`view_index, angle, joint_pos, tcp_pose, stamp`) 저장, 데이터셋 폴더 구조. rosbag2 선택 기록.
- 파일: `tending_data/src/dataset_recorder.cpp`, `tending_data/config/dataset.yaml`.
- 검증: 저장 메타-이미지 정합 스팟체크. 완료기준: 검사 1회당 완전 데이터셋 생성. 리스크: 타임스탬프 정합·디스크 IO.

### 단계 13 — 단일 CNC 실험  ⏳P1(카메라 부착 후)
- 세부: 실제 1대 CNC·엔드밀로 전체 파이프라인 실행(시나리오1 우선). 반복성·이미지품질 측정.
- 검증: 반복 검사 재현성, 결함 가시성. 완료기준: 안정적 1대 검사. 리스크: 조명 반사·초점.

### 단계 14 — 3대 CNC 설정기반 확장
- 세부: 기계별 `config/frames/<machine>.yaml`(cnc_frame·툴위치·접근방향·안전영역)만 교체. 사용자 티칭 절차 문서화. `machine_id`로 런타임 선택.
- 파일: `tending_bringup/config/frames/{mc_a,mc_b,mc_c}.yaml`, `tending_bringup/launch/inspect.launch.py`(`machine_id` 인자).
- 검증: 코드 변경 없이 3대 각각 검사 성공. 완료기준: YAML+티칭만으로 3대 지원. 리스크: 접근방향/주변구조 상이에 따른 도달성.

### 단계 15 — 성능평가·시나리오 비교  ⏳P1
- 세부: 두 카메라 장착 구성을 간섭·도달공간·거리일관성·이미지품질·모션복잡도·캘리브난이도·안전·반복성·기계적응성 기준으로 정량 비교.
- 파일: `tending_bringup/doc/` 평가 리포트(본 문서에 통합).
- 완료기준: 최종 권고 구성 도출. 리스크: 지표 정량화 기준 합의.

### 단계 16 — TendingSystem(C#) 브리지 (`tending_bridge`)  ✅ 구현 완료 (2026-08-06)

**로봇 통신 부분 구현 완료.** 카메라/이미지 전송은 미구현(§16.1).

- **와이어 프로토콜:** `tending_bridge/doc/PROTOCOL.md` **v1** — 개행 구분 JSON 라인 TCP.
  Ubuntu 가 서버(`0.0.0.0:5901`), Windows GUI 가 클라이언트. 단위는 **deg/mm**(UI 네이티브)로 보내고 브리지가 rad/m 변환.
- **소유권 결정:** ROS2(`tm_driver`)가 로봇을 단독 소유하고 Windows GUI 는 HMI. 단,
  ROS2 미기동 시를 대비해 GUI 에서 **TMSVR 5891 직결로 런타임 전환**할 수 있다(두 링크는 상호배타).
- **메시지:** ROS→GUI `state`(10Hz, 플래그 변화 시 즉시)/`event`/`ack`/`result`,
  GUI→ROS `run_inspection`/`cancel`/`goto_pose`/`jog`/`stop`/`ping`.
- **안전:** 모션 명령 1개만 in-flight(BUSY 거부), 검사 중 GUI 무음 3초 시 데드맨 정지,
  e-stop/safeguard/드라이버 끊김 시 이벤트 발행 + 진행 중 작업 중단.
  `stop` 만 `tm_msgs/SetEvent(STOP)` 직결(지연 최소화), 나머지는 전부 `inspection_manager` 경유.
- **신규 파일:** `tending_bridge/{src/tending_bridge_node.cpp, src/json_line_server.cpp,
  include/tending_bridge/json_line_server.hpp, config/bridge.yaml, launch/bridge.launch.py,
  scripts/mock_ui_client.py, doc/PROTOCOL.md}`
- **`tending_interfaces` 추가:** `srv/JogJoint.srv` (수동 조그) → `inspection_manager` 에 핸들러 구현.
  검사 goal 실행 중에는 조그/포즈이동을 거부한다(`inspecting_` 플래그).
- **`robot_io_bridge` 추가 파라미터:** `ignore_safety_flags`(기본 false).
  tmr_ros2 **가상 로봇이 FeedbackState 안전 플래그를 채우지 않아**(e_stop/robot_error/safetyguard 모두 true,
  error_code 쓰레기값) `safety_ok()` 가 항상 실패 → 오프라인 검증이 불가능해 옵트인 탈출구를 뒀다.
  `mvp_inspect.launch.py ignore_safety_flags:=true` 로만 켜고 **실물에서는 절대 사용 금지.**

**검증 완료(2026-08-06, 가상 로봇 + 실제 C# 클라이언트 코드):**
- 프로토콜 왕복: 연결 직후 스냅샷 `state`, 10Hz 스트림, 라인 프레이밍, `id` 매칭, 1초 ping/ack ✓
- 명령 라우팅: GUI → 브리지 → `inspection_manager`(액션/서비스) → `robot_io_bridge` → `/set_positions` ✓
- 거부 경로: `SERVICE_UNAVAILABLE`, `BAD_ARGS`, **`BUSY` 상호배타**(3개 동시 전송 → 1 수락 / 2 거부) ✓
- 안전: `safety_ok` 게이트, `stop` → `SetEvent` 미준비 시 에러 이벤트 ✓
- C# 측: `RosLineClient`/`RosProtocol`/`RosBridgeClient`/`RosRobotAdapter` 컴파일 + 실 브리지 접속 ✓
  어댑터가 `RosState` → `RobotStatus` 매핑, 연결 끊김 시 `connected=false` 처리 ✓

**미검증 (실물 TM5 필요):**
- **실제 로봇 모션.** tmr_ros2 **가상 로봇은 `/set_positions` 를 거부**하므로(FollowJointTrajectory
  경로만 지원) 브리지를 통한 이동은 항상 `ABORTED('init 이동 실패')` 로 끝난다. 이는 브리지가 아니라
  fake 드라이버의 한계이며, 명령이 드라이버까지 도달하는 것은 확인되었다.
- 데드맨 자동 정지(활성 goal 이 유지되어야 하는데 가상 로봇에서는 즉시 실패)
- 실물 e-stop 주입, 링크 모드 전환 시 5891 동시 점유 부재 확인

#### 16.1 카메라 확장 (미구현 · 예약)
`state` 에 `camera` 블록, `event` 에 `code:"CAPTURE"` + `image_path` 추가.
**이미지 바이트는 5901 소켓으로 보내지 않는다** — 데이터셋 폴더를 SMB/NFS 공유하고 경로만 전달(1차 권장).

---

### 단계 16 (원안) — TendingSystem(C#) 데이터 브리지  ⏳P2(후순위)
- 목표: 검사 결과·상태·데이터셋 경로를 C# 기반 `TendingSystem` GUI로 전송(그리고 필요 시 GUI→검사 시작/중지 명령 수신).
- 세부: (a) **경계 노드** `tending_bridge`가 `RunInspection` 결과·`InspectionSample`·로봇/검사 상태를 구독 → 외부 전송. (b) 전송 방식 후보: **rosbridge_suite(websocket+JSON, C#은 ROS# 라이브러리)** 또는 **경량 TCP/JSON 소켓 서버**(C# 측 커스텀 클라이언트). 프로젝트 배포 단순성 기준으로 택1 — 1차 권장은 TCP/JSON(의존성 최소, C# 연동 용이). (c) 스키마: `machine_id, scenario, dataset_path, image_count, per-view {index, angle, tcp_pose, joint_pos, stamp, image_path}, 검사 결과/에러`.
- 파일: `tending_bridge/src/tending_bridge.cpp`, `tending_bridge/config/bridge.yaml`(host/port/스키마 버전), `tending_interfaces` 재사용.
- 검증: 목(mock) C# 클라이언트 또는 `nc`/파이썬 소켓으로 JSON 수신 확인 → 실제 TendingSystem 연동 시 스키마 정합. 완료기준: 검사 1회 결과가 GUI로 전송·표시. 리스크: C# 측 스키마/버전 합의, 네트워크·방화벽, 후순위로 인터페이스 조기 고정 필요.

---

## D. Notion 프로젝트 페이지 구조 (작업 항목)

각 항목 하위에 목표/세부작업/산출물/검증절차/완료기준을 둔다.

1. **TM 로봇 외부통신 조사** — 목표: TMSCT/TMSVR·Listen Node 이해. 산출물: 통신 정리 노트. 완료: 인터페이스 확정.
2. **tmr_ros2 설치·실물 연결 검증** — 목표: colcon 빌드·`tm_bringup` 연결. 산출물: 연결 로그. 검증: `feedback_states` 수신. 완료: 상태 스트림 확인.
3. **Listen Node 통합** — 목표: `send_script`/`set_positions` 래핑(`robot_io_bridge`). 산출물: `tending_control`. 검증: 단일 PTP. 완료: 원격 이동.
4. **상태 수신·원격명령 송신** — 목표: 상태기반 이동·재연결. 산출물: bridge 노드. 완료: 안전 왕복.
5. **좌표프레임·TCP 정의** — 목표: 프레임 트리·TCP. 산출물: `tending_description`·frames YAML. 완료: RViz 정합.
6. **카메라 캘리브레이션** — 목표: 내부+핸드아이. 산출물: `tending_calibration`·`handeye.yaml`. 완료: 변환 저장.
7. **시나리오 1 구현(우선)** — 목표: 손목 90°·궤도 촬영. 산출물: mount_scenario1.xacro·strategy. 완료: 소각도 자동촬영.
8. **시나리오 2 구현** — 목표: L-브라켓·J6 360°. 산출물: mount_scenario2.xacro. 완료: 360° 스윕.
9. **360° 촬영 모션** — 목표: 등각 촬영·트리거. 산출물: trajectory_generator·capture_service. 완료: N장 취득.
10. **안전제어·예외처리** — 목표: e-stop/연결손실/한계. 산출물: `safety_monitor`·limits.yaml. 완료: 결함 시 정지.
11. **1대 CNC 테스트** — 목표: 전체 파이프라인. 산출물: 데이터셋. 완료: 반복 성공.
12. **다중 CNC 확장** ⏳P1 — 목표: YAML 기반 3대. 산출물: frames/{a,b,c}.yaml·티칭 가이드. 완료: 무코드 확장.
13. **성능평가·최종구성 비교** ⏳P1 — 목표: 두 시나리오 정량비교. 산출물: 평가 리포트. 완료: 권고 확정.
14. **TendingSystem(C#) 데이터 브리지** ⏳P2(후순위) — 목표: 검사 데이터 GUI 전송. 산출물: `tending_bridge`·bridge.yaml·전송 스키마. 검증: 목 C# 클라이언트 수신. 완료: 검사 결과 GUI 표시.

> ⏳P1 = 카메라 부착 후, ⏳P2 = 후순위. 표시 없는 항목(1~11의 P0)은 카메라 없이 스텁으로 지금 진행.

---

## E. 최소 기능 제품 (MVP)

**범위:** 시나리오 1(손목 직접 장착), 제어방식 ②규칙기반, 시뮬 최소, 실물 TM5 검증. **카메라는 아직 미부착이므로 `tending_camera`는 stub 모드**(합성/사전저장 이미지)로 동작 — 로봇 이동·트리거·저장·복귀 전 흐름을 카메라 없이 완주하고, 카메라 부착 후 실드라이버로 교체.

**MVP가 하는 일**
1. TM5 연결·현재 상태 확인 (`tm_bringup` + `feedback_states` 구독, 연결 플래그 확인).
2. 안전 초기 포즈로 이동 (`GoToNamedPose "init"`, poses.yaml).
3. 사용자가 제공한 엔드밀 기준 위치로 접근 (frames YAML의 `endmill_axis`·`inspect_distance` 기준 `set_positions`).
4. 정의된 카메라-툴 검사거리 유지.
5. 제한 각도범위 또는 완전 360° 촬영 (stop-and-shoot).
6. 각 검사 포즈에서 이미지 저장 (`CaptureImage` → `tending_data` 이미지+JSON).
7. 완료 후 안전 포즈 복귀.

**MVP 최소 패키지 집합**
- `tending_interfaces` (`CaptureImage.srv`, `GoToNamedPose.srv`, `RunInspection.action` 최소본)
- `tending_control` (`robot_io_bridge`, `inspection_manager` 상태기계, rule 전략)
- `tending_camera` (카메라 드라이버 래핑 + `CaptureImage`)
- `tending_data` (이미지+메타 저장)
- `tending_control/{launch/mvp_inspect.launch.py, config/poses.yaml}` (검사 launch/config)

**MVP 검증 (엔드투엔드)**
```bash
# 1) 드라이버 (tmr_ros2 공식 런치)
ros2 launch tm_driver tm_bringup.launch.py <TM5_IP>
# 2) 상태 확인
ros2 topic echo /feedback_states --field is_sct_connected
# 3) 검사 노드 + 실행 (MVP)
ros2 launch tending_control mvp_inspect.launch.py
ros2 action send_goal /run_inspection tending_interfaces/action/RunInspection \
  "{machine_id: 'mc_a', scenario: 1, angle_start: 0.0, angle_end: 6.283, num_views: 8, inspect_distance: 0.1}" --feedback
# 4) 결과: 데이터셋 폴더에 N장 이미지 + JSON, 로봇 home 복귀
```
완료기준: TM5 실물에서 3~7단계가 무사고로 1회 완주, N장 이미지-포즈 정합 저장.

---

## 최종 권고 — 3번째 제어 방식

**권고: "태스크프레임 기반 재사용 모션 + 사용자 티칭 기준점 + 기계별 YAML 설정"의 하이브리드.** (필요 구간에 MoveIt Cartesian/Pilz 결합)

**이유**
- 요구는 "모든 CNC 자동 인식"이 아니라, 사용자가 **기계 기준프레임·툴위치·접근방향·안전영역**을 제공하면 시스템이 검사 작업을 생성·적응하는 것 → 티칭+상대좌표 방식과 정확히 부합.
- 검사 궤적은 본질적으로 **엔드밀 축(task frame) 기준의 원/등각 스윕**이라, 축 프레임만 바뀌면 동일 파라메트릭 궤적이 재사용된다 → 3대 확장 시 **코드 무변경, YAML+티칭만 교체**.
- tm_driver의 `send_script`/`set_positions`로 직접 명령하므로 **실물 배포가 단순**하고, 규칙기반의 반복성·안전성 장점을 상속.
- 충돌회피가 필요한 구간(시나리오1 궤도경로 등)만 MoveIt Cartesian/Pilz를 선택적으로 결합해 MoveIt의 무거운 셋업 부담을 피함(시뮬 최소 결정과 정합).
- ①단독은 셋업 과다, ②단독은 기계변경 적응 불가 → 하이브리드가 실현 가능성·일반성·안전의 균형점.

**단일 → 3대 CNC 진화 경로**
1. **단일 고정 CNC:** ②규칙기반으로 티칭한 고정 포즈·궤적. 최고 반복성·최단 개발(=MVP).
2. **파라미터화:** 고정 포즈를 `cnc_frame`/`endmill_axis` 기준 상대좌표로 리팩터. 궤적생성기를 태스크프레임 입력으로 전환.
3. **설정 외부화:** 기계별 값을 `config/frames/<machine>.yaml`로 분리, `machine_id` 런타임 선택. 사용자 티칭 절차(기준프레임·툴위치·접근방향·안전영역) 표준화.
4. **3대 운영:** 코드 변경 없이 YAML 3개 + 각 기계 티칭으로 확장. 충돌 우려 구간만 MoveIt 결합.

---

## 패키지 관리 규칙 (재확인)
- 신규 패키지는 모두 `src/` 아래, `tmr_ros2` **외부**에 생성: `tending_interfaces`, `tending_description`, `tending_camera`, `tending_control`, `tending_moveit`, `tending_calibration`, `tending_data`, `tending_bringup`, `tending_bridge`(P2). 문서는 `src/doc/`, 명령은 `src/command.txt`.
- **★ 핵심 원칙 — tmr_ros2 = 코어:** 로봇 **bringup / MoveIt2 / 실물 연동**은 tmr_ros2 공식 런치를 그대로 사용한다(`tm_bringup.launch.py`, `tm5-900_run_move_group.launch.py` 등). Gazebo 는 사용하지 않는다. `tending_*` 는 tmr_ros2 에 없는 **보충**(검사 제어·인터페이스·카메라·데이터)만 담당하고 bringup 을 재구현하지 않는다.
- **패키지 목적:** 각 노드의 launch/config 는 그 노드를 소유한 패키지에 둔다(검사 launch/config → `tending_control`). `tending_bringup` 은 **골격만 유지** — 추후 카메라·링라이트를 tmr_ros2 URDF/xacro 에 부착한 확장 셀 bringup 용(카메라 부착 후 채움).
- `tmr_ros2` 내부에는 **어떤 파일도 생성/수정하지 않는다.** 서비스/토픽/액션/xacro/MoveIt config는 의존성으로만 참조.
- 언어: 신규 노드 C++(rclcpp). launch는 Python.
- 문서는 한국어, 본 통합 문서를 계속 갱신.

## 검증 총괄
- 단위: 각 노드 실물 소각도 드라이런 + RViz 프레임 정합.
- 통합: MVP 엔드투엔드(위 E) → 단일 CNC 반복성 → 3대 YAML 확장.
- 안전: e-stop/연결손실/한계초과 주입 테스트 통과 필수.

---

## 구현 현황 (2026-07-27)

**생성된 패키지 (모두 `src/` 아래, `tmr_ros2` 외부. `colcon build` 전체 통과):**

| 패키지 | 상태 | 주요 산출물 |
|---|---|---|
| `tending_interfaces` | ✅ 빌드 | `RunInspection.action`, `CaptureImage.srv`, `GoToNamedPose.srv`, `InspectionSample.msg` |
| `tending_camera` | ✅ 빌드·스모크테스트 | `camera_node` — `capture_image` 서비스. `use_stub:=true` 시 합성 PPM 저장(하드웨어 없이 검증) |
| `tending_data` | ✅ 빌드·스모크테스트 | `dataset_recorder` — `inspection_sample` → `view_XXXX.json` + `manifest.jsonl` |
| `tending_control` | ✅ 빌드·기동확인 | `robot_io_bridge`(tm_driver 래퍼) + `inspection_manager`/`scenario1_inspect`/`ee_pose_query`. **검사 launch/config 소유**(`launch/{mvp_inspect,scenario1_inspect}.launch.py`, `config/{poses,scenario1}.yaml`, `config/frames/mc_a.yaml`) |
| `tending_bringup` | 🟡 골격 | 카메라·링라이트 부착 셀 bringup 예정(카메라 부착 후). 로봇 bringup 은 tmr_ros2 사용 |

> **아키텍처 정리(2026-07-27):** ① tmr_ros2 = 코어(bringup/MoveIt/실물), tending = 보충. ② 검사 orchestration launch/config 는 `tending_control` 로 이동. ③ **fake 드라이버(`fake_tm_driver`)·`display.launch.py`·자체 RViz 는 제거** — 로봇 표시/시뮬은 tmr_ros2 공식 런치로. ④ `tending_bringup` 은 골격만 유지(추후 카메라 확장 bringup). ⑤ 노드명=executable 명 일치.
| `tending_description` | 🟡 스캐폴드(P1) | `urdf/tending_cell.xacro`(tm_description 재사용, 마운트/CNC TODO) |
| `tending_calibration` | 🟡 스캐폴드(P1) | 핸드아이 계획 문서 (카메라 부착 후) |
| `tending_bridge` | 🟡 스캐폴드(P2) | TendingSystem(C#) 전송 스키마 문서 |

**하드웨어 없이 검증 완료(P0):**
- `capture_image` → 640×480 PPM 생성·경로/타임스탬프 반환 ✓
- `inspection_sample` → 동기 JSON 사이드카 + manifest 저장 ✓
- `inspection_manager` → 액션/서비스 등록, `poses.yaml` 로드 ✓

### 시나리오 1 제어 로직 3종 + EE 유틸 (2026-07-27 추가, 로봇 단독 1차 테스트용)

카메라 없이 로봇 단독으로 임의 좌표 툴을 검사하는 제어 로직. 공통 기하는 `tending_control::pose_utils`
(엔드밀 축 중심 궤도 포즈 생성, 광축이 축을 향함, 검사거리 d 일정). TM 규약(m·rad, tf2 setRPY) 준수.

| 제어 방식 | 구현 | 특징 |
|---|---|---|
| **(1) 룰베이스(하드코딩)** | `scenario1_inspect` `control_mode:=rule` | 축을 base 좌표로 직접 지정, 궤도 포즈를 `set_positions` PTP_T/LINE_T 로 open-loop 전송. **충돌회피/도달성 검사 없음.** 가장 단순·취약 |
| **(2) MoveIt 경로계획** | `tending_moveit/scenario1_inspect_moveit` | `MoveGroupInterface`(group `tmr_arm`)로 각 뷰 계획·실행. `planning_pipeline` = **ompl/chomp/pilz** 선택, `planner_id` 지정. 충돌회피(PlanningScene). `move_group` 필요 |
| **(3) 제안=하이브리드** | `scenario1_inspect` `control_mode:=hybrid` | 축을 **cnc_frame(태스크 프레임)** 기준으로 정의→base 변환, 궤도 **상대 생성**. 각 뷰 **safe_region 박스 검증** 후 Cartesian 실행. YAML(`frames/<machine>.yaml`)만 교체로 기계 재사용 |

- **EE 포즈 추출 유틸(요청 #2):** `tending_control/ee_pose_query` — `feedback_states.tool0_pose`(flange)와 `tool_pose`(TCP)에서 **base 기준 EE 좌표**를 xyz(m)+rpy(deg)+quaternion 으로 출력(`once`/`rate_hz`). 라이브러리로도 재사용(`pose_utils`, `robot_io_bridge::flange_pose()/tcp_pose()`).
- **오프라인 검증:** `pose_utils` gtest 3종 통과(거리·look-at·왕복·궤도 등각). `scenario1_inspect dry_run:=true` 로 rule/hybrid 궤도 포즈 출력 확인(로봇 불필요). MoveIt 방식은 `move_group` 필요(오프라인 dry_run 미제공).
- **신규 파일:** `tending_control/{include/tending_control/pose_utils.hpp, src/pose_utils.cpp, src/scenario1_inspect.cpp, src/ee_pose_query.cpp, test/test_pose_utils.cpp, config/scenario1.yaml, launch/scenario1_inspect.launch.py}`, `robot_io_bridge`(Cartesian 이동 추가), `tending_moveit/*`(+`launch/scenario1_moveit.launch.py`).
- **실행:** (로봇 bringup 은 tmr_ros2 로 먼저 — command.txt [2])
  - 룰/하이브리드: `ros2 launch tending_control scenario1_inspect.launch.py control_mode:=rule|hybrid dry_run:=false`
  - MoveIt: `ros2 launch tm5-900_moveit_config tm5-900_run_move_group.launch.py robot_ip:=<IP>` 후 `ros2 launch tending_moveit scenario1_moveit.launch.py`
  - EE 좌표 추적(수동 티칭): `ros2 run tending_control ee_pose_query` — MoveIt RViz 의 interactive marker 로 로봇을 끌어 옮기며 1Hz 로 base→flange 포즈(+관절값, 복붙용 TM Cartesian/joints)를 출력. 포즈는 **TF(base→ee_frame)** 에서 읽어 RViz 와 일치하고 가상 로봇에서도 유효(fake 의 tool_pose 미계산과 무관). rule 목표(scenario1.yaml axis_point/pose, poses.yaml joints) 채집용.

### 무하드웨어 검증 방법 (fake 제거 후 — tmr_ros2 공식 가상 로봇 활용)

- **★ tmr_ros2 공식 가상(fake) 로봇:** `tm5-900_run_move_group.launch.py` 를 **`robot_ip` 없이** 실행하면 tmr_ros2 가 내장 fake 로봇으로 동작한다(`tm_ros2_composition_moveit.cpp` 의 `is_fake`, 로그 "use fake robot"). `move_group`+RViz+`/set_positions`·`/feedback_states`·`/joint_states`·FollowJointTrajectory 제공, `tmr_arm` 그룹 정상.
  - **검증 완료:** 가상 로봇 + `tending_moveit/scenario1_moveit.launch.py` → OMPL 로 view 0..N 계획·실행, `joint_states` 가 실제로 이동 → **RViz 에서 로봇이 궤도를 도는 것 확인** ✓ (실물 불필요)
  - 주의: `scenario1_moveit.launch.py` 의 robot_description 은 `MoveItConfigsBuilder` 가 아니라 **run_move_group 과 동일하게** `tm_description/xacro/tm5-900.urdf.xacro` + `config/tm5-900.srdf` 로 구성해야 `tmr_arm` 그룹이 유효하다(불일치 시 그룹이 빔).
- **`scenario1_inspect dry_run:=true`** — rule/hybrid 궤도 포즈 계산·출력(이동 없음).
- **`pose_utils` gtest 3종** — 검사거리·look-at·TM 왕복·등각 분할 (통과).

> **fake 드라이버 제거(2026-07-27):** 초기에 `fake_tm_driver`(tm_driver 모사) + `display.launch.py`(RViz) 로 무하드웨어 제어 검증을 했으나(당시 rule 궤도·`run_inspection` 액션 SUCCEEDED 확인), **tmr_ros2 = 코어 원칙**에 따라 제거함. 실물/시뮬 검증은 tmr_ros2 공식 bringup + 실제 TM5 로 수행한다. Gazebo 미사용.

**실물 TM5 필요(미검증):** `inspection_manager` 의 실제 모션 루프(INIT→APPROACH→INSPECT→RETREAT→HOME)는 `tm_driver` + 로봇 연결이 있어야 검증 가능. `move_ptp_joint` 은 `set_positions` 후 `feedback_states` 관절값을 폴링해 도달 확인.

**다음 작업(권장 순서):**
1. 실물 TM5 연결 → `mvp_inspect.launch.py start_driver:=true robot_ip:=<IP>` 로 전체 액션 루프 검증(저속).
2. TMflow 티칭으로 `config/poses.yaml` 자리표시자 값 교체(상부 척 충돌 여유 확인).
3. `safety_monitor` 독립 노드 분리(현재는 `robot_io_bridge` 이동 중 안전검사에 인라인) + 워크스페이스 박스/SW 관절한계.
4. P1: 카메라 실드라이버(`use_stub:=false`) + `tending_calibration` 핸드아이.

**빌드/실행:** `src/doc/README.md`, `src/command.txt` 참조.
