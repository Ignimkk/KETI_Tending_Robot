# ROS 2 ↔ Windows GUI 양방향 연동 — 기술 문서 (Ubuntu 측)

> 작성일: 2026-08-06 · 대상 독자: Ubuntu(ROS 2) 개발자
> Windows(C#) 측 구현은 `KETI_CNCTendingSystem/docs/ROS2_BRIDGE_CLIENT.md` 참조.
> 와이어 프로토콜 정본은 `src/tending_bridge/doc/PROTOCOL.md` (v1).

---

## 1. 배경

두 시스템이 각각 **같은 TM5 로봇에 직접** 붙어 있어 서로 단절되어 있었다.

| 시스템 | 위치 | 로봇 연결 |
|---|---|---|
| `KETI_CNCTendingRobot` (C# WinForms, .NET Framework 4.7.2) | Windows PC | `RobotClient` → TMSVR TCP **5891** 직결. `g_ReqState`/`g_ScenarioNum` 등 TM 전역변수를 직접 write |
| `Tending_ws` (ROS 2 Humble) | Ubuntu 22.04 PC | `tm_driver` → TMSCT **5890** / TMSVR **5891** |

`tending_bridge` 패키지는 `package.xml` + `CMakeLists.txt` + 스키마 초안 문서만 있는 **빈 스캐폴드**였다
(P2 후순위로 미뤄둔 상태).

**목표**: 로봇 명령(GUI→ROS2)과 로봇 상태(ROS2→GUI)가 양방향으로 흐르게 한다.
카메라/이미지 전송은 이번 범위 밖(§9).

---

## 2. 설계 결정과 근거

### 2.1 로봇 소유권 — ROS 2 단독

`tm_driver` 만 TM5 에 연결하고 Windows GUI 는 HMI 로 동작한다.

**근거**: TMSVR(5891)/TMSCT(5890) 은 두 마스터가 동시에 점유할 수 없다. 어느 한쪽이 소유해야 하는데,
검사 오케스트레이션(`inspection_manager`)·모션 전략·안전 감시가 이미 ROS2 쪽에 구현되어 있으므로
ROS2 를 소유자로 두는 것이 자연스럽다.

**단, 완전한 단방향 이전은 위험하다** — ROS2 가 안 떠 있거나 Ubuntu PC 가 다운되면 로봇을 전혀
못 쓰게 된다. 그래서 GUI 에서 **TMSVR 직결로 런타임 전환**할 수 있는 폴백 경로를 남겼다(§2.4).

### 2.2 전송 방식 — 경량 TCP/JSON 라인

| 후보 | 판단 |
|---|---|
| **경량 TCP/JSON 라인** ★채택 | 의존성 0(Ubuntu는 POSIX 소켓, Windows는 `System.Net.Sockets`+이미 참조 중인 Newtonsoft). `tending_bridge/doc/README.md` 의 1차 권장안과 일치 |
| rosbridge_suite + ROS# | ROS 인터페이스를 그대로 노출해 범용적이나, Ubuntu 에 rosbridge 설치 + Windows 에 ROS#/websocket-sharp NuGet 추가 필요. .NET Framework 4.7.2 호환성 확인 부담 |
| 양방향 HTTP REST | UI 의 기존 `HttpListener` 패턴을 재사용할 수 있으나, 10Hz 관절 각도 스트리밍에 폴링은 부적합 |

### 2.3 명령 경로 단일화

`stop` 을 제외한 **모든 명령은 `inspection_manager` 를 통과**한다.
브리지가 `tm_driver` 를 직접 호출하면 `safety_ok()` 검사와 상태기계를 우회하게 되므로,
모션 소유자를 하나로 유지한다.

**유일한 예외 — `stop`**: 비상정지는 지연이 곧 위험이므로 `tm_msgs/srv/SetEvent(STOP)` 를 직접
호출하고, 동시에 활성 goal 도 cancel 한다. 이 예외는 코드와 프로토콜 문서 양쪽에 명시했다.

### 2.4 링크 모드 (사용자 런타임 전환)

| 모드 | 링크 | 사용 가능 기능 |
|---|---|---|
| `ROS_BRIDGE` (기본) | GUI → TCP 5901 → `tending_bridge` → `inspection_manager` → `tm_driver` | Tool 검사, 포즈 이동, 조그, 비상정지, 상태 스트리밍 |
| `DIRECT_TMSVR` (폴백) | GUI → TMSVR 5891 → TM5 | 기존 4 시나리오 (clean-vise / check-chip / load / unload-material) |

**★ 불변식: 두 링크는 절대 동시에 활성화되지 않는다.**
Windows 측 `RobotLinkManager` 가 "이전 링크 완전 해제 → 새 링크 연결" 순서를 강제한다.
Ubuntu 측에서 할 일은 없지만, **운영 절차상 Direct 전환 전에 `tm_bringup` 을 정지해야 한다**(§8.3).

### 2.5 단위 규약 — 와이어는 deg / mm

ROS 는 rad/m 를 쓰지만 와이어는 **deg/mm**(UI 네이티브)로 보내고 브리지가 변환한다.

**근거**: `RobotArmPanel.SetJointAngles(double[] jointsDeg)` 가 deg 를 받는다(파라미터명으로 확인).
변환을 한 곳(브리지)에 몰아 두면 단위 혼동 버그가 생길 지점이 하나로 줄어든다.

---

## 3. 아키텍처

```
┌─ Windows PC ────────────────────────┐   ┌─ Ubuntu 22.04 PC ──────────────────────┐
│                                     │   │                                        │
│ MES ──HTTP 8080──> KETI_CNCTending   │   │                                        │
│   ├ HostServer / ActionCommand       │   │                                        │
│   ├ Seq_MainFlow → Seq_ToolInspect   │   │                                        │
│   │                                  │   │                                        │
│   ├ RobotLinkManager (상호배타)       │   │                                        │
│   │  ├[ROS_BRIDGE]                   │   │                                        │
│   │  │   RosBridgeClient ═══ TCP/JSON 5901 ═══> tending_bridge                    │
│   │  │     └ RosRobotAdapter         │   │        ├ JsonLineServer (poll 루프)     │
│   │  │         └ writes RobotStatus  │   │        ├ run_inspection 액션 클라       │
│   │  │             ↑ UI 패널 폴링     │   │        ├ go_to_named_pose / jog_joint  │
│   │  │                               │   │        ├ feedback_states / tool_pose 구독│
│   │  └[DIRECT_TMSVR]                 │   │        └ SetEvent(STOP) 직결 ← 비상정지만│
│   │      RobotClient ────────────────┼───┼──5891──┐         │                     │
│   │        (ROS2 미기동 시 폴백)       │   │        │  inspection_manager           │
│   └ ucMainTop: 모드 셀렉터 + 상태등   │   │        │    ├ RunInspection 액션 서버   │
│                                     │   │        │    ├ GoToNamedPose / JogJoint  │
└─────────────────────────────────────┘   │        │    └ RobotIoBridge            │
                                          │        │         │                     │
                                          │        └─ 배타 ─ tm_driver ──> TM5      │
                                          └────────────────────────────────────────┘
```

---

## 4. 와이어 프로토콜 v1 (요약)

정본은 `src/tending_bridge/doc/PROTOCOL.md`. 아래는 구현 관점 요약.

**전송**: TCP, Ubuntu 가 서버(`0.0.0.0:5901`), UTF-8, **개행(`\n`) 구분 JSON 1줄 = 1메시지**.
포트 5901 은 TM 의 5890/5891 과 겹치지 않게 선택했다.

**공통 봉투**: `{"v":1, "type":"...", "ts":<unix_ms>}`

### Ubuntu → Windows

| type | 주기/조건 | 내용 |
|---|---|---|
| `state` | 10Hz + **플래그 변화 시 즉시** | `link{sct,svr,driver}`, `safety{e_stop,robot_error,safetyguard,error_code,project_run}`, `joint_deg[6]`, `tcp{x,y,z,rx,ry,rz}`, `job{active,id,cmd,phase,progress,captured}` |
| `event` | 발생 시 | `level`(info/warn/error) + `code` + `message`. code: `SAFETY_ESTOP`/`SAFETY_GUARD`/`ROBOT_ERROR`/`DRIVER_LOST`/`DRIVER_OK`/`DEADMAN_STOP`/`LOG` |
| `ack` | 모든 cmd 에 1회 | `{id, ok, message}`. 거부 사유: `BUSY`/`BAD_ARGS`/`SERVICE_UNAVAILABLE`/`UNKNOWN_CMD`/`NOT_RUNNING` |
| `result` | 비동기 cmd 종결 | `{id, status, success, message, dataset_path, image_count}`. status: `SUCCEEDED`/`ABORTED`/`CANCELED`/`REJECTED` |

`state` 는 하트비트를 겸한다. `job.phase` 는 액션 feedback 의
`INIT/APPROACH/INSPECT/RETREAT/HOME/DONE` 을 그대로 싣는다.

### Windows → Ubuntu

모두 `type:"cmd"` + 클라이언트가 발급한 고유 `id`(GUID 앞 8자).

```
run_inspection  args{machine_id, scenario, angle_start_deg, angle_end_deg,
                     num_views, inspect_distance_mm}
cancel          args{target_id}
goto_pose       args{pose_name, velocity_deg_s}     # home/init/approach/inspect/retreat
jog             args{joint(1..6), delta_deg, velocity_deg_s}
stop            args{}                              # 최우선 처리
ping            args{}                              # 1초 주기
```

### 동작 규칙

1. **`stop` 최우선** — 수신 스레드에서 큐를 건너뛰고 즉시 처리.
2. **모션 명령 1개만 in-flight** — 두 번째는 `ack{ok:false,"BUSY"}`. `cancel`/`stop`/`ping` 은 항상 수락.
3. **데드맨** — 클라이언트 무음 > `client_timeout_ms`(3000) **이고 활성 goal 이 있으면**
   cancel + `SetEvent(STOP)` 후 `DEADMAN_STOP` 이벤트. 유휴 상태에서는 정지시키지 않는다.
4. **재연결** — Windows 가 2초 간격 무한 재시도. 재연결 후 **명령 재전송은 하지 않는다**
   (끊긴 사이 로봇 상태를 보장할 수 없으므로 사용자가 다시 지시하게 한다).

---

## 5. 구현 상세 (Ubuntu)

### 5.1 신규/변경 파일

| 파일 | 상태 | 내용 |
|---|---|---|
| `tending_bridge/doc/PROTOCOL.md` | 신규 | 와이어 프로토콜 정본 v1 |
| `tending_bridge/include/tending_bridge/json_line_server.hpp` | 신규 | TCP 서버 인터페이스 |
| `tending_bridge/src/json_line_server.cpp` | 신규 | POSIX 소켓 + `poll()` 구현 (~380줄) |
| `tending_bridge/src/tending_bridge_node.cpp` | 신규 | ROS 중계 노드 (~600줄) |
| `tending_bridge/config/bridge.yaml` | 신규 | 파라미터 |
| `tending_bridge/launch/bridge.launch.py` | 신규 | 런치 |
| `tending_bridge/scripts/mock_ui_client.py` | 신규 | Windows 없이 프로토콜 검증 |
| `tending_bridge/{CMakeLists.txt, package.xml}` | 수정 | placeholder → 실제 빌드 대상 |
| `tending_interfaces/srv/JogJoint.srv` | 신규 | 수동 조그 |
| `tending_interfaces/CMakeLists.txt` | 수정 | srv 등록 |
| `tending_control/src/inspection_manager.cpp` | 수정 | `jog_joint` 핸들러, `inspecting_` 플래그, goto 안전검사 |
| `tending_control/src/robot_io_bridge.cpp` / `.hpp` | 수정 | `ignore_safety_flags` 파라미터 (§7.2) |
| `tending_control/launch/mvp_inspect.launch.py` | 수정 | `ignore_safety_flags` 런치 인자 |

### 5.2 `JsonLineServer` — 전송 계층

ROS 에 의존하지 않는 순수 POSIX 계층. 자체 스레드에서 `poll()` 루프를 돌린다.

**책임**
- accept / 클라이언트 수 제한(`max_clients`)
- 클라이언트별 부분 수신 라인 누적 → 완전한 라인 단위로만 콜백
- `broadcast()` / `send_to()` 송신 큐 (self-pipe 로 `poll` 깨움)
- 클라이언트별 마지막 수신 시각 추적 (데드맨 판정용)

**동시성 규약 (중요)**

콜백(`on_line`/`on_connect`/`on_disconnect`)은 모두 **내부 IO 스레드**에서 호출되며,
**절대 `mtx_` 를 잡은 채로 호출하지 않는다.** 핸들러가 `send_to()`/`broadcast()` 를 부르면
같은 뮤텍스를 재획득하려 해 교착되기 때문이다. 구체적으로:

- `handle_readable()` 은 수신 버퍼 갱신·라인 분리까지만 락 안에서 하고, 분리된 라인들을
  로컬 벡터로 꺼낸 뒤 **락 밖에서** `on_line_` 을 호출한다.
- `drop_client()` 은 락 안에서 fd 정리·맵 삭제만 하고, `on_disconnect_` 는 **락 밖에서** 호출한다.

기타 방어:
- `kMaxLineBytes`(64KB) 초과 시 연결 종료 — 오동작 클라이언트가 버퍼를 무한히 키우지 못하게
- `TCP_NODELAY` — 상태 스트림/명령 지연 최소화
- `poll` 타임아웃 100ms — 클라이언트가 조용해도 데드맨 판정 주기를 유지
- CRLF 로 보내는 클라이언트 허용(`\r` 제거)

### 5.3 `tending_bridge_node` — ROS 중계

**구독**: `feedback_states`(`tm_msgs/FeedbackState`), `tool_pose`(`geometry_msgs/PoseStamped`)
**액션 클라이언트**: `run_inspection`
**서비스 클라이언트**: `go_to_named_pose`, `jog_joint`, `set_event`(STOP 전용)

**콜백 그룹 구성**

| 그룹 | 대상 | 이유 |
|---|---|---|
| `cb_group_` (Reentrant) | 구독, 클라이언트, `state_timer_` | 액션 실행 대기 중에도 상태 갱신이 계속되어야 함 |
| `watchdog_group_` (MutuallyExclusive) | `watchdog_timer_` | 에지 검출 상태(`prev_driver_alive_`, `deadman_fired_`)를 들고 있어 재진입하면 안 됨 |

**논블로킹 원칙**: JsonLineServer 콜백은 IO 스레드에서 오므로, 여기서 `future.wait_for()` 같은
블로킹을 하면 상태 스트림 전체가 멈춘다. 전부 **콜백 기반** `async_send_request`/`async_send_goal`
만 사용한다.

**작업 슬롯 (`claim_job`/`release_job`)**
`job_active_` 플래그 하나로 모션 명령의 상호배타를 구현한다. `claim_job()` 은 `state_mtx_` 를 잡고
검사·설정을 원자적으로 수행하므로, 같은 TCP 배치로 도착한 여러 명령도 정확히 하나만 통과한다
(§7.1 에서 실측 검증).

**안전 이벤트 에지 검출**
구독이 Reentrant 그룹이라 콜백이 동시 실행될 수 있어, `emit_safety_events()` 는 별도
`event_mtx_` 로 직렬화한다. 같은 전이에 대해 이벤트가 중복 발행되지 않게 하기 위함이다.

### 5.4 `JogJoint` 서비스

```
uint8 joint      # 1..6
float64 delta    # 현재 각도 기준 상대 증분 (rad)
float64 velocity # rad/s, 0 이면 노드 기본값
---
bool ok
string message
```

`inspection_manager` 에 핸들러를 구현했고, 기존 `bridge_->joint_pos()` / `move_to()` /
`safety_ok()` 를 재사용한다. 거부 조건: 검사 실행 중(`inspecting_`), 관절 번호 범위 밖,
상태 미수신, 안전 점검 실패.

`go_to_named_pose` 에도 같은 조건의 사전 검사를 추가했다(기존에는 안전 검사 없이 바로 이동했다).

**`inspecting_` 플래그**: `execute()` 진입 시 set, 소멸자 Guard 로 모든 종료 경로에서 reset.
검사 중 조그/포즈이동이 끼어들어 모션이 충돌하는 것을 막는다.

두 서비스 모두 `cb_group_`(Reentrant)에 등록했다 — 핸들러가 모션 완료까지 블로킹하므로
기본 그룹에 두면 노드의 다른 콜백을 막는다.

---

## 6. 시퀀스

### 6.1 검사 실행 (정상)

```
Windows                tending_bridge          inspection_manager        tm_driver
   │ cmd run_inspection  │                          │                       │
   ├────────────────────>│ claim_job()              │                       │
   │<──── ack{ok:true} ──┤                          │                       │
   │                     ├─ async_send_goal ───────>│                       │
   │                     │                          ├─ INIT ───────────────>│
   │<──── state{job.phase:"INIT"} ──────────────────┤ (feedback)            │
   │                     │                          ├─ APPROACH ───────────>│
   │<──── state{job.phase:"APPROACH", progress} ────┤                       │
   │                     │                          ├─ INSPECT (N views) ──>│
   │<──── state{phase:"INSPECT", captured:k} ───────┤                       │
   │                     │                          ├─ RETREAT/HOME ───────>│
   │<──── result{SUCCEEDED, dataset_path, count} ───┤                       │
   │                     │ release_job()            │                       │
```

`state` 는 10Hz 주기로도 나가지만, 액션 feedback 수신 시에는 **즉시 추가 송신**해
단계 전환이 지연 없이 GUI 에 반영된다.

### 6.2 비상정지

```
Windows                tending_bridge                    tm_driver / inspection_manager
   │ cmd stop            │
   ├────────────────────>│ (수신 스레드에서 큐 우회, 즉시)
   │<──── ack{ok:true} ──┤
   │                     ├─ SetEvent(STOP) ──────────────> tm_driver  (직결, 지연 최소)
   │                     ├─ async_cancel_goal ───────────> inspection_manager
   │<──── result{CANCELED} ─────────────────────────────── (액션 종결)
```

### 6.3 데드맨

```
Windows                tending_bridge
   │ (ping 두절)          │ watchdog 200ms 주기
   │                     │ last_rx_elapsed() > 3000ms  AND  job_active_
   │                     ├─ event{DEADMAN_STOP}  (broadcast)
   │                     ├─ SetEvent(STOP) + cancel goal
```

유휴 상태(활성 goal 없음)에서는 타임아웃이 정지를 유발하지 않는다.

---

## 7. 검증 결과 (2026-08-06)

### 7.1 수행한 검증

가상 로봇(`tm5-900_run_move_group.launch.py`, `robot_ip` 없이) + `inspection_manager` +
`tending_bridge` 를 띄우고, **두 종류의 클라이언트**로 확인했다.

1. `scripts/mock_ui_client.py` (Python)
2. **실제 Windows 측 C# 코드** — `RosLineClient.cs` / `RosProtocol.cs` / `RosBridgeClient.cs` /
   `RosRobotAdapter.cs` 를 dotnet 콘솔 프로젝트로 컴파일해 실 브리지에 접속

| 항목 | 결과 |
|---|---|
| 연결 직후 `state` 스냅샷 1회 (PROTOCOL §6) | ✅ |
| 10Hz 상태 스트림 | ✅ 13초에 129개 수신 |
| 라인 프레이밍 (부분 수신 누적, 다중 라인 배치) | ✅ |
| 1초 ping → ack | ✅ |
| `id` 매칭 (ack/result ↔ 전송 명령) | ✅ `result.Command` 가 `goto_pose` 로 정확히 복원 |
| 명령 라우팅 GUI→브리지→`inspection_manager`→`robot_io_bridge`→`/set_positions` | ✅ |
| 거부: `SERVICE_UNAVAILABLE` (서비스 미기동) | ✅ |
| **거부: `BUSY` 상호배타** | ✅ `run_inspection`×2 + `goto_pose` 를 **한 번의 write** 로 전송 → 1 수락 / 2 거부 |
| `stop` → `set_event` 미준비 시 에러 이벤트 | ✅ |
| 안전 게이트 (`safety_ok` 실패 시 거부) | ✅ `'안전 점검 실패: 비상정지(e_stop) 활성'` |
| 어댑터 seam: `RosState` → `RobotStatus` | ✅ `robotError=true, status='READY', message='E-STOP'` |
| 연결 끊김 시 `connected=false` + `'브리지 연결 끊김'` | ✅ |
| `colcon build` (3개 패키지) | ✅ 경고 0 |

BUSY 검증 실측 출력:
```
전송 id: {'8af77905': 'run_inspection', 'dd2416ee': 'run_inspection', '10f42253': 'goto_pose'}
ACK  8af77905 (run_inspection) ok=True  msg=
ACK  dd2416ee (run_inspection) ok=False msg=BUSY
ACK  10f42253 (goto_pose)      ok=False msg=BUSY
결과: 수락 1 / 거부 2
```

### 7.2 발견된 문제 — 가상 로봇의 한계

검증 중 tmr_ros2 가상 로봇의 제약 두 가지를 발견했다. **둘 다 브리지가 아니라 fake 드라이버의
한계**이며, 실물 TM5 에서는 해당 없다.

**(a) 안전 플래그를 채우지 않는다**

```
$ ros2 topic echo /feedback_states --once
is_svr_connected: false
is_sct_connected: false
robot_error: true
safetyguard_a: true
e_stop: true
error_code: -1086429464      ← 초기화되지 않은 값
```

`RobotIoBridge::safety_ok()` 가 항상 실패해 **오프라인 검증이 아예 불가능**했다.

→ **대응**: `robot_io_bridge` 에 `ignore_safety_flags` 파라미터(기본 `false`)를 추가했다.
`true` 면 "상태 미수신" 검사만 남기고 안전 플래그 검사를 건너뛴다.
기동 시 경고 로그를 남기며, **실물 로봇에서는 절대 사용 금지**다.

```bash
ros2 launch tending_control mvp_inspect.launch.py ignore_safety_flags:=true
# [WARN] ignore_safety_flags=true — 안전 플래그(...)를 무시합니다. 가상 로봇 검증 전용입니다.
```

**(b) `/set_positions` 를 거부한다**

```
[inspection_manager] [ERROR] set_positions 거부됨(스크립트 오류/미연결)
```

가상 로봇은 MoveIt 의 `FollowJointTrajectory` 경로만 지원한다. 따라서 브리지를 통한 이동은
항상 `ABORTED('init 이동 실패')` 로 끝난다. `PROJECT_PLAN.md` 의 기존 기록과도 일치한다
(rule/hybrid 경로는 `dry_run` 으로만 검증했고, 가상 로봇을 실제로 움직인 것은 MoveIt 경로였다).

→ **명령이 드라이버까지 도달하는 것은 확인**되었으므로 배선은 검증되었으나,
**실제 모션은 실물 TM5 가 필요**하다.

### 7.3 미검증 항목 (실물 TM5 필요)

| 항목 | 사유 |
|---|---|
| 실제 로봇 모션 (검사 완주, 포즈 이동, 조그) | 가상 로봇이 `/set_positions` 미지원 |
| 데드맨 자동 정지 | 활성 goal 이 유지되어야 하는데 가상 로봇에서 즉시 실패 |
| 실물 e-stop 주입 → `SAFETY_ESTOP` 이벤트 + 정지 | 실물 필요 |
| LAN 케이블 분리 → `DRIVER_LOST` | 실물 필요 |
| 링크 모드 전환 시 5891 동시 점유 부재 (`ss -tnp \| grep 5891`) | 실물 필요 |
| MES → UI → ROS2 전체 체인 | Windows 빌드 + 실물 필요 |

---

## 8. 운영

### 8.1 기동 순서

```bash
# 1) 로봇 브링업 (tmr_ros2 공식 런치 — 재구현하지 않음)
ros2 launch tm_driver tm_bringup.launch.py <TM5_IP>
#    가상 검증 시:
#    ros2 launch tm5-900_moveit_config tm5-900_run_move_group.launch.py

# 2) 검사 오케스트레이션
ros2 launch tending_control mvp_inspect.launch.py
#    가상 검증 시에만: ignore_safety_flags:=true

# 3) 브리지
ros2 launch tending_bridge bridge.launch.py
#    포트 변경: bridge.launch.py port:=5901
```

### 8.2 네트워크

- Ubuntu 방화벽: `sudo ufw allow 5901/tcp`
- Windows: 아웃바운드 5901 (보통 기본 허용). MES 용 8080 인바운드는 기존대로 유지
- **고정 IP 3대 확정 필요**: TM5 / Ubuntu PC / Windows PC
  → `config/bridge.yaml`(Ubuntu)과 `config.cfg`(Windows)에 각각 기록
- ROS_DOMAIN_ID / RMW 는 Ubuntu PC 내부 통신이므로 변경 없음

### 8.3 링크 모드 전환 절차 (운영 규칙)

GUI 에서 `DIRECT_TMSVR` 로 전환하려면 **Ubuntu 에서 `tm_bringup` 을 먼저 정지**해야 한다.
그러지 않으면 `tm_driver` 와 Windows `RobotClient` 가 같은 5891 을 잡으려 한다.

GUI 는 Ubuntu 프로세스를 제어할 수 없으므로, **브리지 링크 생존 여부를 프록시 신호로** 삼아
확인 다이얼로그를 띄운다. 이는 안전장치이지 보장이 아니므로 절차 준수가 필요하다.

전환 후 되돌릴 때: GUI 를 `ROS_BRIDGE` 로 바꾼 뒤 → Ubuntu 에서 `tm_bringup` 재기동.

### 8.4 파라미터 (`config/bridge.yaml`)

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `bind_address` | `0.0.0.0` | 리스닝 주소 |
| `port` | `5901` | 리스닝 포트 |
| `state_rate_hz` | `10.0` | `state` 송신 주기 |
| `client_timeout_ms` | `3000` | 데드맨 타임아웃 (Windows ping 1초 → 2회 유실까지 허용) |
| `max_clients` | `2` | UI 1 + 디버그 1 |
| `schema_version` | `1` | 봉투의 `v` |
| `driver_timeout_ms` | `2000` | 이 시간 이상 `feedback_states` 끊기면 `DRIVER_LOST` |

### 8.5 목 클라이언트

```bash
python3 src/tending_bridge/scripts/mock_ui_client.py                  # 상태 스트림 관찰
python3 src/tending_bridge/scripts/mock_ui_client.py --goto home
python3 src/tending_bridge/scripts/mock_ui_client.py --jog 6 --delta 30
python3 src/tending_bridge/scripts/mock_ui_client.py --run --views 4
python3 src/tending_bridge/scripts/mock_ui_client.py --run --cancel-after 3
python3 src/tending_bridge/scripts/mock_ui_client.py --stop
python3 src/tending_bridge/scripts/mock_ui_client.py --no-ping --run    # 데드맨 확인
```

### 8.6 트러블슈팅

| 증상 | 원인 / 조치 |
|---|---|
| `bind(0.0.0.0:5901) 실패: Address already in use` | 이전 브리지 프로세스 잔존. `ss -tlnp \| grep 5901` 로 PID 확인 후 종료 |
| `ack{ok:false, "SERVICE_UNAVAILABLE"}` | `inspection_manager` 미기동. 기동 순서 §8.1 확인 |
| `result{ABORTED, "안전 점검 실패: ..."}` | 실제 안전 이상이거나, 가상 로봇이면 `ignore_safety_flags:=true` 필요 |
| `result{ABORTED, "init 이동 실패"}` | 가상 로봇의 `/set_positions` 미지원(§7.2b). 실물에서는 실제 원인 확인 |
| `link.driver=false` | `feedback_states` 2초 이상 끊김. `tm_bringup` 확인 |
| GUI 가 연결되지만 상태가 안 옴 | 방화벽(5901) 또는 스키마 버전 불일치. 브리지 로그의 "스키마 버전 불일치" 경고 확인 |

---

## 9. 카메라 확장 (미구현 · 예약)

이번 단계는 로봇 통신만 다뤘다. 카메라 도입 시 **기존 필드를 바꾸지 않고 덧붙이는 방식**으로
확장한다(따라서 `v` 를 올리지 않아도 된다).

- `state` 에 `"camera": {"connected": bool, "focus_score": double}` 추가
- `event` 에 `code:"CAPTURE"` + `"view_index"`, `"image_path"` 추가
- **이미지 바이트는 5901 소켓으로 보내지 않는다.**
  1차 권장: Ubuntu 데이터셋 폴더를 SMB/NFS 로 공유하고 **경로만** 전달
  대안: 브리지에 HTTP `GET /image/<id>` 부가
- 연결 대상(Windows): `CaptureGallery.AddCapture`, `FocusScoreBadge.SetScore`, `FaceResultGrid`
- **미결 사항**: AI 추론(`chip_detection.exe`)을 Windows 에 남길지 Ubuntu 로 이관할지 —
  이미지 전송 방향이 여기서 갈린다

---

## 10. 남은 작업

1. **실물 TM5 연결 후 §7.3 전 항목 검증** (저속으로)
2. Windows 측 Visual Studio 첫 빌드 (§ROS2_BRIDGE_CLIENT.md)
3. `poses.yaml` 실물 티칭 — Tool 검사 시나리오(툴 아래에서 J6 회전)와
   `inspection_manager` 의 현재 rule 전략이 맞는지 확인
4. `safety_monitor` 독립 노드 분리 (현재 `robot_io_bridge` 이동 중 인라인 검사)
5. 카메라 부착 후 §9 확장

---

## 부록 A. 패키지 관리 규칙 준수 확인

`PROJECT_PLAN.md` 의 규칙을 모두 지켰다.

- ✅ `tmr_ros2/` 내부에 **어떤 파일도 생성/수정하지 않음**. `tm_msgs` 는 읽기 전용 의존성으로만 사용
- ✅ 신규 코드는 전부 `src/` 아래 기존 `tending_*` 패키지 내부
- ✅ 로봇 bringup 을 재구현하지 않고 tmr_ros2 공식 런치를 그대로 사용
- ✅ 신규 노드는 C++(rclcpp), launch 는 Python
- ✅ 문서는 한국어
