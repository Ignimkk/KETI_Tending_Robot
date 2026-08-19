# tending_bridge 와이어 프로토콜 v1

> Ubuntu(ROS 2) ↔ Windows(`KETI_CNCTendingRobot`) 양방향 통신 계약.
> **양측이 동시에 구현하는 기준 문서이므로, 변경 시 `schema_version` 을 올리고 양쪽을 함께 고친다.**

---

## 1. 전송 계층

| 항목 | 값 |
|---|---|
| 프로토콜 | TCP |
| 서버 | **Ubuntu** `tending_bridge` — 기본 바인드 `0.0.0.0:5901` |
| 클라이언트 | **Windows** `RosBridgeClient` (2초 간격 무한 재연결) |
| 인코딩 | UTF-8 |
| 프레이밍 | **개행(`\n`) 구분. JSON 1줄 = 1메시지.** 메시지 내부에는 개행을 넣지 않는다 |
| 원격 클라이언트 | Windows GUI 1대 (`allowed_client_ip`, 현재 `172.21.60.68`) |

로봇 제어 권한은 Ubuntu 본 PC의 로컬 ROS 호출과 허용된 Windows GUI 두 경로만 가진다.
TCP 디버그 클라이언트가 필요할 때는 실물 로봇을 분리한 상태에서 허용 IP를 임시 변경한다.
운영 기동 전 `ROS_LOCALHOST_ONLY=1`을 설정해 다른 LAN 호스트가 ROS 서비스/액션을 직접
호출하지 못하게 하고, Windows GUI는 DDS 대신 본 TCP 브리지만 사용한다.

포트 5901 은 TM 로봇의 TMSCT(5890)/TMSVR(5891) 과 겹치지 않도록 선택했다.

### 단위 규약

와이어는 **UI 네이티브 단위**를 쓰고, 변환은 브리지가 전담한다.

| 물리량 | 와이어 | ROS 내부 |
|---|---|---|
| 관절 각도 | **deg** | rad |
| 위치 | **mm** | m |
| 자세(RPY) | **deg** | rad |
| 속도(관절) | **deg/s** | rad/s |

근거: `RobotArmPanel.SetJointAngles(double[] jointsDeg)` 가 deg 를 받는다.

---

## 2. 공통 봉투

모든 메시지는 아래 필드를 갖는다.

```jsonc
{"v": 1, "type": "state|event|ack|result|cmd", "ts": 1754470000123}
```

- `v` : 스키마 버전(정수). 다르면 수신측은 경고 로그 후 무시한다.
- `ts` : Unix epoch **밀리초**.

---

## 3. Ubuntu → Windows

### 3.1 `state` — 로봇 상태 스트림

기본 **10 Hz** 주기 송신. 추가로 `link`/`safety` 의 bool 플래그가 바뀌면 즉시 1회 더 보낸다.
클라이언트 입장에서 **하트비트 역할을 겸한다.**

```jsonc
{
  "v": 1, "type": "state", "ts": 1754470000123,
  "link":   {"sct": true, "svr": true, "driver": true},
  "safety": {"e_stop": false, "robot_error": false, "safetyguard": false,
             "error_code": 0, "project_run": true},
  "joint_deg": [0.0, -20.1, 90.0, 0.0, 90.0, 0.0],
  "tcp": {"x": 400.0, "y": 0.0, "z": 300.0, "rx": 180.0, "ry": 0.0, "rz": 0.0},
  "job": {"active": true, "id": "a1b2c3", "cmd": "run_inspection",
          "phase": "INSPECT", "progress": 0.63, "captured": 5}
}
```

| 필드 | 출처 | 비고 |
|---|---|---|
| `link.sct` / `link.svr` | `feedback_states.is_sct_connected` / `is_svr_connected` | TM 드라이버 ↔ 로봇 |
| `link.driver` | 피드백 수신 여부 | `false` 면 `tm_driver` 미기동/응답없음 |
| `safety.*` | `feedback_states` 의 `e_stop / robot_error / safetyguard_a / error_code / project_run` | |
| `joint_deg` | `feedback_states.joint_pos` (rad→deg) | 길이 6 |
| `tcp` | `tool_pose` 토픽 (m→mm, quat→RPY deg) | base 기준 TCP |
| `job` | 활성 `run_inspection` goal | 없으면 `{"active": false}` |

`job.phase` 값: `INIT | APPROACH | INSPECT | RETREAT | HOME | DONE` (액션 feedback 그대로).

### 3.2 `event` — 안전/로그 알림

```jsonc
{"v": 1, "type": "event", "ts": 1754470000123,
 "level": "error", "code": "SAFETY_ESTOP", "message": "e-stop 감지, 검사 중단"}
```

`level` : `info | warn | error`

| `code` | 의미 |
|---|---|
| `SAFETY_ESTOP` | e-stop 감지 |
| `SAFETY_GUARD` | 안전문(safetyguard) 작동 |
| `ROBOT_ERROR` | 로봇 에러 플래그 (+`error_code`) |
| `DRIVER_LOST` | `feedback_states` 수신 끊김 |
| `DRIVER_OK` | 수신 복구 |
| `DEADMAN_STOP` | 클라이언트 무음으로 자동 정지 (§5.3) |
| `LOG` | 일반 로그 |

### 3.3 `ack` — 명령 즉시 응답

모든 `cmd` 에 대해 **반드시 1회** 보낸다. 수락 여부만 알린다.

```jsonc
{"v": 1, "type": "ack", "id": "<cmd 의 id>", "ok": true, "message": ""}
```

거부 사유 예: `"BUSY"`, `"UNKNOWN_CMD"`, `"BAD_ARGS"`, `"NOT_READY"`, `"SERVICE_UNAVAILABLE"`

### 3.4 `result` — 비동기 명령 종결

`ack{ok:true}` 로 수락된 **비동기 명령**(`run_inspection`, `goto_pose`, `jog`)이 끝나면 보낸다.

```jsonc
{"v": 1, "type": "result", "id": "<cmd 의 id>", "status": "SUCCEEDED",
 "success": true, "dataset_path": "/tmp/tending_dataset/mc_a",
 "image_count": 8, "message": ""}
```

`status` : `SUCCEEDED | ABORTED | CANCELED | REJECTED`
`dataset_path` / `image_count` 는 `run_inspection` 에만 의미가 있다.

---

## 4. Windows → Ubuntu

모든 명령은 `type:"cmd"` 이며 **클라이언트가 생성한 고유 `id`**(GUID 권장)를 갖는다.
응답(`ack`/`result`)은 이 `id` 로 매칭한다.

```jsonc
// 검사 실행 (비동기)
{"v":1,"type":"cmd","id":"<guid>","cmd":"run_inspection",
 "args":{"machine_id":"mc_a","scenario":1,
         "angle_start_deg":0.0,"angle_end_deg":360.0,
         "num_views":8,"inspect_distance_mm":100.0}}

// 실행 중인 명령 취소
{"v":1,"type":"cmd","id":"<guid>","cmd":"cancel","args":{"target_id":"<취소할 cmd id>"}}

// 명명 포즈 이동 (비동기)
{"v":1,"type":"cmd","id":"<guid>","cmd":"goto_pose",
 "args":{"pose_name":"home","velocity_deg_s":20.0}}
//   pose_name: home | init | approach | inspect | retreat

// 단일 관절 조그 (비동기)
{"v":1,"type":"cmd","id":"<guid>","cmd":"jog",
 "args":{"joint":6,"delta_deg":30.0,"velocity_deg_s":20.0}}
//   joint: 1..6, delta_deg: 현재 각도 기준 상대 증분

// 비상 정지 (동기, 최우선)
{"v":1,"type":"cmd","id":"<guid>","cmd":"stop","args":{}}

// 하트비트
{"v":1,"type":"cmd","id":"<guid>","cmd":"ping","args":{}}
```

---

## 5. 동작 규칙

### 5.1 `stop` 최우선

`stop` 은 수신 스레드에서 **명령 큐를 건너뛰고 즉시** 처리한다.
`tm_msgs/srv/SetEvent(STOP)` 을 직접 호출하고, 활성 goal 이 있으면 함께 cancel 한다.
지연 최소화를 위해 `inspection_manager` 를 우회하는 **유일한** 경로다.

### 5.2 모션 명령은 1개만 in-flight

`run_inspection` / `goto_pose` / `jog` 중 하나가 실행 중이면, 두 번째 모션 명령은
`ack{ok:false, message:"BUSY"}` 로 거부한다. `cancel` / `stop` / `ping` 은 항상 받는다.

### 5.3 데드맨 (연결 감시)

- Windows 는 **1초마다 `ping`** 을 보낸다.
- 브리지는 **현재 명령을 시작한 GUI**로부터 아무 메시지도 못 받은 시간이
  `client_timeout_ms`(기본 3000)를 넘거나 해당 GUI 연결이 끊기고
  **활성 goal 이 있으면** → goal cancel + `SetEvent(STOP)` 실행 후 `DEADMAN_STOP` 이벤트를 broadcast.
- 유휴 상태(goal 없음)에서는 타임아웃이 정지를 유발하지 않고 연결만 정리한다.

### 5.4 재연결

- Windows 클라이언트는 `rosBridgeReconnectMs`(기본 2000) 간격으로 무한 재시도한다.
- 연결이 끊긴 동안 UI 는 명령 버튼을 비활성화하고 상태 배너를 표시한다.
- 재연결 직후 브리지가 보내는 첫 `state` 로 UI 를 동기화한다. **명령 재전송은 하지 않는다**
  (끊긴 사이 로봇이 어떤 상태인지 보장할 수 없으므로 사용자가 다시 지시하게 한다).

### 5.5 명령 경로 단일화

`stop` 을 제외한 모든 명령은 `inspection_manager` 의 액션/서비스를 통과한다.
모션 소유자를 하나로 유지해 안전 검사(`safety_ok`)와 상태기계를 우회하지 못하게 한다.

---

## 6. 연결 직후 시퀀스

```
Windows                          Ubuntu(tending_bridge)
   │ ── TCP connect ──────────────> │
   │ <───────────── state (즉시 1회) │   현재 상태 스냅샷
   │ ── cmd ping (1s 주기) ────────> │
   │ <───────────── ack             │
   │ <───────────── state (10Hz)    │
```

---

## 7. 설정 (`config/bridge.yaml`)

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `bind_address` | `0.0.0.0` | 리스닝 주소 |
| `port` | `5901` | 리스닝 포트 |
| `allowed_client_ip` | `172.21.60.68` | 접속을 허용할 Windows GUI IPv4; 빈 값은 전체 허용 |
| `state_rate_hz` | `10.0` | `state` 송신 주기 |
| `client_timeout_ms` | `3000` | 데드맨 타임아웃 |
| `max_clients` | `1` | 원격 GUI 한 대만 허용 |
| `schema_version` | `1` | 봉투의 `v` 값 |

---

## 8. 카메라 확장 (미구현 · 예약)

이번 단계는 로봇 통신만 다룬다. 카메라 도입 시 아래를 **덧붙이는 방식**으로 확장한다
(기존 필드는 바꾸지 않으므로 `v` 는 유지 가능).

- `state` 에 `"camera": {"connected": bool, "focus_score": double}` 블록 추가
- `event` 에 `code:"CAPTURE"` + `"view_index"`, `"image_path"` 필드 추가
- **이미지 바이트는 이 소켓으로 보내지 않는다.** Ubuntu 데이터셋 폴더를 SMB/NFS 로 공유하고
  경로만 전달하는 방식이 1차 권장안 (대안: 브리지에 HTTP `GET /image/<id>` 부가)
