# Tending — 엔드밀 로봇 비전 검사 시스템

TM5 로봇으로 상부 CNC 척에 매달린 엔드밀의 측면 360°를 촬영·검사하는 시스템.

## 문서 / 동기화 규칙
- **[PROJECT_PLAN.md](PROJECT_PLAN.md)** : 전체 실행 계획(통합 문서). `~/.claude/plans/end-mill-robotic-vision-functional-yao.md` 와 **항상 동일하게 유지**.
- **[../command.txt](../command.txt)** (`src/command.txt`) : 실행 명령어 모음. 노드/런치/파라미터 변경 시 갱신.
- 본 README : 패키지 역할 요약.

## 핵심 원칙
- **tmr_ros2 = 코어**: 로봇 bringup / MoveIt2 / 실물 연동은 tmr_ros2 공식 런치를 그대로 사용(`tm_bringup.launch.py`, `tm5-700_run_move_group.launch.py`). Gazebo 미사용.
- **tending_* = 보충**: tmr_ros2 에 없는 검사 제어·인터페이스·카메라·데이터. bringup 을 재구현하지 않음.

## 패키지 역할 (모두 `tmr_ros2` 외부, 읽기 전용 의존)
| 패키지 | 역할 |
|---|---|
| `tending_control` | 검사 제어 — `inspection_manager`, `scenario1_inspect`(rule/hybrid), `ee_pose_query`, `robot_io_bridge`/`pose_utils`. 검사 launch/config 소유 |
| `tending_moveit` | 시나리오1 MoveIt(충돌회피) 제어 `scenario1_inspect_moveit` (tmr_ros2 move_group 연결) |
| `tending_camera` | 카메라(부착 전 stub) `capture_image` |
| `tending_data` | 검사 데이터 저장 `dataset_recorder` |
| `tending_interfaces` | msg/srv/action |
| `tending_bringup` | (골격) 카메라·링라이트 부착 셀 bringup 예정 |
| `tending_description`/`tending_calibration`/`tending_bridge` | P1/P2 스캐폴드 |

## 빠른 시작
```bash
cd ~/dev_ws/prject/NCC/Tending_ws
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

# 로봇 bringup 은 tmr_ros2 로 (실물)
ros2 launch tm_driver tm_bringup.launch.py 172.21.43.12
# 시나리오1 rule 검사 (오프라인 미리보기는 dry_run:=true)
ros2 launch tending_control scenario1_inspect.launch.py control_mode:=rule dry_run:=false
```
자세한 명령은 `src/command.txt` 참조.

## ⚠️ 실물 배포 전
- `tending_control/config/poses.yaml`, `scenario1.yaml` 의 관절값/축좌표는 자리표시자 → TMflow 티칭/`ee_pose_query` 로 교체.
- TMflow Ethernet Slave / Listen Node 활성 + 프로젝트 실행 상태여야 `tm_driver` 연결.
