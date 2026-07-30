# tending_bridge [P2 후순위 스캐폴드]

C# 기반 `TendingSystem` GUI 로 검사 데이터를 전송하는 경계 노드.

## 목표
- `run_inspection` 결과, `inspection_sample`, 로봇/검사 상태를 GUI 로 전송
- (필요 시) GUI → 검사 시작/중지 명령 수신

## 전송 방식
- 1차 권장: **경량 TCP/JSON 소켓 서버** — 의존성 최소, C# 연동 용이
- 대안: **rosbridge_suite(websocket + JSON)** + C# `ROS#`

## 전송 스키마(초안)
```json
{
  "machine_id": "mc_a", "scenario": 1,
  "dataset_path": "/tmp/tending_dataset/mc_a", "image_count": 8,
  "views": [
    {"index": 0, "angle": 0.0, "tcp_pose": [x,y,z, qx,qy,qz,qw],
     "joint_pos": [j1..j6], "stamp": 0.0, "image_path": "..."}
  ],
  "result": {"success": true, "message": ""}
}
```

## 설정
- `tending_bridge/config/bridge.yaml` : host/port/스키마 버전
