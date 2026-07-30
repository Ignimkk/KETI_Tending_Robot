cmake_minimum_required(VERSION 3.5)
project(tending_bridge)

find_package(ament_cmake REQUIRED)

# [P2 스캐폴드] TendingSystem(C#) 연동 브리지 — 후순위.
#   - tending_bridge(tending_bridge/src/tending_bridge.cpp) : RunInspection 결과/InspectionSample/
#     로봇·검사 상태를 구독 → 외부 전송(경계 노드)
#   - 전송 방식 1차 권장: 경량 TCP/JSON 소켓 서버 (C# 커스텀 클라이언트), 대안: rosbridge websocket
#   - 의존(구현 시): rclcpp, tending_interfaces
# 현재는 빌드 대상 없음(placeholder).

install(DIRECTORY doc
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL)

ament_package()
