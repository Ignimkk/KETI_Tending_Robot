# 검사 1개 뷰(각도)에서 수집한 이미지 + 로봇 상태 동기 샘플
std_msgs/Header header

uint32 view_index          # 뷰 인덱스 (0..N-1)
float64 angle              # 검사 각도 (rad), 태스크프레임 기준
float64[] joint_pos        # 촬영 순간 관절값 (rad)
geometry_msgs/Pose tcp_pose# 촬영 순간 TCP 포즈 (base 기준)
string image_path          # 저장된 이미지 파일 경로
