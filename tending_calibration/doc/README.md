# tending_calibration [P1 스캐폴드]

카메라 부착(≈2026-08-10) 후 구현.

## 목표
- 외부 산업용 카메라 내부 파라미터 캘리브레이션(선행)
- eye-in-hand 핸드아이 캘리브레이션 → `flange → camera_optical` 변환 산출
- TCP 를 카메라 광학중심에 설정 (TMflow TCP 또는 send_script `ChangeTCP`)

## 계획 노드
- `handeye_collect` : ChArUco/체스보드 타깃을 여러 로봇 포즈에서 촬영, (플랜지 포즈, 이미지) 쌍 수집
- `handeye_solve`   : `cv::calibrateHandEye` 로 변환 계산, 재투영오차 검증, `handeye.yaml` 저장

## 산출물
- `tending_bringup/config/handeye.yaml`
