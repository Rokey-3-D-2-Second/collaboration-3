# 디지털 트윈 기반 서비스 로봇 운영 시스템 구성

## 프로젝트 개요

- **목표**: 
- **주요 기능**: 
- **사용 장비**: turtlebot3
- **개발 환경**: Ubuntu 22.04, ROS2 Humble
- **주요 기술 스택**: ROS2, Gazebo, OpenCV
- **기간**: 2025.06.23 ~ 2025.07.04

## 시연 영상

- 영상 링크: [Demo Video](https://youtu.be/pgqLOggiihU)

<div align="center">

[Demo Video](https://github.com/user-attachments/assets/fda8426f-27c1-484d-97ba-ff309782b422)

</div>

## 다이어그램

<div align="center">

![협동-3 1주차 다이어그램 drawio](https://github.com/user-attachments/assets/30f80edf-bb54-403e-8b98-8cfb8fafcfc0)

</div>

## 상세설명

### 문제정의

- 복잡한 도로 환경에서 자율주행 서비스 로봇 운영의 신뢰성·안정성 확보 필요.
- 차선·신호·교차로·차단기 등 다양한 교통 요소 인식의 난이도.
- 실제 환경 변동(빛, 그림자, 노이즈 등) 및 로봇 간 협동의 어려움.

### 해결방안

- HSV/Canny+Hough/SIFT 등 다중 알고리즘 기반 차선·교통 표지·신호 인식 구현.
- ROI, MIN_MATCH_COUNT, MIN_MSE_DECISION 등 매개변수 최적화로 오검출/노이즈 대응.
- FSM(유한상태기계) 기반 로봇 상태관리, 다양한 감지·주행 로직 모듈화.
- Trajectory Following(리더-팔로워) 등 다로봇 실시간 협동 및 waypoint 기반 경로 추종.

### 주요기능

- **차선 감지:** HSV(색상 기반 곡선 감지), Canny+Hough(직선 검출), CTE(중앙선 추정) 통합.
- **교통 표지/신호 감지:** SIFT(특징점 기반), ROI/MATCH_COUNT/MSE 기반 오탐 개선.
- **교차로/차단기 상황 대응:** ROI, Blob 파라미터 최적화, 각 환경별 맞춤 처리.
- **PID 기반 주행 제어:** 실시간 속도/방향 보정, 안정적인 자율주행 구현.
- **FSM 상태 관리:** 정지/좌회전/우회전 등 이벤트 기반 동작 전이, 오류/예외 최소화.
- **리더-팔로워 협동 주행:** 실시간 waypoint 공유, lookahead, 저역통과 필터 기반 부드러운 추종, 안전거리 유지.
- **통합 시스템 및 실증:** Gazebo 시뮬레이터·실환경 모두에서 통합 실증, 오픈소스 기반 확장성 확보협동-3 발표자료.

## 프로젝트 기여자

- 문승연: opm0508@naver.com
- 이요셉: rheejoseph54@gmail.com
- 이호준: hojun7889@gmail.com
- 홍지원: jw1122.h@gmail.com

## 교육과정 및 참고자료

### 교육과정

<div align="center">

| 주차 | 기간 | 구분 | 강의실 |
| --- | --- | --- | --- |
| <6주차> | 2025.06.23(월) ~ 2025.06.27(금) | 협동-3 | * 별관 : C-4호 |

| 차시 | 학습내용 | 세부항목 |
| --- | --- | --- |
| 1 | RVIZ, RQT, MoveIt |  |
| 2 | Gazebo |  |
| 3 | 자율주행1 |  |
| 4 | 자율주행2 |  |
| 5 | 1주차 프로젝트 발표 | 1주차 프로젝트 발표 |
| 6 | 프로젝트 설계 | 시스템 설계 및 환경 구성 |
| 7 | 개발 | 기능 구현 및 Unit Test |
| 8 | 개발 | 기능 구현 및 Unit Test |
| 9 | 개발 | 기능 구현 및 Unit Test |
| 10 | 프로젝트 발표 | 프로젝트 발표 및 산출물 정리 |

</div>

### 참고자료

- https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/
- https://github.com/Rokey-3-D-2-Second/turtlebot3_autorace/tree/humble
- https://github.com/Rokey-3-D-2-Second/turtlebot3_simulations/tree/humble
