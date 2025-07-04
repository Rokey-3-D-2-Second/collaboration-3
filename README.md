# 디지털 트윈 기반 서비스 로봇 운영 시스템 구성

## 프로젝트 개요

- **목표**: 복잡한 도로 환경에서 자율주행 로봇이 차선·신호·표지판 등 다양한 교통상황을 신뢰성 있게 인식·대응하며, 실시간 협동주행 및 안정적 서비스 제공
- **주요 기능**: 차선 감지, 교통 표지/신호 감지, 교차로/차단기 상황 대응, PID 기반 주행 제어, FSM 상태 관리, 리더-팔로워 협동 주행
- **사용 장비**: turtlebot3
- **개발 환경**: Ubuntu 22.04, ROS2 Humble
- **주요 기술 스택**: ROS2, Gazebo, OpenCV
- **기간**: 2025.06.23 ~ 2025.06.27

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
</div>

### 참고자료

- https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/
- https://github.com/Rokey-3-D-2-Second/turtlebot3_autorace/tree/humble
- https://github.com/Rokey-3-D-2-Second/turtlebot3_simulations/tree/humble

<br>

---

<br>

# ROADY: 다목적 자율주행 로봇의 ArUco 마커 기반, 도로 낙하물 자동 수거 서비스

디지털 트윈 기반 서비스 로봇 운영 시스템 구성

## 프로젝트 개요

- **목표**: 다목적 자율주행 로봇의 ArUco 마커 기반, 도로 낙하물 자동 수거 서비스
- **주요 기능**: 차선 감지, PID 기반 주행 제어, FSM 상태 관리 + Database, Aruco Marker 감지 및 manipulator을 활용한 수거
- **사용 장비**: turtlebot3 waffle pi(Open CR, Jetson Nano), OpenMANIPULATOR-X
- **개발 환경**: Ubuntu 22.04, ROS2 Humble
- **주요 기술 스택**: ROS2, Moveit, OpenCV, SQLite
- **기간**: 2025.06.30 ~ 2025.07.04

## 시연 영상

- 영상 링크: [Demo Video](https://youtu.be/scLdyGJXrE8)

<div align="center">

[Demo Video](https://github.com/user-attachments/assets/fcc3915f-7e03-40fe-9db0-ac256dae2819)

</div>

## 다이어그램

<div align="center">

![협동-3 2주차 FSMdrawio](https://github.com/user-attachments/assets/14da158e-594f-44cc-a9e8-cef59bc26276)
![협동-3 2주차 Flowchartdrawio](https://github.com/user-attachments/assets/5d751ad9-888a-445c-ba2e-1892dfa8e356)

</div>

## 상세설명

### 문제정의

- **도로 위 낙하물**(돌, 파편, 맨홀 뚜껑 등)로 인한 차량 손상 및 2차 사고가 증가하고 있음.
- 2022~2024년 전기차 배터리 손상 사고 405건 중 172건(약 43%)이 도로 위 돌출물/낙하물과의 충돌이 원인.
- **기존 수거 방식**은 작업자가 위험에 직접 노출되고, 수작업 과정에서 실시간 기록·공유가 미흡.
- 낙하물 발생 시 **정확한 위치 파악 및 신속 대응**의 어려움이 있음.

### 해결방안

- **자율주행 로봇 및 매니퓰레이터**(Turtlebot3 + OpenMANIPULATOR-X)을 활용해 낙하물을 자동 탐지 및 수거.
- **ArUco 마커** 등 비전기술을 활용해 로봇이 낙하물 위치를 자동 인식·접근.
- **수거 및 상태 기록**을 실시간 데이터로 관리 및 모니터링(데이터베이스 연동).
- **차선 인식/추종** 기반 자율주행과 FSM(유한 상태 기계)로 상태별 동작 자동화.
- **작업자 안전 확보**: 수작업을 대체해 도로 위 작업자의 위험 노출 최소화.
- **사고·손상 예방**: 빠른 탐지 및 수거로 차량 손상·2차 사고 예방.

### 주요기능

- **차선 인식 기반 자율주행**
  - 카메라 실시간 영상에서 노란색/흰색 차선 검출, PID 제어로 조향 및 속도 결정.
  - 다양한 이미지 전처리 기법(HSV 변환, 평탄화, 마스킹, 블러 등)으로 외부 환경(빛, 그림자 등) 변화에 대응.
- **ArUco 마커 인식 및 PnP**(pick & place)
  - ArUco 마커를 인식하여 낙하물 위치 자동 탐지 및 로봇팔로 접근.
  - 카메라 캘리브레이션, 좌표 변환으로 정확한 위치 판단.
- **FSM(유한상태기계) + Database 연동**
  - 로봇의 상태(정지, 차선 추종, 탐색, 수거 등)를 상태기계로 관리.
  - 모든 상태 변화 및 제어 기록을 DB에 저장(분석 및 튜닝).
  - 주행/제어 파라미터를 DB에서 실시간 수정 가능(속도, PID 계수 등).
  - 상태 진입 횟수, 체류 시간 등 통계 정보 자동 축적(성능 분석/최적화 활용).
- **이미지 처리 및 실시간 튜닝**
  - 자동 노출, 화이트 밸런스, 감마 조절로 다양한 시간대·날씨에 대응.
  - 동적 마스킹, 경계선 강조, 잡음 제거 등 robust image pipeline.
- **통합 주행 및 수거 프로세스**
  - 차선 추종 → 마커 인식 → 낙하물 접근 → pick & place(수거) → 상태 기록·모니터링까지 전 자동화.

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
| <7주차> | 2025.06.30(월) ~ 2025.07.04(금)	| 협동-3	| * 별관 : C-4호 |

| 차시 | 학습내용 | 세부항목 |
| --- | --- | --- |
| 6 | 프로젝트 설계 | 시스템 설계 및 환경 구성 |
| 7 | 개발 | 기능 구현 및 Unit Test |
| 8 | 개발 | 기능 구현 및 Unit Test |
| 9 | 개발 | 기능 구현 및 Unit Test |
| 10 | 프로젝트 발표 | 프로젝트 발표 및 산출물 정리 |

</div>

### 참고자료

- https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/
- https://github.com/Rokey-3-D-2-Second/turtlebot3_autorace2/tree/humble
- https://github.com/Rokey-3-D-2-Second/collaboration-3/tree/main/aruco_yolo
- https://github.com/Rokey-3-D-2-Second/collaboration-3/tree/main/turtlebot_moveit

