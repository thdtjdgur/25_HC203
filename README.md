## 💡1. 프로젝트 개요
1-1. 프로젝트 소개
- 프로젝트 명: 메디웨이(MediWay) - 업무 보조 및 병원 내 길안내 로봇
- 프로젝트 정의: 의약품 배달·호출 시 위치 이동·내비 제공 로봇을 개발해 스마트 병원 구축을 위한 로봇

1-2. 개발 배경 밒 필요성
- 현재 병원에서는 의료진과 직원들이 직접 의약품을 운반하거나, 환자가 병원 내부에서 길을 찾는 데 어려움을
  겪는 경우가 많다. 이러한 문제를 해결하기 위해 실내 길찾기 어플이나 길안내 로봇이 있느나, 각각 접근성이
  어렵다는 점과 기능이 하나로 제한된다는 점을 개선하기 위해 본 프로젝트를 구상하였다.

1-3. 프로젝트 특장점
- AI 필터링(칼만·PF)과 다중 BLE 비콘 삼각측량으로 실시간 위치 오차 ±30 cm 달성
- 호출 시 사용자 위치로 이동해 내비게이션/안내 제공(맥락 기반 동선 최적화)
- ROS 기반 경로 최적화·PID 제어·LiDAR 장애물 회피로 병원 내 완전 자율 주행
- 서보모터를 통한 맞춤 의약품 선별·적재·배달로 수작업 대체

1-4. 주요기능
- 실시간 위치추적 : BLE 핑거프린팅과 IMU(칼만·EKF) 융합으로 정밀 위치 산출, 경로 시각화
- 자율 주행 내비게이션 : AMCL로 위치 파악, A* 경로 생성, LiDAR 기반 이동 장애물 회피
- 자동 의약품 배달 : 호출 시 사용자 위치로 이동, 미호출 시 지정 구간 왕복 운행
- 사용자 안내 디스플레이 : 직관 UI로 실시간 위치·방향·이동 경로 제공
- 센서 융합 주행 안정화 : LiDAR·myAHRS+·엔코더 통합으로 주행 신뢰성 향상
- 하드웨어 업그레이드 : 고기어비 DC 모터 적용으로 주행 성능 개선

1-5. 기대 효과 및 활용 분야
- 기대 효과 : 의료진 반복 운반 업무 자동화로 이동 시간 절감(하루 최대 2시간, 연간 약 500시간),
  즉시 길안내·신속 전달로 환자·의료진 만족도 향상, 서보모터 기반 선별로 오배송·지연 오류 감소
- 활용 분야 : 병동 내 의약품·검체·식자재 정기/긴급 배달, 환자·방문객 실내 길안내, 원내 순찰·소모품
  점검 보조, 교육·데모(수업·전시) 플랫폼, 요양원·재활센터 등 의료·복지 시설로의 배달·안내 서비스 확장

1-6. 기술 스택
- 로봇/미들웨어 : ROS Noetic (Catkin, ROS msg pub/sub, Service, Action), RViz
- 운영체제 : Ubuntu 20.04 LTS, Raspberry Pi OS
- 개발환경/IDE : Visual Studio Code, Git
- 개발언어 : C++, Python
- 센서 드라이버·프로토콜 : LiDAR/IMU ROS 드라이버, EKF/칼만 필터 파이프라인
- 하드웨어(디바이스) : NVIDIA Jetson Nano, Raspberry Pi 4B, Arduino Mega, RA35GM(DC Motor)
- 센서 : YDLIDAR, myAHRS+, Minew BLE Indoor Beacon
- 통신 : Wi-Fi (UDP), BLE
- 프로젝트 관리·협업 : Git, KakaoTalk, Zoom, Slack
- 기타 : 구동 휠 지름 10 cm


## 💡2. 팀원소개
![개발자 소개](https://github.com/thdtjdgur/25_HC203/blob/main/%EC%97%AD%ED%95%A0%EB%B6%84%EB%8B%B4.png)


## 💡3. 시스템 구성도
3.1. 하드웨어 설계도

<img src="https://github.com/thdtjdgur/25_HC203/blob/main/%ED%95%98%EB%93%9C%EC%9B%A8%EC%96%B4%20%EC%84%A4%EA%B3%84%EB%8F%84.png" width="500"/>

3.2. 주행 서비스 흐름도

<img src="https://github.com/thdtjdgur/25_HC203/blob/main/%EC%A3%BC%ED%96%89%20%EC%84%9C%EB%B9%84%EC%8A%A4%20%ED%9D%90%EB%A6%84%EB%8F%84.png" width="500"/>

3.3. 디스플레이 서비스 흐름도

<img src="https://github.com/thdtjdgur/25_HC203/blob/main/%EB%94%94%EC%8A%A4%ED%94%8C%EB%A0%88%EC%9D%B4%20%EC%84%9C%EB%B9%84%EC%8A%A4%20%ED%9D%90%EB%A6%84%EB%8F%84.png" width="500"/>

