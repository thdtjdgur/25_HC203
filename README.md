# 💡1. 프로젝트 개요

<p align="center">
  <img src="https://img.shields.io/badge/PLATFORM-JETSON%20NANO%20%7C%20RASPBERRY%20PI%204B-25344F?style=for-the-badge&labelColor=555555" alt="Platform: Jetson Nano and Raspberry Pi 4B" />
  <img src="https://img.shields.io/badge/SOFTWARE-C%2B%2B%20%2B%20PYTHON%20%2B%20ROS-0C8D7B?style=for-the-badge&labelColor=555555" alt="Software: C++, Python, and ROS" />
  <img src="https://img.shields.io/badge/PROJECT-MEDIWAY-C65D00?style=for-the-badge&labelColor=555555" alt="Project: MediWay" />
  <img src="https://img.shields.io/badge/NAVIGATION-LIDAR%20%2B%20BLE-1F5FDB?style=for-the-badge&labelColor=555555" alt="Navigation: LiDAR and BLE" />
</p>

## 1-1. 프로젝트 소개
- 프로젝트 명: 메디웨이(MediWay) - 업무 보조 및 병원 내 길안내 로봇
- 프로젝트 정의: 의약품 배달·호출 시 위치 이동·내비 제공 로봇을 개발해 스마트 병원 구축을 위한 로봇

## 1-2. 개발 배경 및 필요성
- 현재 병원에서는 의료진과 직원들이 직접 의약품을 운반하거나, 환자가 병원 내부에서 길을 찾는 데 어려움을
  겪는 경우가 많다. 이러한 문제를 해결하기 위해 실내 길찾기 어플이나 길안내 로봇이 있느나, 각각 접근성이
  어렵다는 점과 기능이 하나로 제한된다는 점을 개선하기 위해 본 프로젝트를 구상하였다.

## 1-3. 프로젝트 특장점
- AI 필터링(칼만·PF)과 다중 BLE 비콘 삼각측량으로 실시간 위치 오차 ±30 cm 달성
- 호출 시 사용자 위치로 이동해 내비게이션/안내 제공(맥락 기반 동선 최적화)
- ROS 기반 경로 최적화·PID 제어·LiDAR 장애물 회피로 병원 내 완전 자율 주행
- 서보모터를 통한 맞춤 의약품 선별·적재·배달로 수작업 대체

## 1-4. 주요기능
- 실시간 위치추적 : BLE 핑거프린팅과 IMU(칼만·EKF) 융합으로 정밀 위치 산출, 경로 시각화
- 자율 주행 내비게이션 : AMCL로 위치 파악, A* 경로 생성, LiDAR 기반 이동 장애물 회피
- 자동 의약품 배달 : 호출 시 사용자 위치로 이동, 미호출 시 지정 구간 왕복 운행
- 사용자 안내 디스플레이 : 직관 UI로 실시간 위치·방향·이동 경로 제공
- 센서 융합 주행 안정화 : LiDAR·myAHRS+·엔코더 통합으로 주행 신뢰성 향상
- 하드웨어 업그레이드 : 고기어비 DC 모터 적용으로 주행 성능 개선

## 1-5. 기대 효과 및 활용 분야
- 기대 효과 : 의료진 반복 운반 업무 자동화로 이동 시간 절감(하루 최대 2시간, 연간 약 500시간),
  즉시 길안내·신속 전달로 환자·의료진 만족도 향상, 서보모터 기반 선별로 오배송·지연 오류 감소
- 활용 분야 : 병동 내 의약품·검체·식자재 정기/긴급 배달, 환자·방문객 실내 길안내, 원내 순찰·소모품
  점검 보조, 교육·데모(수업·전시) 플랫폼, 요양원·재활센터 등 의료·복지 시설로의 배달·안내 서비스 확장

## 1-6. 기술 스택
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


# 💡2. 팀원소개
![개발자 소개](https://github.com/thdtjdgur/25_HC203/blob/main/%EC%97%AD%ED%95%A0%EB%B6%84%EB%8B%B4.png)


# 💡3. 시스템 구성도
## 3.1. 하드웨어 설계도

<img src="https://github.com/thdtjdgur/25_HC203/blob/main/%ED%95%98%EB%93%9C%EC%9B%A8%EC%96%B4%20%EC%84%A4%EA%B3%84%EB%8F%84.png" width="500"/>



## 3.2. 주행 서비스 흐름도

<img src="https://github.com/thdtjdgur/25_HC203/blob/main/%EC%A3%BC%ED%96%89%20%EC%84%9C%EB%B9%84%EC%8A%A4%20%ED%9D%90%EB%A6%84%EB%8F%84.png" width="500"/>

1. Arduino mega와 ros시리얼통신을 통해 엔코더값을 전달받는다.
2. 엔코더, imu(myahrs+), ydlidar센서 융합을 통해 로봇의 위치를 갱신한다.
3. 기존 병원의 지도를 불러와 라이다의 센서값을 통해 amcl알고리즘으로 로봇의 위치를 파악한다.
4. 라즈베리파이로부터 상황에 따라 사용자위치, 진료실위치, 약품배달위치 좌표를 전달받고 목표위치를 파악 한 후  moe_base로 전달한다.
5. 목표위치와 현재 로봇의 위치를 바탕으로 병원에서의 로봇의 위치를 파악한다.
6. 라이다를 통해 실시간으로 장애물 회피경로를 생성한다.
7. 생성된 geometry_msgs/Twist 데이터를 모터에 전달하여 주행한다.



## 3.3. 디스플레이 서비스 흐름도

<img src="https://github.com/thdtjdgur/25_HC203/blob/main/%EB%94%94%EC%8A%A4%ED%94%8C%EB%A0%88%EC%9D%B4%20%EC%84%9C%EB%B9%84%EC%8A%A4%20%ED%9D%90%EB%A6%84%EB%8F%84.png" width="500"/>

1. BLE beacon을 통해 미리 병원 전체의 신호 데이터베이스를 저장하여 핑거프린팅 맵을 만든다.
2. 실시간으로 위치추적 시 비콘에게서 RSSI 신호를 받고, 핑거프린팅 기법을 통해 현재 위치 예측한다.
3. IMU센서의 PDR기법을 통해 걸음 수와 방향을 검출한다.
4. IMU센서의 데이터와 비콘의 데이터를 받아 EKF를 통해 융합한다.
5. EKF의 위치 정보와 IMU의 방향정보를 지도에 표시한다.


# 💡4. 작품 소개영상
[![한이음 메디웨이 썸네일](https://github.com/thdtjdgur/25_HC203/blob/main/%ED%95%9C%EC%9D%B4%EC%9D%8C%20%EB%A9%94%EB%94%94%EC%9B%A8%EC%9D%B4%20%EC%8D%B8%EB%84%A4%EC%9D%BC.png)](https://www.youtube.com/watch?si=7n6Uzyxs3dZalsro&v=azfa6Psx8fo&feature=youtu.be)

# 💡5. 핵심 소스코드
- 라즈베리파이와 ROS 노드 간 UDP 통신을 통해 좌표, 신호를 주고받으며 로봇을 제어한다.
- 오도메트리와 라이다/IMU 데이터를 TF 변환해 로봇의 현재 위치를 추적한다.
- 상태머신(사용자 호출, 진료실 이동, 약품 배달 등)에 따라 목표 좌표를 설정하고 네비게이션을 수행한다.
```
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
mediway1 노드
라즈베리파이 픽셀(u,v) → 미터(X,Y)
RViz Marker / /move_base_simple/goal /clicked_position
"""

import socket
import rospy
from geometry_msgs.msg import Point, PoseStamped, Pose, Quaternion
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32, Header

import math
import tf2_ros
import tf2_geometry_msgs

# ───────── 파라미터 & 상수 ─────────
PIX_W, PIX_H = 762.0, 574.0
lu_x, lu_y = 1.0, -1.8
ld_x, ld_y = 3.7, -0.3
ru_x, ru_y = -1.0, 1.7
rd_x, rd_y = 1.5, 3.2

RPI_IP = "10.24.184.1"     # ← 실제 IP
RPI_PORT = 5005            # ← RPi 수신 포트(좌표/신호)
RPI_PORT_SIGNAL = 5006     # ← 999,999 등 신호 분리용(원치 않으면 RPI_PORT와 동일하게 써도 됨)

GOAL_THRESH = 0.35

# 약품 위치(실좌표로 교체 가능)
MED1_GOAL = (1.1, -1.1)
MED2_GOAL = (3.0, -0.1)

# 상태 정의
STATE_IDLE          = "IDLE"
STATE_TO_USER       = "TO_USER"        # 사용자에게 이동 중
STATE_AT_USER       = "AT_USER"        # 사용자 도착(진료실 좌표 대기)
STATE_TO_CLINIC     = "TO_CLINIC"      # 진료실 이동 중 (RPi가 좌표 보냄)
STATE_TO_MED1       = "TO_MED1"        # 약1 이동 중 (RPi 좌표 없음)
STATE_TO_MED2       = "TO_MED2"        # 약2 이동 중 (RPi 좌표 없음)
STATE_WAIT_INPUT    = "WAITING_FOR_INPUT"
STATE_DELIVERING    = "DELIVERING"

# ───────── 전역 변수 ─────────
robot_x, robot_y = 0.0, 0.0
goal_x, goal_y   = 0.0, 0.0
state = STATE_IDLE

# 사용자 도착 신호 1회 송신용
user_signal_sent = False

# 약품 배달 단계(현재 타깃): True면 약1, False면 약2
medicine_target_is_med1 = True

# 약품 투입 중 들어온 사용자 호출 보류 용
pending_user_call = False
pending_u = None
pending_v = None

# 소켓
pos_sock = None  # 로봇 좌표 TX
tx_sock  = None  # 신호 TX

# TF
_tf_buffer = None
_tf_listener = None


def pixel_to_meter(u: float, v: float):
    s = u / PIX_W
    t = v / PIX_H
    X = ((1-s)*(1-t)*lu_x + s*(1-t)*ru_x + (1-s)*t*ld_x + s*t*rd_x)
    Y = ((1-s)*(1-t)*lu_y + s*(1-t)*ru_y + (1-s)*t*ld_y + s*t*rd_y)
    return X, Y


def send_pose_timer_cb(event):
    """0.5초마다 로봇 좌표를 RPi로 전송 (항상 송신: RPi가 필요할 때만 사용)"""
    global robot_x, robot_y, pos_sock
    if pos_sock is None:
        return
    try:
        msg = f"{robot_x:.3f},{robot_y:.3f}".encode()
        pos_sock.sendto(msg, (RPI_IP, RPI_PORT))
    except Exception as e:
        rospy.logwarn(f"[POSE_TX] UDP send failed: {e}")


def odom_callback(msg: Odometry):
    """odom→map 최신 시각 변환 (extrapolation 방지), 콜백 내 sleep 금지"""
    global robot_x, robot_y, _tf_buffer
    odom_pose = msg.pose.pose
    try:
        transform = _tf_buffer.lookup_transform(
            "map",
            msg.header.frame_id,   # 보통 "odom"
            rospy.Time(0),         # 최신
            rospy.Duration(0.2)
        )
    except Exception as ex:
        rospy.logwarn(f"Transform failed: {ex}")
        return

    map_pose_st = tf2_geometry_msgs.do_transform_pose(
        PoseStamped(header=msg.header, pose=odom_pose),
        transform
    )
    robot_x = map_pose_st.pose.position.x
    robot_y = map_pose_st.pose.position.y


def distance_to_goal() -> float:
    return math.hypot(robot_x - goal_x, robot_y - goal_y)


def set_goal(x, y):
    global goal_x, goal_y
    goal_x, goal_y = x, y


def udp_listener():
    global _tf_buffer, _tf_listener
    global pos_sock, tx_sock
    global state, goal_x, goal_y
    global user_signal_sent
    global medicine_target_is_med1
    global pending_user_call, pending_u, pending_v

    rospy.init_node("mediway1")

    point_pub  = rospy.Publisher("/clicked_position", Point, queue_size=10)
    goal_pub   = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)
    marker_pub = rospy.Publisher("/goal_marker", Marker, queue_size=10)
    servo_pub  = rospy.Publisher("/servo_control", Int32, queue_size=10)

    # TF
    _tf_buffer = tf2_ros.Buffer()
    _tf_listener = tf2_ros.TransformListener(_tf_buffer)
    rospy.Subscriber("/odom", Odometry, odom_callback, queue_size=1)

    # UDP
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", 5005))
    sock.settimeout(2)  # 2초 안에 오면 got_udp=True (전제 유지)

    tx_sock  = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    pos_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rospy.Timer(rospy.Duration(0.5), send_pose_timer_cb)  # 0.5초 주기 좌표 TX

    rospy.loginfo("UDP 5005 listening (mediway1)")
    rate = rospy.Rate(50)

    # 초기 상태: 약1부터 시작
    set_goal(*MED1_GOAL)
    state = STATE_TO_MED1
    rospy.loginfo("약품1 배달 지점으로 이동중(초기)")

    while not rospy.is_shutdown():
        # 1) RPi 좌표 수신
        got_udp = False
        u = v = None
        try:
            data, _ = sock.recvfrom(1024)
            u_str, v_str = data.decode().strip().split(",")
            u, v = float(u_str), float(v_str)
            got_udp = True
        except socket.timeout:
            pass
        except (ValueError, IndexError):
            rospy.logwarn("잘못된 UDP 데이터 형식 수신")

        # 1.5) 약품 이동 중 사용자 호출 즉시 전환
        if got_udp and state in [STATE_TO_MED1, STATE_TO_MED2]:
            set_goal(*pixel_to_meter(u, v))
            state = STATE_TO_USER
            user_signal_sent = False
            pending_user_call = False
            rospy.loginfo("사용자 호출 수신 → 사용자에게로 즉시 전환")

        # 1.6) 약품 투입 대기/배출 중 호출은 보류
        if got_udp and state in [STATE_WAIT_INPUT, STATE_DELIVERING]:
            pending_user_call = True
            pending_u, pending_v = u, v
            rospy.loginfo("사용자 호출 수신(약품 투입 중) → 종료 후 사용자로 전환 예약")

        # 2) 상태 머신 본 처리
        if state == STATE_TO_MED1:
            if distance_to_goal() < GOAL_THRESH:
                rospy.loginfo("약품1 배달 지점 도착!")
                state = STATE_WAIT_INPUT
                # 정지
                goal_pub.publish(PoseStamped(
                    header=Header(stamp=rospy.Time.now(), frame_id="map"),
                    pose=Pose(position=Point(robot_x, robot_y, 0), orientation=Quaternion(0,0,0,1))
                ))
            else:
                rospy.loginfo("약품1 배달 지점으로 이동중")

        elif state == STATE_TO_MED2:
            if distance_to_goal() < GOAL_THRESH:
                rospy.loginfo("약품2 배달 지점 도착!")
                state = STATE_WAIT_INPUT
                goal_pub.publish(PoseStamped(
                    header=Header(stamp=rospy.Time.now(), frame_id="map"),
                    pose=Pose(position=Point(robot_x, robot_y, 0), orientation=Quaternion(0,0,0,1))
                ))
            else:
                rospy.loginfo("약품2 배달 지점으로 이동중")

        elif state == STATE_WAIT_INPUT:
            # 블로킹 입력(현장에선 서비스/액션 권장)
            try:
                user_input = input("약품 배달 지점 도착. 약품 번호(1-4)를 입력하고 Enter: ")
                servo_num = int(user_input)
                if 1 <= servo_num <= 4:
                    rospy.loginfo(f"{servo_num}번 약품 배출 시작.")
                    servo_pub.publish(servo_num)
                    state = STATE_DELIVERING
                else:
                    print("잘못된 번호입니다. 1~4 사이 숫자를 입력하세요.")
            except ValueError:
                print("숫자를 입력해주세요.")

        elif state == STATE_DELIVERING:
            rospy.loginfo("약품 배출 중... (약 6초간 대기)")
            rospy.sleep(3.0)
            rospy.loginfo("약품 배출 완료.")

            # 배출 중 호출이 들어왔다면 최우선 전환
            if pending_user_call:
                set_goal(*pixel_to_meter(pending_u, pending_v))
                state = STATE_TO_USER
                user_signal_sent = False
                pending_user_call = False
                rospy.loginfo("배출 완료 → 예약된 사용자 호출로 전환")
            else:
                # 다음 약품 지점으로 즉시 이동 시작
                if medicine_target_is_med1:
                    # 방금 약1 처리 → 약2로
                    medicine_target_is_med1 = False
                    set_goal(*MED2_GOAL)
                    state = STATE_TO_MED2
                    rospy.loginfo("약품2 배달 지점으로 이동 시작")
                else:
                    # 방금 약2 처리 → 약1로
                    medicine_target_is_med1 = True
                    set_goal(*MED1_GOAL)
                    state = STATE_TO_MED1
                    rospy.loginfo("약품1 배달 지점으로 이동 시작")

        elif state == STATE_IDLE:
            # 대기 상태: 호출이 오면 0.5초마다 좌표가 온다
            if got_udp:
                set_goal(*pixel_to_meter(u, v))
                state = STATE_TO_USER
                user_signal_sent = False
                rospy.loginfo("사용자에게로 이동 시작")

        elif state == STATE_TO_USER:
            if got_udp:
                set_goal(*pixel_to_meter(u, v))
                rospy.loginfo("사용자에게로 이동중")
            if distance_to_goal() < GOAL_THRESH:
                state = STATE_AT_USER
                rospy.loginfo("사용자에게 도착")
                # 필요 시 사용자 도착 신호 1회 송신
                if not user_signal_sent:
                    try:
                        tx_sock.sendto(b"999,999", (RPI_IP, RPI_PORT_SIGNAL))
                        rospy.loginfo("Sent UDP '999,999' to Raspberry Pi")
                        user_signal_sent = True
                    except Exception as e:
                        rospy.logwarn(f"UDP send failed: {e}")

        elif state == STATE_AT_USER:
            # 진료실 좌표가 들어올 때까지 대기
            if got_udp:
                set_goal(*pixel_to_meter(u, v))
                state = STATE_TO_CLINIC
                rospy.loginfo("진료실로 이동 시작")
            else:
                rospy.loginfo("진료실 좌표 대기중")

        elif state == STATE_TO_CLINIC:
            # 요구사항: 항상 "진료실로 이동중"만 출력
            if got_udp:
                set_goal(*pixel_to_meter(u, v))   # 계속 추종
            else:
                # RPi가 좌표 전송을 멈춤(= 진료실 도착 판단) → 약품1로
                set_goal(*MED1_GOAL)
                state = STATE_TO_MED1
            rospy.loginfo("진료실로 이동중")

        # 3) 네비 목표 발행
        goal_msg = PoseStamped()
        goal_msg.header.stamp = rospy.Time.now()
        goal_msg.header.frame_id = "map"
        goal_msg.pose.position.x = goal_x
        goal_msg.pose.position.y = goal_y
        goal_msg.pose.orientation.w = 1.0
        goal_pub.publish(goal_msg)

        # 4) RViz 시각화
        point_pub.publish(Point(goal_x, goal_y, 0.0))
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "clicked_goal"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = goal_x
        marker.pose.position.y = goal_y
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.15
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.0, 0.0, 1.0, 1.0
        marker_pub.publish(marker)

        rospy.loginfo(f"STATE({state}) GOAL({goal_x:.2f},{goal_y:.2f})")
        rate.sleep()


if __name__ == "__main__":
    try:
        udp_listener()
    except rospy.ROSInterruptException:
        pass


```

- UDP 통신으로 라즈베리파이와 로봇 간 좌표·신호를 주고받으며 제어한다.
- BLE·IMU 데이터를 EKF로 융합해 실내 위치를 추정하고 지도에 표시한다.
- 사용자의 목적지 선택에 따라 경로 탐색·벽 회피·로봇 호출 등 상태 기반 네비게이션을 수행한다.
```
import joblib
import sys
import serial
import time
import numpy as np
import socket
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QShortcut, QLabel, QGraphicsDropShadowEffect
from PyQt5.QtCore import QTimer, QMutex, Qt, QFile, QTextStream, QPointF, QThread, pyqtSignal
from PyQt5.QtGui import QKeySequence, QColor

# --- 모듈 임포트 ---
from app_config import load_config
from fingerprinting import FingerprintDB
from trilateration import EKF
from ble_scanner import BLEScanThread
from map_viewer import MapViewer
from serial_reader import SerialReader
from event import SelectionDialog
from bin import create_binary_map
from Astar import find_path, create_distance_map
from robot_tracker import RobotTrackerThread
from lgbm_predictor import LGBM_Classifier_Predictor

# --- UDP 수신 스레드 클래스 ---
class UDPReceiverThread(QThread):
    message_received = pyqtSignal(str)
    def __init__(self, port, parent=None):
        super().__init__(parent)
        self.port, self.is_running = port, True
    def run(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(('0.0.0.0', self.port)); sock.settimeout(1.0)
        print(f"UDP 수신 대기 시작 (포트: {self.port})")
        while self.is_running:
            try:
                data, _ = sock.recvfrom(1024)
                message = data.decode().strip()
                if message: self.message_received.emit(message)
            except socket.timeout: continue
        sock.close()
    def stop(self): self.is_running = False


# --- 메인 애플리케이션 클래스 ---
class IndoorPositioningApp(QWidget):
    def __init__(self, config):
        super().__init__()
        self.config, self.room_coords = config, {room['name']: (room['x'], room['y']) for room in config['rooms']}
        self.rssi_mutex, self.rssi_data = QMutex(), {}
        # fused_pos를 튜플이 아닌 NumPy 배열로 초기화
        self.current_speed, self.current_yaw, self.fused_pos = 0.0, 180.0, np.array([0.0, 0.0])
        self.target_room, self.last_start_grid, self.BLOCK_SIZE = None, None, 10

        # 벽 회피 기능 파라미터
        self.AVOIDANCE_THRESHOLD_GRID = 50
        self.REPULSION_STRENGTH = 0.03

        self.robot_arrival_processed = False

        self.udp_target_ip = self.config.get('udp_target_ip', "10.24.184.20")
        self.udp_target_port = self.config.get('udp_target_port', 5005)
        self.udp_socket, self.udp_send_timer = socket.socket(socket.AF_INET, socket.SOCK_DGRAM), QTimer(self)
        self.udp_destination_timer = QTimer(self)
        self.udp_receiver = UDPReceiverThread(port=self.config.get('udp_listen_port', 5006))

        # 벽 회피 보정을 위한 타이머
        self.wall_avoidance_timer = QTimer(self)


        self._init_logic_components(); self._init_ui(); self._connect_signals(); self._start_timers()

    def _init_logic_components(self):
        binary_grid_as_list = create_binary_map(self.config['map_file'], block_size=self.BLOCK_SIZE)
        if binary_grid_as_list is not None:
            self.binary_grid = np.array(binary_grid_as_list)
            dist_map_as_list, self.max_dist = create_distance_map(self.binary_grid)
            self.distance_map = np.array(dist_map_as_list)
        else:
            self.close()
        self.ekf = EKF(self.config.get('ekf_dt', 1.0))
        try:
            self.lgbm_predictor = joblib.load('lgbm_predictor.pkl')
            print("저장된 LGBM Predictor 객체를 성공적으로 불러왔습니다.")
        except FileNotFoundError:
            print("오류: 저장된 Predictor 파일(lgbm_predictor.pkl)을 찾을 수 없습니다.")
            self.lgbm_predictor = None
        self.ble_scanner_thread = BLEScanThread(self.config)
        try:
            imu_port, baudrate = self.config.get('imu_port', '/dev/ttyUSB0'), self.config.get('imu_baudrate', 115200)
            self.serial_port = serial.Serial(imu_port, baudrate); time.sleep(1); self.serial_port.write(b'ZERO\n')
            self.serial_reader = SerialReader(port=imu_port, baudrate=baudrate); self.serial_reader.start()
        except serial.SerialException as e:
            print(f"시리얼 포트 오류: {e}."); self.serial_reader = None

        self.robot_tracker = RobotTrackerThread(port=self.config.get('robot_udp_port', 5005))

    def _init_ui(self):
        self.toast_label = QLabel(self); self.toast_label.setObjectName("Toast"); self.toast_label.setAlignment(Qt.AlignCenter); self.toast_label.hide()

        # 로봇 호출 상태 위젯
        self.setObjectName("MainWindow")
        self.robot_status_widget = QWidget(self)
        self.robot_status_widget.setObjectName("RobotStatus")
        self.robot_status_widget.hide()
        status_layout = QHBoxLayout(self.robot_status_widget)
        status_layout.setContentsMargins(25, 10, 25, 10); status_layout.setSpacing(20)
        status_layout.addWidget(QLabel("로봇이 오고 있습니다..."))
        self.stop_call_btn = QPushButton("중지"); self.stop_call_btn.setObjectName("StopCallButton")
        status_layout.addWidget(self.stop_call_btn)
        shadow = QGraphicsDropShadowEffect(); shadow.setBlurRadius(25); shadow.setColor(QColor(0, 0, 0, 80)); shadow.setOffset(0, 4)
        self.robot_status_widget.setGraphicsEffect(shadow)

        # 로봇 도착 확인 위젯
        self.arrival_prompt_widget = QWidget(self)
        self.arrival_prompt_widget.setObjectName("ArrivalPrompt")
        self.arrival_prompt_widget.hide()
        arrival_layout = QVBoxLayout(self.arrival_prompt_widget)
        arrival_layout.setContentsMargins(20, 20, 20, 20); arrival_layout.setSpacing(15); arrival_layout.setAlignment(Qt.AlignCenter)
        message_layout = QHBoxLayout(); message_layout.setSpacing(15); message_layout.setAlignment(Qt.AlignCenter)
        message_label = QLabel("로봇이 도착했습니다.<br>진료실로 이동하시겠습니까?")
        message_label.setAlignment(Qt.AlignCenter)
        message_layout.addWidget(message_label)
        self.confirm_move_btn = QPushButton("확인"); self.confirm_move_btn.setObjectName("ConfirmMoveButton")
        arrival_layout.addLayout(message_layout)
        arrival_layout.addWidget(self.confirm_move_btn, alignment=Qt.AlignCenter)
        arrival_shadow = QGraphicsDropShadowEffect(); arrival_shadow.setBlurRadius(25); arrival_shadow.setColor(QColor(0, 0, 0, 80)); arrival_shadow.setOffset(0, 4)
        self.arrival_prompt_widget.setGraphicsEffect(arrival_shadow)

        # 길안내 상태 위젯
        self.navigation_status_widget = QWidget(self)
        self.navigation_status_widget.setObjectName("NavigationStatus")
        self.navigation_status_widget.hide()
        nav_layout = QHBoxLayout(self.navigation_status_widget)
        nav_layout.setContentsMargins(25, 10, 25, 10); nav_layout.setSpacing(20)
        nav_layout.addWidget(QLabel("로봇을 따라 이동하세요.."))
        self.cancel_nav_btn = QPushButton("취소"); self.cancel_nav_btn.setObjectName("CancelNavButton")
        nav_layout.addWidget(self.cancel_nav_btn)
        nav_shadow = QGraphicsDropShadowEffect(); nav_shadow.setBlurRadius(25); nav_shadow.setColor(QColor(0, 0, 0, 80)); nav_shadow.setOffset(0, 4)
        self.navigation_status_widget.setGraphicsEffect(nav_shadow)

        # 현재 경로 안내 위젯 (좌측 상단)
        self.current_nav_widget = QWidget(self)
        self.current_nav_widget.setObjectName("CurrentNav")
        self.current_nav_widget.hide()
        nav_info_layout = QHBoxLayout(self.current_nav_widget)
        nav_info_layout.setContentsMargins(20, 10, 20, 10); nav_info_layout.setSpacing(15)
        self.current_nav_label = QLabel("안내: ")
        self.current_nav_cancel_btn = QPushButton("취소")
        self.current_nav_cancel_btn.setObjectName("CurrentNavCancelButton")
        nav_info_layout.addWidget(self.current_nav_label)
        nav_info_layout.addWidget(self.current_nav_cancel_btn)
        current_nav_shadow = QGraphicsDropShadowEffect(); current_nav_shadow.setBlurRadius(20); current_nav_shadow.setColor(QColor(0, 0, 0, 70)); current_nav_shadow.setOffset(0, 3)
        self.current_nav_widget.setGraphicsEffect(current_nav_shadow)

        # 메인 레이아웃
        self.map_viewer = MapViewer(self.config['map_file'], self.config['px_per_m_x'], self.config['px_per_m_y'])
        self.map_viewer._init_est_items(0, 0, 180.0)
        self.nav_btn = QPushButton("길안내"); self.nav_btn.setObjectName("NAV")
        self.robot_btn = QPushButton("로봇\n호출"); self.robot_btn.setObjectName("Robot")
        right_layout = QVBoxLayout(); right_layout.addWidget(self.nav_btn); right_layout.addWidget(self.robot_btn)
        main_layout = QHBoxLayout(self); main_layout.addWidget(self.map_viewer); main_layout.addLayout(right_layout)

        main_layout.setSpacing(13)

        self.setWindowTitle("ODIGA"); self.setFocusPolicy(Qt.StrongFocus); self.load_stylesheet('stylesheet.qss'); self.showFullScreen(); self.setFocus()

    def _connect_signals(self):
        self.ble_scanner_thread.detected.connect(self._on_ble_device_detected)

        if self.serial_reader:
            self.serial_reader.heading_received.connect(self._on_yaw_update)
            self.serial_reader.speed_received.connect(self._on_speed_update)
        self.nav_btn.clicked.connect(self._show_selection_dialog)
        self.robot_btn.clicked.connect(self._on_robot_call_clicked)
        self.stop_call_btn.clicked.connect(self._on_robot_call_stop_clicked)
        self.confirm_move_btn.clicked.connect(self._on_arrival_confirmed)
        self.cancel_nav_btn.clicked.connect(self._on_navigation_cancel_clicked)
        self.current_nav_cancel_btn.clicked.connect(lambda: self._stop_navigation("안내를 취소했습니다."))
        shortcut = QShortcut(QKeySequence("G"), self); shortcut.activated.connect(self._start_ble_scan)
        self.udp_send_timer.timeout.connect(self._send_position_udp)
        self.udp_destination_timer.timeout.connect(self._send_destination_udp)
        self.udp_receiver.message_received.connect(self._on_robot_message_received)
        self.robot_tracker.robot_position_updated.connect(self._on_robot_position_update)

        # 벽 회피 타이머의 timeout 신호를 _apply_wall_avoidance 메서드에 연결
        self.wall_avoidance_timer.timeout.connect(self._apply_wall_avoidance)


    def _start_timers(self):
        self.rssi_clear_timer = QTimer(self); self.rssi_clear_timer.timeout.connect(self._clear_rssi_cache); self.rssi_clear_timer.start(2000)
        self.udp_receiver.start()
        self.robot_tracker.start()


    def _on_robot_position_update(self, px, py):
        self.map_viewer.update_robot_position(px, py)

    def _send_position_udp(self):
        px, py = self.fused_pos[0] * self.config['px_per_m_x'], self.fused_pos[1] * self.config['px_per_m_y']
        message = f"{int(px)},{int(py)}"
        self.udp_socket.sendto(message.encode(), (self.udp_target_ip, self.udp_target_port))
        print(f"UDP Sent: {message}")

    def _send_destination_udp(self):
        """설정된 목적지(target_room)의 좌표를 UDP로 전송합니다."""
        if self.target_room:
            dest_m = self.target_room
            dest_px = dest_m[0] * self.config['px_per_m_x']
            dest_py = dest_m[1] * self.config['px_per_m_y']
            message = f"{int(dest_px)},{int(dest_py)}"
            self.udp_socket.sendto(message.encode(), (self.udp_target_ip, self.udp_target_port))
            print(f"UDP Destination Sent: {message}")
        else:
            self.udp_destination_timer.stop()
            print("오류: 목적지가 설정되지 않아 목적지 전송을 중단합니다.")

    def _show_toast(self, message, duration=3000):
        self.toast_label.setText(message); self.toast_label.adjustSize()
        self._update_popup_position(self.toast_label)
        self.toast_label.show(); self.toast_label.raise_()
        QTimer.singleShot(duration, self.toast_label.hide)

    def _update_popup_position(self, popup_widget):
        x = (self.width() - popup_widget.width()) / 2; y = 50
        popup_widget.move(int(x), int(y))

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._update_popup_position(self.toast_label)
        self._update_popup_position(self.robot_status_widget)
        self._update_popup_position(self.arrival_prompt_widget)
        self._update_popup_position(self.navigation_status_widget)
        self.current_nav_widget.move(20, 20)

    def _update_navigation_path(self):
        if not self.target_room: return

        user_px = self.fused_pos[0] * self.config['px_per_m_x']
        user_py = self.fused_pos[1] * self.config['px_per_m_y']
        target_px = self.target_room[0] * self.config['px_per_m_x']
        target_py = self.target_room[1] * self.config['px_per_m_y']

        distance = np.sqrt((user_px - target_px)**2 + (user_py - target_py)**2)

        if distance < 25:
            self._stop_navigation("<b>목적지에 도착했습니다.</b> 안내를 종료합니다.")
            return

        start_m, end_m = self.fused_pos, self.target_room
        start_grid, end_grid = self.meters_to_grid(start_m), self.meters_to_grid(end_m)

        # 시작점이 맵 범위 안에 있는지 확인하는 안전 코드
        grid_height, grid_width = self.binary_grid.shape
        if not (0 <= start_grid[0] < grid_height and 0 <= start_grid[1] < grid_width):
            print(f"경고: 시작점 {start_grid}이(가) 맵 범위를 벗어났습니다. 경로를 업데이트하지 않습니다.")
            return # 함수를 즉시 종료하여 오류 방지

        if self.last_start_grid == start_grid: return
        self.last_start_grid = start_grid
        path_grid = find_path(self.binary_grid, start_grid, end_grid, self.distance_map, self.max_dist, self.config.get('penalty_strength', 2.5))
        path_pixels = [self.grid_to_pixels(pos) for pos in path_grid] if path_grid else None
        self.map_viewer.draw_path(path_pixels)

    def _get_direction_from_yaw(self, yaw):
        """Yaw 각도를 N, E, S, W 방향으로 변환합니다."""
        if 45 <= yaw < 135:
            return 'S'
        elif 135 <= yaw < 225:
            return 'W'
        elif 225 <= yaw < 315:
            return 'N'
        else: # 315 <= yaw or yaw < 45
            return 'E'

    def _on_ble_device_detected(self, rssi_vec):
        self.rssi_mutex.lock()
        self.rssi_data.update(rssi_vec)
        local_rssi_copy = self.rssi_data.copy()
        self.rssi_mutex.unlock()

        if len(local_rssi_copy) >= 1:
            if self.lgbm_predictor:
                try:
                    # 1. IMU 센서에서 받은 Yaw 값으로 현재 방향('N' 등)을 결정합니다.
                    direction = self._get_direction_from_yaw(self.current_yaw)
                    local_rssi_copy['direction'] = direction

                    # 2. Predictor 객체의 predict 메소드를 호출합니다. (내부에서 모든 전처리 수행)
                    predicted_label = self.lgbm_predictor.predict(local_rssi_copy)

                    # 3. 예측된 레이블('x_y')을 좌표로 변환합니다.
                    x_str, y_str = predicted_label.split('_')
                    pts_grid = (int(x_str)  , int(y_str) )
                    pts_pixels_qpoint = self.grid_to_pixels(pts_grid)

                    px_per_m_x = self.config.get('px_per_m_x', 1.0)
                    px_per_m_y = self.config.get('px_per_m_y', 1.0)
                    pts_meters = np.array([
                        pts_pixels_qpoint.x() * 19 / px_per_m_x,
                        pts_pixels_qpoint.y() * 19 / px_per_m_y
                    ])
                    self.ekf.update(pts_meters)

                    self.fused_pos = self.ekf.get_state()[:2].flatten()

                    self.map_viewer.mark_estimated_position(*self.fused_pos, self.current_yaw)
                    self._update_navigation_path()

                except Exception as e:
                    print(f"LGBM 예측 중 오류 발생: {e}")


    def _on_speed_update(self, speed):
        self.current_speed = speed
        self.ekf.predict(self.current_yaw, self.current_speed)
        self.fused_pos = self.ekf.get_state()[:2].flatten()
        self._update_navigation_path()

    def _on_yaw_update(self, yaw):
        self.current_yaw = yaw
        self.map_viewer.move_to(*self.fused_pos, self.current_yaw)

    def _clear_rssi_cache(self):
        self.rssi_mutex.lock(); self.rssi_data.clear(); self.rssi_mutex.unlock()

    def _show_selection_dialog(self):
        dialog = SelectionDialog(self)
        if dialog.exec():
            selected = dialog.selected_room
            self.target_room = self.room_coords[selected]
            self._show_toast(f"<b>{selected}</b>로 안내를 시작합니다.")
            self.last_start_grid = None
            self._update_navigation_path()
            self.current_nav_label.setText(f"<b>{selected}</b> 안내중")
            self.current_nav_widget.adjustSize()
            self.current_nav_widget.show()
            self.current_nav_widget.raise_()
        else:
            self._show_toast("안내를 취소했습니다.", duration=2000)
            self.target_room = None
            self.map_viewer.draw_path(None)
            self.current_nav_widget.hide()

    def _start_ble_scan(self):
        if not self.ble_scanner_thread.isRunning():
            self.ble_scanner_thread.start()
            print("BLE Scan Started.")
            # BLE 스캔 시작과 함께 1초 간격으로 벽 회피 타이머를 시작
            if not self.wall_avoidance_timer.isActive():
                self.wall_avoidance_timer.start(1000) # 1000ms = 1초
                print("벽 회피 보정 타이머를 시작합니다 (1초 간격).")

    def _on_robot_call_clicked(self):
        if not self.target_room:
            self._show_toast("로봇을 부를 목적지를 선택해주세요.", duration=2500)

            dialog = SelectionDialog(self)
            if dialog.exec():
                selected = dialog.selected_room
                self.target_room = self.room_coords[selected]
                self.last_start_grid = None
                self._update_navigation_path()
                self.current_nav_label.setText(f"<b>{selected}</b> 안내중")
                self.current_nav_widget.adjustSize()
                self.current_nav_widget.show()
                self.current_nav_widget.raise_()
                self._show_toast(f"<b>{selected}</b>(으)로 목적지 설정 후 로봇을 호출합니다.")
            else:
                self._show_toast("로봇 호출을 취소했습니다.", duration=2000)
                return

        if self.udp_send_timer.isActive():
            self._show_toast("이미 로봇이 호출되었습니다.")
            return

        self.robot_arrival_processed = False

        self.udp_send_timer.start(1000)
        self.robot_status_widget.adjustSize()
        self._update_popup_position(self.robot_status_widget)
        self.robot_status_widget.show()
        self.robot_status_widget.raise_()

    def _on_robot_call_stop_clicked(self):
        if self.udp_send_timer.isActive():
            self.udp_send_timer.stop()
        if self.udp_destination_timer.isActive():
            self.udp_destination_timer.stop()
        self.robot_status_widget.hide()
        self.arrival_prompt_widget.hide()
        self.navigation_status_widget.hide()
        self._show_toast("로봇 호출을 중지했습니다.")

    def _on_arrival_confirmed(self):
        self.arrival_prompt_widget.hide()
        self.udp_destination_timer.start(500)
        self.navigation_status_widget.adjustSize()
        self._update_popup_position(self.navigation_status_widget)
        self.navigation_status_widget.show()
        self.navigation_status_widget.raise_()

    def _stop_navigation(self, message):
        """길안내를 중지하고 관련 UI를 정리합니다."""
        if self.udp_destination_timer.isActive():
            self.udp_destination_timer.stop()
        self.navigation_status_widget.hide()
        self.current_nav_widget.hide()
        self.target_room = None
        self.map_viewer.draw_path(None)
        self._show_toast(message)

    def _on_navigation_cancel_clicked(self):
        self._stop_navigation("길안내를 취소했습니다.")

    def _on_robot_message_received(self, message):
        print(f"로봇으로부터 메시지 수신: '{message}'")
        if message == "1":
            self.robot_status_widget.hide()
            self.arrival_prompt_widget.hide()
            self.navigation_status_widget.hide()
            if self.udp_send_timer.isActive():
                self.udp_send_timer.stop()
            self._show_toast("로봇이 도착했습니다.")

        elif message == "999,999":
            if not self.robot_arrival_processed:
                self.robot_arrival_processed = True
                print("로봇 도착 신호 (999,999) 수신. [최초 1회 처리]")

                if self.udp_send_timer.isActive():
                    self.udp_send_timer.stop()

                self.robot_status_widget.hide()

                self.arrival_prompt_widget.adjustSize()
                self._update_popup_position(self.arrival_prompt_widget)
                self.arrival_prompt_widget.show()
                self.arrival_prompt_widget.raise_()

    def load_stylesheet(self, filename):
        qss_file = QFile(filename);
        if qss_file.open(QFile.ReadOnly | QFile.Text): self.setStyleSheet(QTextStream(qss_file).readAll())
        else: print(f"'{filename}' 스타일시트 로드 실패!")

    def meters_to_grid(self, pos_m):
        row, col = int(pos_m[1] * self.config['px_per_m_y'] / self.BLOCK_SIZE), int(pos_m[0] * self.config['px_per_m_x'] / self.BLOCK_SIZE)
        return (row, col)

    def grid_to_pixels(self, pos_grid):
        px, py = pos_grid[1] * self.BLOCK_SIZE + self.BLOCK_SIZE / 2, pos_grid[0] * self.BLOCK_SIZE + self.BLOCK_SIZE / 2
        return QPointF(px, py)

    def _apply_wall_avoidance(self):
        """현재 위치가 벽에 너무 가까우면 보정합니다. (타이머에 의해 주기적으로 호출됨)"""
        if self.fused_pos is None or self.distance_map is None:
            return

        current_grid = self.meters_to_grid(self.fused_pos)
        row, col = current_grid

        height, width = self.distance_map.shape
        if not (0 <= row < height and 0 <= col < width):
            return

        distance_to_wall = self.distance_map[row][col]
        if distance_to_wall >= self.AVOIDANCE_THRESHOLD_GRID:
            return

        # 거리 맵 그라디언트 근사
        grad_r = self.distance_map[min(row + 1, height - 1)][col] - self.distance_map[max(row - 1, 0)][col]
        grad_c = self.distance_map[row][min(col + 1, width - 1)] - self.distance_map[row][max(col - 1, 0)]

        repulsion_vector_grid = np.array([grad_c, grad_r])
        norm = np.linalg.norm(repulsion_vector_grid)
        if norm < 1e-6:
            return

        direction_vector = repulsion_vector_grid / norm
        penetration_depth = self.AVOIDANCE_THRESHOLD_GRID - distance_to_wall
        correction_magnitude_grid = penetration_depth * self.REPULSION_STRENGTH
        correction_vector_grid = direction_vector * correction_magnitude_grid

        correction_m_x = correction_vector_grid[0] * self.BLOCK_SIZE / self.config['px_per_m_x']
        correction_m_y = correction_vector_grid[1] * self.BLOCK_SIZE / self.config['px_per_m_y']
        correction_vector_m = np.array([correction_m_x, correction_m_y])

        # fused_pos와 EKF 상태 동시에 보정
        self.fused_pos += correction_vector_m
        try:
            
            self.ekf.x[0] = self.fused_pos[0]
            self.ekf.x[1] = self.fused_pos[1]
            
            if hasattr(self.ekf, "P"):
                self.ekf.P[:2, :2] *= 1.2
                
        except Exception as e:
            print(f"EKF 상태 보정 중 오류 발생: {e}")

        # 보정된 위치를 지도에 즉시 반영
        self.map_viewer.mark_estimated_position(*self.fused_pos, self.current_yaw)
        print(f"벽 회피 적용: ({correction_m_x:.2f}, {correction_m_y:.2f})m 보정됨")

        self._update_navigation_path()

    def closeEvent(self, event):
        self.robot_tracker.stop()
        self.udp_receiver.stop()
        self.ble_scanner_thread.stop()
        if self.serial_reader: self.serial_reader.stop()

        # 프로그램 종료 시 벽 회피 타이머 정지
        if self.wall_avoidance_timer.isActive():
            self.wall_avoidance_timer.stop()

        super().closeEvent(event)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    config = load_config()
    main_window = IndoorPositioningApp(config)
    sys.exit(app.exec_())
```
