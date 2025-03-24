#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Point
import signal  # Ctrl+C로 프로그램 종료 처리

def keyboard_input_publisher():
    rospy.init_node('keyboard_input_publisher', anonymous=True)
    
    # 퍼블리셔 생성
    pub = rospy.Publisher('/input_coordinates', Point, queue_size=10)
    
    rate = rospy.Rate(1)  # 1Hz로 주기적으로 발행
    
    # 기본 좌표 초기값 (아직 입력 안 되었으면 0,0,0)
    current_point = Point()
    current_point.x = 0.0
    current_point.y = 0.0
    current_point.z = 0.0
    
    # 입력 받는 스레드를 따로 두거나 while 내 입력 대기 대신 non-blocking 방식이 좋지만,
    # 단순 구조에서는 아래처럼 구현 가능:
    import threading

    def input_thread():
        nonlocal current_point #내부함수에서 외부함수의 변수를 바꿔야하므로로
        while not rospy.is_shutdown():
            try:
                x = float(input("x 좌표 입력: "))
                y = float(input("y 좌표 입력: "))
                z = float(input("z 좌표 입력: "))
                current_point.x = x
                current_point.y = y
                current_point.z = z
                rospy.loginfo(f"입력값 갱신: x={x}, y={y}, z={z}")
            except ValueError:
                rospy.logwarn("잘못된 입력입니다. 숫자를 입력하세요.")
    
    # 입력 받는 쓰레드 시작
    t = threading.Thread(target=input_thread) 
    t.daemon = True
    t.start()
    
    # 메인 루프: 주기적으로 현재 좌표값 발행
    while not rospy.is_shutdown():
        pub.publish(current_point)
        rospy.loginfo(f"현재 퍼블리시 중: x={current_point.x}, y={current_point.y}, z={current_point.z}")
        rate.sleep()

if __name__ == '__main__':
    keyboard_input_publisher()
