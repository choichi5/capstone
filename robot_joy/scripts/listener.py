#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Software License Agreement (BSD License)


import rospy
import threading
import packets    # SCARA 패킷 명령어 저장되어 있는 사용자 헤더파일
from std_msgs.msg import String, Float32MultiArray
from sensor_msgs.msg import Joy
import sys
import signal
import numpy as np
#from geometry_msgs.msg import Point

'''
def callback(data):
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)
'''

#### global variables ####
servo_on = 0
servo_off = 0
whereareyou = 1
joystic_xyzw = [0.0, 0.0, 0.0, 0.0]
speed_btn = [0, 0]
speed_num = 0
start_pos_btn = 0

order_angle = [0,0,0,0]
# order_angle = [0, 10, 61.777, 51.]
NEXT_POS = []


# 변환행렬 상수
T_mtx = np.array([[-1,  0,  0, 920],
                  [0,  -1,  0,  90],
                  [0,   0,  1, 540],
                  [0,   0,  0,   1]])

'''
# === 전역 변수 ===
current_position = [0.0, 0.0, 0.0]
goal_position = [0.0, 0.0, 0.0]
threshold = 5.0  # 임계값 (mm)

# === 현재 좌표 구독 콜백 ===
def current_pos_callback(data):
    global current_position
    current_position = data.data  # Float32MultiArray 타입으로 들어옴
    rospy.loginfo(f"[현재 좌표] {current_position}")

# === 키보드 입력 좌표 콜백 ===
def keyboard_input_callback(data):
    global goal_position
    goal_position = [data.x, data.y, data.z]
    rospy.loginfo(f"[목표 좌표 입력] {goal_position}")
    '''


## node 비정상 작동 시 강제 종료를 위한 함수 ##
def signal_handler(signal, frame): # ctrl + c -> exit program
        print('You pressed Ctrl+C!')
        sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)
    

## 조이스틱 값을 받는 함수 ##
def joycallback(data):
    global servo_on, servo_off, whereareyou, joystic_xyzw, start_pos_btn
    servo_off = data.buttons[8]
    servo_on = data.buttons[9]
    # speed_slow_btn = data.buttons[1] # B버튼 (속도 현저히 낮추는 버튼)
    start_pos_btn = data.buttons[2]  # A버튼 (실험 시작 위치로 이동)
    # whereareyou = data.buttons[0]  # Y버튼 (현재 좌표 확인)
    joystic_xyzw[0] = data.axes[0]  # x
    joystic_xyzw[1] = data.axes[1]  # y
    joystic_xyzw[2] = data.axes[4]  # z
    joystic_xyzw[3] = data.axes[3]  # w
    speed_btn[0] = data.buttons[4]  # speed +
    speed_btn[1] = data.buttons[1]  # speed -


def controlcallback(data):
    global order_angle, NEXT_POS, T_mtx

    # 토크 값을 SCARA 좌표에 맞게 변환
    rotation_mtx = T_mtx[:3, :3]   # 회전 행렬 부분만 추출
    workspace_torque = np.array([data.data[0], data.data[1], 0, 1])
    scara_torque = np.dot(T_mtx, workspace_torque)

    # 각도 값 할당
    order_angle[:2] = scara_torque[:2]
    order_angle[2] = 54.897
    # order_angle[3] = data.data[2]
    order_angle[3] = 71.663

    # STX, DUMMY, BC, ch.1, Motion Type, Coordinate unit
    packet = [
        0x02,           # STX
        0xFF,           # DUMMY
        0x42, 0x43,     # BC
        0x30,           # ch.1
        0x31,           # Motion Type: Joint-based(0)  / Linear-based(1)
        0x31            # Coordinate unit: Degree (Deg) / mm (1)
    ]

    # 각도 값을 문자열로 변환하여 패킷에 추가 (10비트로 구성, 남는 자리는 0x20으로 채움)
    for angle in order_angle:
        angle_packet = []
        if angle < 0:
            angle_packet.append(0x2D)  # 음수일 경우 '-' 부호 추가 (0x2D)
        angle_str = "{:.3f}".format(abs(angle))  # 소수점 포함하여 3자리까지만
        angle_packet.extend([ord(char) for char in angle_str])  # 문자열을 각 문자로 변환하여 추가
        # 남는 자릿수를 0x20으로 채워 총 10비트로 맞춤
        while len(angle_packet) < 10:
            angle_packet.insert(0, 0x20)  # 왼쪽에 0x20 추가하여 10비트 맞춤
        packet.extend(angle_packet)

    # EXT 추가
    packet.append(0x03)  # EXT

    # LRC 계산 (DUMMY부터 EXT 이전까지 XOR)
    lrc = packet[1]
    for byte in packet[2:-1]:  # EXT 이전까지 XOR
        lrc ^= byte

    # LRC 추가
    packet.append(lrc)
    packet.append(0x06)

    # 패킷을 문자 리스트로 변환하여 전역 변수에 저장
    NEXT_POS = [chr(val) for val in packet]

    # 콜백 함수 내에서 패킷 출력
    # print("Generated Packet: ", NEXT_POS)
    # print("Generated Packet: ", packets.START_POS)
    # print(len(NEXT_POS), len(packets.START_POS))
    




## 속도 인자 값을 조절하는 함수 ##
def speed_func(dn):
    global speed_num

    speed_num += dn
    if speed_num >= 50000:
        speed_num -= dn
    elif speed_num <= 0:
        speed_num -= dn

    arr = []
    for i in str(speed_num):
        arr.append(i)

    for i in range(5):
         packets.SPEED[i+4] = '0'   # speed value reset
    
    for i in range(len(arr)):       # new speed value input
         packets.SPEED[8-i] = arr[len(arr)-1-i]         


## 패킷의 LRC 값을 계산해주는 함수 ##
def lrc_calc(arr1):
    # DUMMY 부터 EXT 이전까지 XOR 연산
    LRC = 0x00  # 초기 LRC 값
    for byte in arr1[1:-3]:  # DUMMY 부터 EXT 이전까지
        LRC ^= byte

    # 계산된 LRC 값을 패킷에 넣기
    arr1[-2] = LRC


# ## 기기로부터 입력받은 현재위치 패킷 값 ##
# def xyz_callback(data):
#     print(data)


'''
# === 자동 움직임 발행 함수 ===
def auto_move_by_diff(current_pos, goal_pos, pub3):
    diff = np.array(goal_pos) - np.array(current_pos)

    # X축
    if diff[0] > threshold:
        pub3.publish(packets.MUVX[0])  # +X
    elif diff[0] < -threshold:
        pub3.publish(packets.MUVX[1])  # -X

    # Y축
    if diff[1] > threshold:
        pub3.publish(packets.MUVY[0])  # +Y
    elif diff[1] < -threshold:
        pub3.publish(packets.MUVY[1])  # -Y

    # Z축
    if diff[2] > threshold:
        pub3.publish(packets.MUVZ[0])  # +Z
    elif diff[2] < -threshold:
        pub3.publish(packets.MUVZ[1])  # -Z
'''


##############
### 메인함수 ###
##############

def listener():

    global NEXT_POS

    rospy.init_node('listener', anonymous=True)

    #### Subscribe section ####
    # rospy.Subscriber('chatter', String, callback)
    rospy.Subscriber('/joy', Joy, joycallback)                                     # 조이스틱 값 구독
    rospy.Subscriber('/scara_joint_commands', Float32MultiArray, controlcallback)  # 제어 각도 값 구독
    # rospy.Subscriber('/read', String, xyz_callback)  
                                # SCARA 패킷 값 구독
    # rospy.Subscriber('/input_coordinates', Point, keyboard_input_callback) #최치혁혁
    # rospy.Subscriber('/scara_coordi', Float32MultiArray, current_pos_callback)

    #### Publish section ####
    pub = rospy.Publisher('packet', String, queue_size=10)     # end effect의 좌표 요청 전용 publisher handle
    pub2 = rospy.Publisher('packet2', String, queue_size=10)   # 각 관절의 현재 각도 요청 전용 publisher handle
    pub3 = rospy.Publisher('packet3', String, queue_size=10)   # 움직임에 대한 지령 전용 publisher handle
    rate = rospy.Rate(10) # 10hz

''' 최치혁
        rate = rospy.Rate(2)  # 2 Hz 루프 주기

    while not rospy.is_shutdown():
        # 현재 좌표와 목표 좌표 비교 후 자동 지령 발행
        auto_move_by_diff(current_position, goal_position, pub3)
        rate.sleep()

    rospy.spin()
    '''



    #### 알고리즘 파트 ####
    while not rospy.is_shutdown():

        if servo_on == 1:
            pub.publish(packets.SVON); #rospy.loginfo(packets.SVON)
        if servo_off == 1:
            pub.publish(packets.SVOFF); #rospy.loginfo(packets.SVOFF)
        
        # 현재 End Effect 위치 값 받기 (Cartecian 좌표계 기준)
        if whereareyou == 1:
            pub.publish(''.join(packets.COORDI))
            # pub2.publish(''.join(packets.JOINT))
            pub3.publish(''.join(NEXT_POS))

        # 실험 시작 위치로 이동
        if start_pos_btn == 1: 
            pub3.publish(''.join(packets.START_POS))
            
        
        # X moving
        if (joystic_xyzw[0] <= -0.8 and joystic_xyzw[0] >= -1.0):
            pub3.publish(packets.MUVX[1]); #rospy.loginfo(packets.MUVX[0])
        if (joystic_xyzw[0] >= 0.8 and joystic_xyzw[0] <= 1.0):
            pub3.publish(packets.MUVX[0]); #rospy.loginfo(packets.MUVX[1])

        # Y moving
        if (joystic_xyzw[1] <= -0.8 and joystic_xyzw[1] >= -1.0):
            pub3.publish(packets.MUVY[0]); #rospy.loginfo(packets.MUVY[1])
        if (joystic_xyzw[1] >= 0.8 and joystic_xyzw[1] <= 1.0):
            pub3.publish(packets.MUVY[1]); #rospy.loginfo(packets.MUVY[0])

        # Z moving
        if (joystic_xyzw[2] <= -0.8 and joystic_xyzw[2] >= -1.0):
            pub3.publish(packets.MUVZ[0]); #rospy.loginfo(packets.MUVZ[0])
        if (joystic_xyzw[2] >= 0.8 and joystic_xyzw[2] <= 1.0):
            pub3.publish(packets.MUVZ[1]); #rospy.loginfo(packets.MUVZ[1])

        # W moving
        if (joystic_xyzw[3] <= -0.8 and joystic_xyzw[3] >= -1.0):
            pub3.publish(packets.MUVW[0]); #rospy.loginfo(packets.MUVW[0])
        if (joystic_xyzw[3] >= 0.8 and joystic_xyzw[3] <= 1.0):
            pub3.publish(packets.MUVW[1]); #rospy.loginfo(packets.MUVW[1])   
        
        # Speed value Change
        if speed_btn[0] == 1: 
            speed_func(100); pub3.publish(''.join(packets.SPEED)); rospy.loginfo(''.join(packets.SPEED))
        if speed_btn[1] == 1: 
            pub3.publish(''.join(packets.SPEED))
            # speed_func(-1000); pub3.publish(''.join(packets.SPEED)); rospy.loginfo(''.join(packets.SPEED))


        rate.sleep()
                     

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()




if __name__ == '__main__':
    listener()

