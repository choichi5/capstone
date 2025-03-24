#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Software License Agreement (BSD License)

import binascii
import rospy
import packets    # SCARA 패킷 명령어 저장되어 있는 사용자 헤더파일
from std_msgs.msg import String, Float32MultiArray
import struct
import numpy as np

import sys
import signal


'''

SCARA의 현재 위치에 대한 패킷은 다음과 같이 echo된다

02  // STX
FF  // Dummy
30  // flag
20 20 37 30 37 2E 30 32 31 20  // x축 xxxxx.xxx  위치 좌표는 소수점 아래 세 자리까지 계산됨
20 20 20 35 33 2E 33 33 31 20  // y축 xxxxx.xxx 
20 20 20 2D 33 2E 36 31 39 20  // z축 xxxxx.xxx 
20 2D 31 30 36 2E 36 30 31 20  // w축 xxxxx.xxx 
31  // ARM (로봇의 자세 여부)
03  // ETX
E4  // LRC

로봇 자세를 의미하는 ARM의 경우는 다음과 같이 분류된다.
    - 0: Left form (왼쪽으로 굽어져 있는 형태)
    - 1: right form (오른쪽으로 굽어져 있는 형태)
    - 2: no form (굽어져있지 않은 형태)

'''



# 변환행렬 상수
T_mtx = np.array([[-1,  0,  0, 920],
                  [0,  -1,  0,  90],
                  [0,   0,  1, 540],
                  [0,   0,  0,   1]])

# 최종 변환된 로봇 팔 end effect의 좌표 값을 저장하는 리스트 변수
final_scara = [0,0,0,0,0,0,0]
final_coordi = [0,0,0]
final_joint = [0,0,0,0]

RAD2DEG = 57.2958


## node 비정상 작동 시 강제 종료를 위한 함수 ##
def signal_handler(signal, frame): # ctrl + c -> exit program
        print('You pressed Ctrl+C!')
        sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)


def convert_to_float(str_value):
    try:
        return float(str_value)
    except ValueError:
        return None

def xyz_callback(data):
    global T_mtx, final_scara, RAD2DEG, final_coordi, final_joint

    # print(data.data)

    if(len(data.data) == 47):
        
        # XYZ 데이터일 경우
        if(data.data[0] == "1"): 
            # 수신된 메시지를 문자열로 추출
            x_hex = data.data[4:13]
            y_hex = data.data[14:23]
            z_hex = data.data[24:33]
            w_hex = data.data[34:43]
            arm_hex = data.data[44]
            
            
            # 16진수로 구성된 좌표 값을 float형으로 변환
            x_val = convert_to_float(x_hex)
            y_val = convert_to_float(y_hex)
            z_val = convert_to_float(z_hex)
            w_val = convert_to_float(w_hex)

            # print(x_val, y_val, z_val, w_val)
            scara_coordi = np.array([x_val, y_val, -(z_val+400), 1]) # 계산을 위해 SCARA 기준 위치 값 행렬에 대입
            sensor_coordi = T_mtx.dot(scara_coordi.T) # 위치변환 행렬과 행렬곱 진행
            
            np.set_printoptions(precision=3)   # 각 위치 값을 소수점 아래 세 자리까지만 출력 설정
            print(sensor_coordi)
            final_coordi = [sensor_coordi[0], sensor_coordi[1], sensor_coordi[2]]

            arm = convert_to_float(arm_hex)

            ## 현재 SCARA의 자세 정보 확인
            arm_states = {
                    0: "Left form",
                    1: "Right form",
                    2: "No form"
                }
            arm_status = arm_states.get(arm, "Unknown form")

            # print("ARM status: ", arm_status)


        # Joint 각도 데이터일 경우
        elif(data.data[0] == "2"): 
            # 수신된 메시지를 문자열로 추출
            x_hex2 = data.data[4:13]
            y_hex2 = data.data[14:23]
            z_hex2 = data.data[24:33]
            w_hex2 = data.data[34:43]
            
            
            # 16진수로 구성된 좌표 값을 float형으로 변환
            x_val2 = convert_to_float(x_hex2)
            y_val2 = convert_to_float(y_hex2)
            z_val2 = convert_to_float(z_hex2)
            w_val2 = convert_to_float(w_hex2)

            # 각도 값 배열에 저장
            final_joint = [x_val2, y_val2, z_val2, w_val2]


        # XYZ값과 관절 각도 값 저장
        final_scara[:3] = final_coordi     # end effect의 XYZ 좌표 값 저장
        # final_scara[3:7]= final_joint      # 각 관절의 현재 각도 값 저장
        np.set_printoptions(precision=3)   # 각 위치 값을 소수점 아래 세 자리까지만 출력 설정
        # print(np.array(final_scara))

    # else:
        # print("...adjusting....")








##############
### 메인함수 ###
##############

def reading():
    global final_scara

    #### node setting ####
    rospy.init_node('reading', anonymous=True)

    #### PUblish section ####
    pub = rospy.Publisher('scara_coordi', Float32MultiArray, queue_size=10)

    #### Subscribe section ####
    rospy.Subscriber('/read_scara', String, xyz_callback)       # SCARA 패킷 값 구독
    
    
    rate = rospy.Rate(10) # 100hz
    coordi_value = Float32MultiArray()

    #### 알고리즘 파트 ####
    while not rospy.is_shutdown():
        coordi_value.data = final_scara
        pub.publish(coordi_value)

        rate.sleep()
             
             

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    reading()