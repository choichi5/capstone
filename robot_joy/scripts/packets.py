# -*- coding: utf-8 -*-
# Servo On Packet
# import numpy as np


#### Protocol Constant ####
STX = 0x02
EXT = 0x03
ACM = 0x06
NAK = 0x15
RST = 0x12
DUMMY = 0xFF
# LGT is not an Constant. You should calculate it by XOR 
# from DUMMY to in front of EXT 


############################# Packets #########################




## Servo Motor On ##
SVON = ''.join([chr(STX),
                chr(DUMMY),
                chr(0x44),   # D
                chr(0x42),   # B
                chr(0x30),   # ch.1
                chr(0x31),   # on(1)
                chr(EXT),
                chr(0xF8),   # LGT
                chr(ACM)])


## Servo Motor Off ## 
SVOFF = ''.join([chr(STX),
                 chr(DUMMY),
                 chr(0x44),   # D
                 chr(0x42),   # B
                 chr(0x30),   # ch.1
                 chr(0x30),   # off(0)
                 chr(EXT),
                 chr(0xF9),   # LGT
                 chr(ACM)])











###########################
## XYZ Coordinate Moving ##
########################### 

## X Coordi. Moving ##
MUVX = [''.join([chr(STX),
                 chr(DUMMY),
                 chr(0x42),   # B
                 chr(0x45),   # E
                 chr(0x30),   # ch.1
                 chr(0x30),   # Axis(x)
                 chr(0x31),   # Direction (+)
                 chr(0x31),   # Motion Type (XYZ)
                 chr(EXT),
                 chr(0xF8),   # LGT
                 chr(0xCF),
                 chr(ACM)]),

        ''.join([chr(STX),
                 chr(DUMMY),
                 chr(0x42),   # B
                 chr(0x45),   # E
                 chr(0x30),   # ch.1
                 chr(0x30),   # Axis(X)
                 chr(0x30),   # Direction (-)
                 chr(0x31),   # Motion Type (XYZ)
                 chr(EXT),
                 chr(0xF9),   # LGT
                 chr(0xCF),
                 chr(ACM)])
            ]


## Y Coordi. Moving ##
MUVY = [''.join([chr(STX),
                 chr(DUMMY),
                 chr(0x42),   # B
                 chr(0x45),   # E
                 chr(0x30),   # ch.1
                 chr(0x31),   # Axis(Y)
                 chr(0x31),   # Direction (+)
                 chr(0x31),   # Motion Type (XYZ)
                 chr(EXT),
                 chr(0xF9),   # LGT
                 chr(0xCF),
                 chr(ACM)]),

        ''.join([chr(STX),
                 chr(DUMMY),
                 chr(0x42),   # B
                 chr(0x45),   # E
                 chr(0x30),   # ch.1
                 chr(0x31),   # Axis(Y)
                 chr(0x30),   # Direction (-)
                 chr(0x31),   # Motion Type (XYZ)
                 chr(EXT),
                 chr(0xF8),   # LGT
                 chr(0xCF),
                 chr(ACM)])
            ]


## Z Coordi. Moving ##
MUVZ = [''.join([chr(STX),
                 chr(DUMMY),
                 chr(0x42),   # B
                 chr(0x45),   # E
                 chr(0x30),   # ch.1
                 chr(0x32),   # Axis(Z)
                 chr(0x31),   # Direction (+)
                 chr(0x31),   # Motion Type (XYZ)
                 chr(EXT),
                 chr(0xFA),   # LGT
                 chr(0xCF),
                 chr(ACM)]),

        ''.join([chr(STX),
                 chr(DUMMY),
                 chr(0x42),   # B
                 chr(0x45),   # E
                 chr(0x30),   # ch.1
                 chr(0x32),   # Axis(Z)
                 chr(0x30),   # Direction (-)
                 chr(0x31),   # Motion Type (XYZ)
                 chr(EXT),
                 chr(0xFB),   # LGT
                 chr(0xCF),
                 chr(ACM)])
            ]


## W Coordi. Moving ##
MUVW = [''.join([chr(STX),
                 chr(DUMMY),
                 chr(0x42),   # B
                 chr(0x45),   # E
                 chr(0x30),   # ch.1
                 chr(0x33),   # Axis(W)
                 chr(0x31),   # Direction (+)
                 chr(0x31),   # Motion Type (XYZ)
                 chr(EXT),
                 chr(0xFB),   # LGT
                 chr(0xCF),
                 chr(ACM)]),

        ''.join([chr(STX),
                 chr(DUMMY),
                 chr(0x42),   # B
                 chr(0x45),   # E
                 chr(0x30),   # ch.1
                 chr(0x33),   # Axis(W)
                 chr(0x30),   # Direction (-)
                 chr(0x31),   # Motion Type (XYZ)
                 chr(EXT),
                 chr(0xFA),   # LGT
                 chr(0xCF),
                 chr(ACM)])
            ]




## Servo Motor Speed
SPEED = [chr(STX),
         chr(DUMMY),
         chr(0x43),   # C
         chr(0x42),   # B
         chr(0x30),   # 10000
         chr(0x30),   # 1000
         chr(0x31),   # 100
         chr(0x30),   # 10
         chr(0x30),   # 1
         chr(EXT),
         chr(0xCF),   # LRC
         chr(ACM)]



## Servo Motor Present Coordi.(in XYZ axis)
COORDI = [chr(STX),
         chr(DUMMY),
         chr(0x41),   # A
         chr(0x43),   # C
         chr(0x30),   # ch.1
         chr(0x32),   # 0: Encorder / 1: Joint / 2: XYZ Standard   => 2 used
         chr(EXT),    
         chr(0xFF),   # LRC
         chr(ACM)]

JOINT  = [chr(STX),
          chr(DUMMY),
          chr(0x41),   # A
          chr(0x43),   # C
          chr(0x30),   # ch.1
          chr(0x31),   # 0: Encorder / 1: Joint / 2: XYZ Standard => 1 used
          chr(EXT),    
          chr(0xFC),   # LRC
          chr(ACM)]


START_POS = [chr(val) for val in [
               STX, DUMMY, 
               0x42, 0x43,       # BC
               0x30,             # ch.1
               0x30,             # 모션 Type: Joint 기준 이동
               0x30,             # 좌표계 단위: 도(Deg)
               0x20, 0x20, 0x20, 0x20, 0x2D, 0x33, 0x2E, 0x39, 0x37, 0x33,  # Axis 1 command
               0x20, 0x20, 0x20, 0x20, 0x32, 0x35, 0x2E, 0x35, 0x34, 0x33,  # Axis 2 command
               0x20, 0x20, 0x20, 0x20, 0x35, 0x34, 0x2E, 0x38, 0x39, 0x37,  # Axis 3 command
               0x20, 0x20, 0x20, 0x20, 0x35, 0x30, 0x2E, 0x30, 0x39, 0x34,  # Axis 4 command
               EXT, 
               0xD7              # LRC
               ]]
# 54.897 = 35 34 2E 38 39 37
# 61.777 = 36 31 2E 37 37 37