# -*- coding: utf-8 -*-
"""
Created on Thu Jul 28 16:06:09 2022

@author: Dapeng
"""

import time
# import random
from serial import Serial
ser = Serial('COM12', 38400, timeout = 0.5)
send_interval_1 = 0.1
move_interval_1 = 1
send_interval_2 = 0.04
move_interval_2 = 0.3
motor_1 = 'P1'
motor_2 = 'P3'
offset = 100
forward = 300
for i in range(16):
    cmd = motor_1+str(512 + offset - forward * (i % 2))+'\n'
    ser.write(bytes(cmd, 'ascii'))
    time.sleep(send_interval_2)
    cmd = motor_2+str(512 - offset + forward * (i % 2))+'\n'
    ser.write(bytes(cmd, 'ascii'))
    time.sleep(move_interval_2)
ser.close()
# ser.write(bytes('P20\n', 'ascii'))

# for j in range(11):
#     ser.write(bytes('P1'+str(random.randint(200, 510))+'\n', 'ascii'))
#     time.sleep(send_interval_1)
#     ser.write(bytes('P3'+str(200)+'\n', 'ascii'))
#     time.sleep(move_interval_1)    
#     ts0 = time.time()
#     for i in range(10):
#         motor = 'P3'
#         cmd = motor+str(200 + 30 * i)+'\n'
#         ser.write(bytes(cmd, 'ascii'))
#         time.sleep(send_interval_1)

    
    #print(time.time() - ts0)
    # ser.write(bytes('P3'+str(random.randint(0, 510))+'\n', 'ascii'))
    # time.sleep(move_interval)
    # ser.write(bytes('P3'+str(random.randint(0, 510))+'\n', 'ascii'))
    # time.sleep(move_interval)
    # ser.write(bytes('P3'+str(random.randint(0, 510))+'\n', 'ascii'))
    # time.sleep(move_interval)
    # if j % 2 == 0 :
    #     ser.write(bytes('P1'+str(random.randint(300, 750))+'\n', 'ascii'))
    #     time.sleep(move_interval)
    #     ser.write(bytes('P3'+str(random.randint(100, 500))+'\n', 'ascii'))
    #     time.sleep(move_interval)
    #     ser.write(bytes('P3'+str(random.randint(500, 900))+'\n', 'ascii'))
    #     time.sleep(move_interval)
    # if j % 2 == 1 :
    #     ser.write(bytes('P1'+str(random.randint(300, 750))+'\n', 'ascii'))
    #     time.sleep(move_interval)
    #     ser.write(bytes('P3'+str(random.randint(100, 500))+'\n', 'ascii'))
    #     time.sleep(move_interval)
    #     ser.write(bytes('P3'+str(random.randint(500, 900))+'\n', 'ascii'))
    #     time.sleep(move_interval)
    # time.sleep(send_interval)