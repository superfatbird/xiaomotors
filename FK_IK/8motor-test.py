# -*- coding: utf-8 -*-
"""
Created on Sat Jul 23 09:19:25 2022

@author: Dapeng
"""

import time
from serial import Serial
ser = Serial('COM12', 38400, timeout = 0.5)
send_interval = 1
move_interval = 0.001
# ser.write(bytes('P20\n', 'ascii'))

for j in range(4):
    ser.write(bytes('M8\n', 'ascii'))
    time.sleep(send_interval)
    ser.write(bytes('B8\n', 'ascii'))
    time.sleep(send_interval)
ser.close()