# -*- coding: utf-8 -*-
"""
Created on Fri Jul 15 08:04:32 2022

@author: Dapeng
"""
a = None
b = None
from add_CRC import add_crc
a = [0x01]
b = a
add_crc(0,b)
print([hex(x) for x in b])
a = [0x01]