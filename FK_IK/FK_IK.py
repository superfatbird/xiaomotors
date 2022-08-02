# -*- coding: utf-8 -*-
"""
Created on Fri Jul 15 16:00:13 2022

@author: Dapeng
"""
import math
import numpy as np
r1 = 0.1
r2 = 0.1
# function to caculate foward kinematics, angle in radians
def fk(theta1, theta2):
    x = r1 * math.cos(theta1) + r2 * math.cos(theta1 + theta2)
    y = r1 * math.sin(theta1) + r2 * math.sin(theta1 + theta2)
    theta = theta1 + theta2
    return x, y, theta
# function to caculate jacobian, angle in radians
def jacobian(theta1, theta2):
    return [[-r1 * math.sin(theta1) - r2 * math.sin(theta1 + theta2), -r2 * math.sin(theta1 + theta2)],
            [r1 * math.cos(theta1) + r2 * math.cos(theta1 + theta2), r2 * math.cos(theta1 + theta2)],
            [1, 1]]
# try the function
# print([round(x,3) for x in fk(0.75 * math.pi, 1.5 * math.pi)])
# print([round(x,3) for x in fk(math.pi, math.pi)])
# print(fk(math.pi, math.pi))
# print(jacobian(math.pi, math.pi))
# for i in range(3):
#     print([round(x,3) for x in jacobian(math.pi, math.pi)[i]])
A = np.linspace(math.pi/6, math.pi * 11/6, 301)
for i in range(len(A)):
    [result[i],] = fk(A[i], 0)
