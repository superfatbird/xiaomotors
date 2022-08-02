import board
import busio
import digitalio
import time
import XL320
control = digitalio.DigitalInOut(board.D2)
control.direction = digitalio.Direction.OUTPUT
control.value = False
uart = busio.UART(board.TX, board.RX, baudrate=1000000)
motors = XL320.Dynamixel_XL320(uart,control)
color_numbers = 7
motor_numbers = 2
while True:
    for j in range (color_numbers):
        for i in range(motor_numbers):
            motors.set_register(i+1, 25, j+1)
        time.sleep(0.5)
    
    for i in range(motor_numbers):
        motors.set_register(i+1, 25, 0)
        time.sleep(0.001)
    
    time.sleep(1)
    
    for j in range (color_numbers):
        for i in range(motor_numbers):
            motors.set_register(i+1, 25, j+1)
            time.sleep(0.1)
    
    for i in range(motor_numbers):
        motors.set_register(i+1, 25, 0)
        time.sleep(0.001)
    
    time.sleep(1)