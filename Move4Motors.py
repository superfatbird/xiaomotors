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
while True:
    for i in range(4):
        motors.set_position(i+1, 0)
        time.sleep(0.001)
    time.sleep(2)
    for i in range(4):
        motors.set_position(i+1, 500)
        time.sleep(0.001)
    time.sleep(2)
    for i in range(4):
        motors.set_position(i+5, 0)
        time.sleep(0.001)
    time.sleep(2)
    for i in range(4):
        motors.set_position(i+5, 500)
        time.sleep(0.001)
    time.sleep(2)