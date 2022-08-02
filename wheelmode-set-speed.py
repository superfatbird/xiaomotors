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
for i in range(8):
    # turn off the torque
    motors.set_register(i+1, 24, 0)
    time.sleep(0.001)
    # set to wheel mode
    motors.set_register(i+1, 11, 1)
    time.sleep(0.001)
    # turn on the torque
    motors.set_register(i+1, 24, 1)
    time.sleep(0.001)
    motors.set_register_dual(i+1, 32, 10)
    time.sleep(0.001)
while True:
    for i in range(8):
        motors.set_register_dual(i+1, 32, 500)
        time.sleep(0.001)
    time.sleep(2)
    for i in range(8):
        motors.set_register_dual(i+1, 32, 10)
        time.sleep(0.001)
    time.sleep(2)
    for i in range(8):
        motors.set_register_dual(i+1, 32, 1524)
        time.sleep(0.001)
    time.sleep(2)
    for i in range(8):
        motors.set_register_dual(i+1, 32, 1034)
        time.sleep(0.001)
    time.sleep(2)