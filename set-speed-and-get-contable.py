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
Motor_ID = 6
color_numbers = 7
for i in range(8):
        motors.set_register_dual(i+1, 32, 500)
        time.sleep(0.001)
# Get the information from EEPROM Area
print("The model is ", motors.get_register_dual(Motor_ID,0))
print("The Firmware Version is ", motors.get_register(Motor_ID,2))
print("The ID is ", motors.get_register(Motor_ID,3))
print("The Baud Rate is ", motors.get_register(Motor_ID,4))
print("The Return Delay time is ", motors.get_register(Motor_ID,5))
print("The CW Angle limit is ", motors.get_register_dual(Motor_ID,6))
print("The CCW Angle limit is ", motors.get_register_dual(Motor_ID,8))
print("The Control Mode is ", motors.get_register(Motor_ID,11))
print("The Temperature Limit is ", motors.get_register(Motor_ID,12))
print("The Min Volotage Limit is ", motors.get_register(Motor_ID,13))
print("The Max Volotage Limit is ", motors.get_register(Motor_ID,14))
print("The Max Torque is ", motors.get_register_dual(Motor_ID,15))
print("The Status Return Level is ",motors.get_register(Motor_ID, 17))
print("The Shutdown Error Info is ", motors.get_register(Motor_ID,18))
# Get the information from RAM Area
print("The Torque Enable is ",motors.get_register(Motor_ID, 24))
print("The LED is ",motors.get_register(Motor_ID, 25))
print("D Gain is ",motors.get_register(Motor_ID, 27))
print("I Gain is ",motors.get_register(Motor_ID, 28))
print("P Gain is ",motors.get_register(Motor_ID, 29))
print("The Goal Position is ", motors.get_register_dual(Motor_ID, 30))
print("The Moving speed is ", motors.get_register_dual(Motor_ID, 32))
print("The Torque Limit is ", motors.get_register_dual(Motor_ID, 35))
print("The Present Position is ", motors.get_register_dual(Motor_ID, 37))
print("The Present speed is ", motors.get_register_dual(Motor_ID, 39))
print("The Present Load is ", motors.get_register_dual(Motor_ID, 41))
print("Present voltage is ",motors.get_register(Motor_ID, 45))
print("Present temperature is ", motors.get_register(Motor_ID, 46))
print("Registered Instruction? ", motors.get_register(Motor_ID, 47))
print("Moving? ", motors.get_register(Motor_ID, 49))
print("The Hardware Error Status is ", motors.get_register(Motor_ID, 50))
print("Punch? ", motors.get_register_dual(Motor_ID, 51))
while True:
    for i in range(color_numbers):
        motors.set_register(Motor_ID, 25, i)
        time.sleep(0.1)    