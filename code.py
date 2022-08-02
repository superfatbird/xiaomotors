import board
import busio
import digitalio
import time
import XL320

control = digitalio.DigitalInOut(board.D12)
control.direction = digitalio.Direction.OUTPUT
control.value = True
uart = busio.UART(board.TX, board.RX, baudrate=1000000)
motors = XL320.Dynamixel_XL320(uart,control)

# Code usage examples

# --move motor to a position under joint mode
# 
# --set motor speed to 300
# motors.set_register_dual(2, 32, 300)
# --broadcast ping (more than 2 motors, otherwise no 14 to 28 bytes)and
# --print the 14th to 28 bytes from motor returned packets in HEX
# result = motors.ping(0xFE)
# print([hex(x) for x in result[14:28]])
# print("A ERR byte from Writedata: ", hex(motors.last_error))
# print("Ping 1 gets ", [hex(x) for x in motors.ping(1)])
# print("Ping 2 gets ", [hex(x) for x in motors.ping(2)])
# print("Ping 0xFE gets ", [hex(x) for x in motors.ping(0xFE,2)])
# print("Present temperature of 1 is ", motors.get_register(1, 46))
# print("Present temperature of 2 is ", motors.get_register(2, 46))
# motors.set_register(1, 0x19, 1) # Motor 1 LED to red
# motors.set_register(2, 25, 1)   # Motor 2 LED to red  
# motors.set_register(1, 25, 2)   # Motor 1 LED to green
# motors.set_register(2, 14, 125) # Set motor 2 voltage high limit to 12.5v
# --Bulk read (read temperature from motor1 and motor 2)
# motors.bulk_read([1, 46 & 0xFF, 46 >> 8 & 0xFF, 1 & 0xFF, 1 >> 8 & 0xFF,2, 46 & 0xFF, 46 >> 8 & 0xFF, 1 & 0xFF, 1 >> 8 & 0xFF])

result = motors.read_data(1, 4, 1)
print("The baudrate is ",result[9])
time.sleep(read_delay)

result = motors.read_data(1, 12, 1)
print("The Temp_Limit is ",result[9])
time.sleep(read_delay)

result = motors.read_data(1, 14, 1)
print("Max Vol limit is ",result[9]/10)
time.sleep(read_delay)

result = motors.read_data(1, 24, 1)
print("Torque Enabled? ",result[9])
time.sleep(read_delay)

result = motors.read_data(1, 45, 1)
print("Present voltage is ",result[9]/10)
time.sleep(read_delay)

result = motors.read_data(1, 46, 1)
print("Present temperature is ",result[9])
time.sleep(read_delay)

result = motors.read_data(1, 50, 1)
print("The HES is ", result[9])
print("The Full HES is ", [hex(x) for x in result])
time.sleep(read_delay)

while True:
    # ping motor with ID = 1
    result = motors.ping(1)
    print([hex(x) for x in result[0:14]])
    
    # convert bytearray to string
    # data_string = ''.join([chr(b) for b in result])
    print(hex(result[8]))
    
         