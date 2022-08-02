# The MIT License (MIT)
#
# Copyright (c) 2022 Dapeng Zhang
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

"""
`dynamixel-XL320`
================================================================================

Circuitpython driver library for the Dynamixel XL320 from Robotis.
The main different between different motors are the control table items. 

Dynamixels XL320
https://emanual.robotis.com/docs/en/dxl/x/xl320/#torque-enable

DYNAMIXEL Protocol 2.0 
https://emanual.robotis.com/docs/en/dxl/protocol2/

Lucian Copeland's dynamixel library
https://github.com/hierophect/Hierophect_Circuitpython_Dynamixel

This was inspired by Lucian Copeland's dynamixel library
**Software and Dependencies:**

# * Adafruit's Circuitpython Releases: https://github.com/adafruit/circuitpython/releases
# * Adafruit's Bus Device library: https://github.com/adafruit/Adafruit_CircuitPython_BusDevice
# * Adafruit's Register library: https://github.com/adafruit/Adafruit_CircuitPython_Register

* Author(s):

    - Dapeng
"""

__version__ = "0.0.0-auto.0"
# __repo__ = "https://github.com/.../....git"

import time
from micropython import const
from makepacket import add_crc

# Control table Addresses, this is Motor determined, ref XL320 emanual
# XL320 has different control table to other motors like AX12
# EEPROM
DYN_REG_MODEL_NUMBER_L          = const(0x00)
# DYN_REG_MODEL_NUMBER_H          = const(0x01)
DYN_REG_FIRMWARE_VER            = const(0x02)
DYN_REG_ID                      = const(0x03)
DYN_REG_BAUD                    = const(0x04)
DYN_REG_RETURN_DELAY            = const(0x05)
DYN_REG_CW_ANGLE_LIMIT_L        = const(0x06)
# DYN_REG_CW_ANGLE_LIMIT_H        = const(0x07)
DYN_REG_CCW_ANGLE_LIMIT_L       = const(0x08)
# DYN_REG_CCW_ANGLE_LIMIT_H       = const(0x09)
# -- Reserved = const(0x00)
DYN_REG_CONTROL_MODE			= const(0x0B)
DYN_REG_LIMIT_MAX_TEMP			= const(0x0C)
DYN_REG_LIMIT_MIN_VOLT          = const(0x0D)
DYN_REG_LIMIT_MAX_VOLT          = const(0x0E)
DYN_REG_MAX_TORQUE_L            = const(0x0F)
# DYN_REG_MAX_TORQUE_H            = const(0x10)
DYN_REG_STATUS_RETURN_LEVEL     = const(0x11)
DYN_REG_ALARM_SHUTDOWN          = const(0x12)

# RAM
DYN_REG_TORQUE_ENABLE           = const(0x18)
DYN_REG_LED                     = const(0x19)
# -- Reserved = const(0x00)
DYN_REG_D_GAIN					= const(0x1B)
DYN_REG_I_GAIN					= const(0x1C)
DYN_REG_P_GAIN					= const(0x1D)
DYN_REG_GOAL_POSITION_L         = const(0x1E)
# DYN_REG_GOAL_POSITION_H       = const(0x1F) #with this does not work, in the write data function
DYN_REG_MOVING_SPEED_L          = const(0x20)
# DYN_REG_MOVING_SPEED_H          = const(0x21)
# -- Reserved = const(0x00)
DYN_REG_TORQUE_LIMIT_L          = const(0x23)
# DYN_REG_TORQUE_LIMIT_H          = const(0x24)
DYN_REG_PRESENT_POSITION_L      = const(0x25)
# DYN_REG_PRESENT_POSITION_H      = const(0x26)
DYN_REG_PRESENT_SPEED_L         = const(0x27)
# DYN_REG_PRESENT_SPEED_H         = const(0x28)
DYN_REG_PRESENT_LOAD_L          = const(0x29)
# DYN_REG_PRESENT_LOAD_H          = const(0x2A)
# -- Reserved = const(0x00)
# -- Reserved = const(0x00)
DYN_REG_PRESENT_VOLTAGE         = const(0x2D)
DYN_REG_PRESENT_TEMP            = const(0x2E)
DYN_REG_REGISTERED_INST         = const(0x2F)
# -- Reserved = const(0x00)
DYN_REG_MOVING                  = const(0x31)
DYN_REG_HARDWARE_ERR_STATUS		= const(0x32)
DYN_REG_PUNCH_L                 = const(0x33)
DYN_REG_PUNCH_H                 = const(0x34)

# Dynamixel Error status,this is protocol 2.0 determined
DYN_ERR_NONE					= const(0x00)
DYN_ERR_RESULT_FAIL				= const(0x01)
DYN_ERR_INST_ERROR				= const(0x02)
DYN_ERR_CRC_ERROR				= const(0x03)
DYN_ERR_DATA_RANGE				= const(0x04)
DYN_ERR_DATA_LENGTH				= const(0x05)
DYN_ERR_DATA_LIMIT				= const(0x06)
DYN_ERR_ACCESS					= const(0x07)
DYN_ERR_ALERT                   = const(0x80)
# this need attention
DYN_ERR_INVALID                 = const(0x08)

# Dynamixel Instruction, this is protocol 2.0 determined
DYN_INST_PING					= const(0x01)
DYN_INST_READ					= const(0x02)
DYN_INST_WRITE					= const(0x03)
DYN_INST_REG_WRITE				= const(0x04)
DYN_INST_ACTION					= const(0x05)
DYN_INST_FACTORY_RESET			= const(0x06)
DYN_INST_REBOOT					= const(0x08)
DYN_INST_CLEAR					= const(0x10) # ONLY Protocol 2.0
DYN_INST_STATUS					= const(0x55) # ONLY Protocol 2.0 (Return)
DYN_INST_SYNC_READ				= const(0x82) # ONLY Protocol 2.0
DYN_INST_FAST_SYNC_READ			= const(0x8A) # ONLY Protocol 2.0
DYN_INST_SYNC_WRITE				= const(0x83)
DYN_INST_BULK_READ				= const(0x92)
DYN_INST_BULK_WRITE				= const(0x93) # ONLY Protocol 2.0
DYN_INST_FAST_BULK_READ			= const(0x9A) # ONLY Protocol 2.0

DYN_BROADCAST_ID                = const(0xFE)

# Instruction Packet (send packet from controller's view)
# | 0xFF | 0xFF | 0xFD | 0x00 | ID | LEN_L | LEN_H | INST | PARAM_1-PARAM_N | CRC_L | CRC_H |

# Status Packet (recieved packet  from controller's view)
# | 0xFF | 0xFF | 0xFD | 0x00 | ID | LEN_L | LEN_H | INST | ERR | PARAM_1-PARAM_N | CRC_L | CRC_H |

# Length = the number of Parameters + 3

# Status packet, length =  the number of Parameters + 3 + 1(ERR Byte)
# https://emanual.robotis.com/docs/en/dxl/protocol2/ 5.1.3.3, ATTN: LEN_L = 0x07 

# LENGTH takes two bits example as in the following link, there are six parameters
# https://emanual.robotis.com/docs/en/dxl/protocol2/ 5.3.3.2, ATTN: P1 P2, LEN_L = 0x09

class Dynamixel_XL320:
    def __init__(self, uart, direction_pin):
        self._uart = uart
        self._dir = direction_pin
        # Disable RX when not explicitly reading
        self._dir.value = True
        self.last_error = DYN_ERR_INVALID

    # -----------------------
    # Dynamixel Instructions:
    # -----------------------

    # dyn_id is the motor id(0xFE is the broadcast ID), if broadcast ID used, How
    # many Bytes will be recieved is motor number determined, each motor send back
    # 14 Byte packet about to respond the ping()
    # currently return full status packet
    def ping(self, dyn_id, n_motor = 1):
        length = 3 # no parameter, 0+3 = 3
        if dyn_id == DYN_BROADCAST_ID and n_motor ==1:
            print("Number of connected motor argument needed")
            return 0
        
        # Prepare a ping instruction packet as protocal 2.0 needs 
        # HEADER PART is "0xFF, 0xFF, 0xFD, 0x00"
        # ID PART is "dyn_id"
        # LENGTH PART is "length & 0xFF and length >> 8 & 0xFF", this convert length to LEN_L and LEN_H to fit the protocal 2.0 packet 
        # After LENGTH PART is the INSTRUCTION PART, here is "DYN_INST_PING"
        
        data = [0xFF, 0xFF, 0xFD, 0x00, dyn_id, length & 0xFF, length >> 8 & 0xFF, DYN_INST_PING]
        # Then the data need to add CRC16 Bytes 
        add_crc(0, data)
        array = bytes(data)
        # Write out the request
        self._uart.write(array)
        time.sleep(0.0005) # critical, 0.001 will miss first few Bytes, Delay required to avoid premature opening of RX
        # Open the RX line and read data
        self._dir.value = False
        # 30 is temporal number, need a way to figure out how to handle this smartly
        packet = self._uart.read(14*n_motor) # how long to read is number of motors determined...
        self._dir.value = True
        time.sleep(0.001)
        # Check if packet timed out and is empty
        if packet is None:
            raise RuntimeError("Could not find motor at supplied address")
        # Otherwise, return the error
        self.last_error = packet[8]
        return packet
        # return packet[8:11]
    
    # dyn_id is the motor id(0xFE is the broadcast ID). It is a number
    # reg_addr is where to read data. It is a number(NOT a list).
    # nbytes is how many bytes to read from the reg_addr. It is a number
    def read_data(self, dyn_id, reg_addr, nbytes):
        length = 7 # address takes 2 bytes in the packet, nbytes takes 2 bytes, 2+2+3 = 7
        if dyn_id == DYN_BROADCAST_ID:
            print("Read Instruction does not respond to Broadcast")
            return 0
        # Prepare a read instruction packet as protocal 2.0 needs 
        # HEADER PART is "0xFF, 0xFF, 0xFD, 0x00"
        # ID PART is "dyn_id"
        # LENGTH PART is "length & 0xFF and length >> 8 & 0xFF", this convert length to LEN_L and LEN_H to fit the protocal 2.0 packet 
        # After LENGTH PART is the INSTRUCTION PART, here is "DYN_INST_READ"
        # ADDRESS PART is "reg_addr & 0xFF , reg_addr >> 8 & 0xFF" , this convert reg_addr to ADDR_L and ADDR_H to fit the protocal 2.0 packet 
        # N_BYTE PART is "nbytes & 0xFF, nbytes >> 8 & 0xFF"
        data = [0xFF, 0xFF, 0xFD, 0x00, dyn_id, length & 0xFF, length >> 8 & 0xFF,
        DYN_INST_READ, reg_addr & 0xFF , reg_addr >> 8 & 0xFF, nbytes & 0xFF, nbytes >> 8 & 0xFF]
        
        # Then the data need to add CRC16 Bytes 
        add_crc(0, data)
        # Convert list to bytes for sending
        array = bytes(data)
        # Write out the request
        self._uart.write(array)
        time.sleep(0.0005) # Critical, 0.001 will miss first few Bytes, Delay required to avoid premature opening of RX
        # Open the RX line and read data
        self._dir.value = False
        # 30 is temporal number, need a way to figure out how to handle this smartly
        packet = self._uart.read(11 + nbytes)
        #packet = self._uart.read(nbytes+6)
        self._dir.value = True
        time.sleep(0.001)
        # Check if packet timed out and is empty
        if packet is None:
            raise RuntimeError("Could not find motor at supplied address")
        self.last_error = packet[8]
        return packet
        
    # Write data without check responds, for debugging    
    def write_data_no_re(self, dyn_id, reg_addr, parameters):
        length = len(parameters) + 3 + 2 # reg_addr need to added, the 2 is for that    
        # Prepare a write instruction packet as protocal 2.0 needs 
        # HEADER PART is "0xFF, 0xFF, 0xFD, 0x00"
        # ID PART is "dyn_id"
        # LENGTH PART is "length & 0xFF and length >> 8 & 0xFF", this convert length to LEN_L and LEN_H to fit the protocal 2.0 packet 
        # After LENGTH PART is the INSTRUCTION PART, here is "DYN_INST_WRITE"
        # ADDRESS PART is "reg_addr & 0xFF , reg_addr >> 8 & 0xFF" , this convert reg_addr to ADDR_L and ADDR_H to fit the protocal 2.0 packet 
        data = [0xFF, 0xFF, 0xFD, 0x00, dyn_id, length & 0xFF, length >> 8 & 0xFF,
        DYN_INST_WRITE, reg_addr & 0xFF , reg_addr >> 8 & 0xFF]
        data.extend(parameters)
        # Add CRC16 Bytes to the data
        add_crc(0, data)
        array = bytes(data)
        # Write the data
        self._uart.write(array)
    
    # dyn_id is the motor id(0xFE is the broadcast ID). It is a number
    # reg_addr is where to write data. It is a number(NOT a list).
    # parameters is the data to write into the reg_addr. It is a list.
    def write_data(self, dyn_id, reg_addr, parameters):
        length = len(parameters) + 3 + 2 # reg_addr need to added, the 2 is for that    
        # Prepare a write instruction packet as protocal 2.0 needs 
        # HEADER PART is "0xFF, 0xFF, 0xFD, 0x00"
        # ID PART is "dyn_id"
        # LENGTH PART is "length & 0xFF and length >> 8 & 0xFF", this convert length to LEN_L and LEN_H to fit the protocal 2.0 packet 
        # After LENGTH PART is the INSTRUCTION PART, here is "DYN_INST_WRITE"
        # ADDRESS PART is "reg_addr & 0xFF , reg_addr >> 8 & 0xFF" , this convert reg_addr to ADDR_L and ADDR_H to fit the protocal 2.0 packet 
        data = [0xFF, 0xFF, 0xFD, 0x00, dyn_id, length & 0xFF, length >> 8 & 0xFF,
        DYN_INST_WRITE, reg_addr & 0xFF , reg_addr >> 8 & 0xFF]
        data.extend(parameters)
        # Add CRC16 Bytes to the data
        add_crc(0, data)
        array = bytes(data)
        # Write the data
        self._uart.write(array)
        time.sleep(0.0005) # # critical, 0.001 will miss first few Bytes, Delay required to avoid premature opening of RX
        # Check for error only if this is not broadcast mode, if it is broadcast mode
        # no status packet return
        if dyn_id != DYN_BROADCAST_ID:
            # Open the RX line, but only return error
            self._dir.value = False
            # read more than the motor send, takes long time out....
            packet = self._uart.read(11)
            self._dir.value = True
            time.sleep(0.001)
            # Check if packet timed out and is empty
            if packet is None:
                raise RuntimeError("Could not find motor at supplied address")
            # self._uart.reset_input_buffer()
            self.last_error = packet[8]
        else:
            self.last_error = DYN_ERR_INVALID

    # dyn_id is the motor id(0xFE is the broadcast ID). It is a number
    # reg_addr is where to write data. It is a number(NOT a list).
    # parameters is the data to write into the reg_addr. It is a list.
    # more than 1 parameters can be put in this list, for XL320, 1 parameter
    # with 2 Bytes will be fine.
    def reg_write(self, dyn_id, reg_addr, parameters):
        length = len(parameters) + 3 + 2 # reg_addr need to added, the 2 is for that    
        # Prepare a reg_write instruction packet as protocal 2.0 needs 
        # HEADER PART is "0xFF, 0xFF, 0xFD, 0x00"
        # ID PART is "dyn_id"
        # LENGTH PART is "length & 0xFF and length >> 8 & 0xFF", this convert length to LEN_L and LEN_H to fit the protocal 2.0 packet 
        # After LENGTH PART is the INSTRUCTION PART, here is "DYN_INST_REG_WRITE"
        # ADDRESS PART is "reg_addr & 0xFF , reg_addr >> 8 & 0xFF" , this convert reg_addr to ADDR_L and ADDR_H to fit the protocal 2.0 packet 
        data = [0xFF, 0xFF, 0xFD, 0x00, dyn_id, length & 0xFF, length >> 8 & 0xFF,
        DYN_INST_REG_WRITE, reg_addr & 0xFF , reg_addr >> 8 & 0xFF]
        data.extend(parameters)
        # Add CRC16 Bytes to the data
        add_crc(0, data)
        array = bytes(data)
        # Write the data
        self._uart.write(array)
        time.sleep(0.0005)
        if dyn_id != DYN_BROADCAST_ID:
            self._dir.value = False
            packet = self._uart.read(11)
            self._dir.value = True
            time.sleep(0.001)
            if packet is None:
                raise RuntimeError("Could not find motor at supplied address")
            self.last_error = packet[8]
        else:
            self.last_error = DYN_ERR_INVALID
    
    # dyn_id is the motor id(0xFE is the broadcast ID). It is a number
    def action(self, dyn_id):
        length = 3
        # Prepare an action instruction packet as protocal 2.0 needs 
        # HEADER PART is "0xFF, 0xFF, 0xFD, 0x00"
        # ID PART is "dyn_id"
        # LENGTH PART is "length & 0xFF and length >> 8 & 0xFF", this convert length to LEN_L and LEN_H to fit the protocal 2.0 packet 
        # After LENGTH PART is the INSTRUCTION PART, here is "DYN_INST_ACTION"
        # ADDRESS PART is "reg_addr & 0xFF , reg_addr >> 8 & 0xFF" , this convert reg_addr to ADDR_L and ADDR_H to fit the protocal 2.0 packet 
        data = [0xFF, 0xFF, 0xFD, 0x00, dyn_id, length & 0xFF, length >> 8 & 0xFF, DYN_INST_ACTION]
        # Add CRC16 Bytes to the data
        add_crc(0, data)
        array = bytes(data)
        # Write the data
        self._uart.write(array)
        time.sleep(0.0005)
        if dyn_id != DYN_BROADCAST_ID: # BROADCAST can be used to write, but not responded from motors, don't check
            self._dir.value = False
            packet = self._uart.read(11)
            self._dir.value = True
            time.sleep(0.001)
            if packet is None:
                raise RuntimeError("Could not find motor at supplied address")
            self.last_error = packet[8]
        else:
            self.last_error = DYN_ERR_INVALID
    
    # dyn_id is the motor id(0xFE is the broadcast ID). It is a number
    # parameter is a number, 0xFF, 1 or 2; 0xFF : Reset all; 0x01 : Reset all except ID ; 0x02 : Reset all except ID and Baudrate
    # In case of when Packet ID is a Broadcast ID 0xFE and Option is Reset All 0xFF, 
    def factory_reset(self, dyn_id, parameter):
        length = 3 + 1 # One parameter as 0xFF : Reset all; 0x01 : Reset all except ID ; 0x02 : Reset all except ID and Baudrate
        if dyn_id == DYN_BROADCAST_ID and parameter == 0xFF:
            print("Factory Reset Instruction(0x06) will NOT be activated.")
            return 0
        # Prepare an factory_reset instruction packet as protocal 2.0 needs 
        # HEADER PART is "0xFF, 0xFF, 0xFD, 0x00"
        # ID PART is "dyn_id"
        # LENGTH PART is "length & 0xFF and length >> 8 & 0xFF", this convert length to LEN_L and LEN_H to fit the protocal 2.0 packet 
        # After LENGTH PART is the INSTRUCTION PART, here is "DYN_INST_FACTORY_RESET"
        # ADDRESS PART is "reg_addr & 0xFF , reg_addr >> 8 & 0xFF" , this convert reg_addr to ADDR_L and ADDR_H to fit the protocal 2.0 packet 
        data = [0xFF, 0xFF, 0xFD, 0x00, dyn_id, length & 0xFF, length >> 8 & 0xFF, DYN_INST_FACTORY_RESET, parameter]
        # Add CRC16 Bytes to the data
        add_crc(0, data)
        array = bytes(data)
        # Write the data
        self._uart.write(array)
        time.sleep(0.0005) # # critical, 0.001 will miss first few Bytes, Delay required to avoid premature opening of RX
        # Check for error only if this is not broadcast mode, if it is broadcast mode
        # no status packet return
        if dyn_id != DYN_BROADCAST_ID:
            # Open the RX line, but only return error
            self._dir.value = False
            # read more than the motor send, takes long time out....
            packet = self._uart.read(11)
            self._dir.value = True
            time.sleep(0.001)
            # Check if packet timed out and is empty
            if packet is None:
                raise RuntimeError("Could not find motor at supplied address")
            # self._uart.reset_input_buffer()
            self.last_error = packet[8]
        else:
            self.last_error = DYN_ERR_INVALID

    def reboot(self, dyn_id):
        length = 3 # no parameter
        
        # Prepare an reboot instruction packet as protocal 2.0 needs 
        # HEADER PART is "0xFF, 0xFF, 0xFD, 0x00"
        # ID PART is "dyn_id"
        # LENGTH PART is "length & 0xFF and length >> 8 & 0xFF", this convert length to LEN_L and LEN_H to fit the protocal 2.0 packet 
        # After LENGTH PART is the INSTRUCTION PART, here is "DYN_INST_REBOOT"
        # ADDRESS PART is "reg_addr & 0xFF , reg_addr >> 8 & 0xFF" , this convert reg_addr to ADDR_L and ADDR_H to fit the protocal 2.0 packet 
        data = [0xFF, 0xFF, 0xFD, 0x00, dyn_id, length & 0xFF, length >> 8 & 0xFF, DYN_INST_REBOOT]
        # Add CRC16 Bytes to the data
        add_crc(0, data)
        array = bytes(data)
        # Write the data
        self._uart.write(array)
        time.sleep(0.0005) # # critical, 0.001 will miss first few Bytes, Delay required to avoid premature opening of RX
        # Check for error only if this is not broadcast mode, if it is broadcast mode
        # no status packet return
        if dyn_id != DYN_BROADCAST_ID:
            # Open the RX line, but only return error
            self._dir.value = False
            # read more than the motor send, takes long time out....
            packet = self._uart.read(11)
            self._dir.value = True
            time.sleep(0.001)
            # Check if packet timed out and is empty
            if packet is None:
                raise RuntimeError("Could not find motor at supplied address")
            # self._uart.reset_input_buffer()
            self.last_error = packet[8]
        else:
            self.last_error = DYN_ERR_INVALID
    
    # dyn_ids is a list, list of moter ids, the data will be read from
    # nbytes is number of bytes to read from the reg_addr
    def sync_read(self, reg_addr, nbytes, dyn_ids):
        length = len(dyn_ids) + 3 + 2 + 2# reg_addr takes the extra 2, nbytes takes extra 2  
                   
        # Prepare a sync_read instruction packet as protocal 2.0 needs 
        # HEADER PART is "0xFF, 0xFF, 0xFD, 0x00"
        # ID PART is "DYN_BROADCAST_ID"
        # LENGTH PART is "length & 0xFF and length >> 8 & 0xFF", this convert length to LEN_L and LEN_H to fit the protocal 2.0 packet 
        # After LENGTH PART is the INSTRUCTION PART, here is "DYN_INST_SYNC_READ"
        # ADDRESS PART is "reg_addr & 0xFF , reg_addr >> 8 & 0xFF" , this convert reg_addr to ADDR_L and ADDR_H to fit the protocal 2.0 packet 
        data = [0xFF, 0xFF, 0xFD, 0x00, DYN_BROADCAST_ID, length & 0xFF, length >> 8 & 0xFF,
        DYN_INST_SYNC_READ, reg_addr & 0xFF , reg_addr >> 8 & 0xFF, nbytes & 0xFF , nbytes >> 8 & 0xFF]
        data.extend(dyn_ids)
        # Add CRC16 Bytes to the data
        add_crc(0, data)
        array = bytes(data)
        # Write the data
        self._uart.write(array)
        for i in range(len(dyn_ids)):
            time.sleep(0.0005) # Critical, 0.001 will miss first few Bytes, Delay required to avoid premature opening of RX
            # Open the RX line and read data
            self._dir.value = False
            # 30 is temporal number, need a way to figure out how to handle this smartly
            packet = self._uart.read(11 + nbytes)
            #packet = self._uart.read(nbytes+6)
            self._dir.value = True
            time.sleep(0.001)
            # Check if packet timed out and is empty
            if packet is None:
                raise RuntimeError("Could not find motor at supplied address")
            self.last_error = packet[8]
            print([hex(x) for x in packet]) 
            
    # parameters is a list in the form [ID1, 1st Byte, 2nd Byte...ID2, 1st Byte, 2nd Byte..., ID3, ... ]
    # other motors may has more Bytes to write, for XL320, currently 2 Bytes is enough
    def sync_write(self, reg_addr, nbytes, parameters):
        length = len(parameters) + 3 + 2 + 2# reg_addr takes the extra 2, nbytes takes extra 2  
                   
        # Prepare a sync_write instruction packet as protocal 2.0 needs 
        # HEADER PART is "0xFF, 0xFF, 0xFD, 0x00"
        # ID PART is "DYN_BROADCAST_ID"
        # LENGTH PART is "length & 0xFF and length >> 8 & 0xFF", this convert length to LEN_L and LEN_H to fit the protocal 2.0 packet 
        # After LENGTH PART is the INSTRUCTION PART, here is "DYN_INST_SYNC_WRITE"
        # ADDRESS PART is "reg_addr & 0xFF , reg_addr >> 8 & 0xFF" , this convert reg_addr to ADDR_L and ADDR_H to fit the protocal 2.0 packet 
        data = [0xFF, 0xFF, 0xFD, 0x00, DYN_BROADCAST_ID, length & 0xFF, length >> 8 & 0xFF,
        DYN_INST_SYNC_WRITE, reg_addr & 0xFF , reg_addr >> 8 & 0xFF, nbytes & 0xFF , nbytes >> 8 & 0xFF]
        data.extend(parameters)
        # Add CRC16 Bytes to the data
        add_crc(0, data)
        array = bytes(data)
        # Write the data
        self._uart.write(array)
        self.last_error = DYN_ERR_INVALID
     
    # parameters is list in the form [ID, reg_addr_L, reg_addr_H, nbytes_L, nbytes_H, repeat...]
    # parameters list example : [ID1, reg_addr & 0xFF, reg_addr >> 8 & 0xFF, nbytes & 0xFF, nbytes >> 8 & 0xFF,...]
    def bulk_read(self, parameters):
        length = len(parameters) + 3 # reg_addr and nbytes now in the parameters
        nbytes = parameters[3] & 0xFF | (parameters[4] & 0xFF ) << 8          
        # Prepare a bulk_read instruction packet as protocal 2.0 needs 
        # HEADER PART is "0xFF, 0xFF, 0xFD, 0x00"
        # ID PART is "DYN_BROADCAST_ID"
        # LENGTH PART is "length & 0xFF and length >> 8 & 0xFF", this convert length to LEN_L and LEN_H to fit the protocal 2.0 packet 
        # After LENGTH PART is the INSTRUCTION PART, here is "DYN_INST_BULK_READ"
        data = [0xFF, 0xFF, 0xFD, 0x00, DYN_BROADCAST_ID, length & 0xFF, length >> 8 & 0xFF,
        DYN_INST_BULK_READ]
        data.extend(parameters)
        # Add CRC16 Bytes to the data
        add_crc(0, data)
        array = bytes(data)
        # Write the data
        self._uart.write(array)
        # read will be tricky...I hope this is rights
        time.sleep(0.0005) # Critical, 0.001 will miss first few Bytes, Delay required to avoid premature opening of RX
        for i in range(int(len(parameters) / 5)):
            # Open the RX line and read data
            self._dir.value = False
            # 30 is temporal number, need a way to figure out how to handle this smartly
            packet = self._uart.read(10 + 2 * nbytes)
            # Check if packet timed out and is empty
            if packet is None:
                raise RuntimeError("Could not find motor at supplied address")
            self.last_error = packet[8]
            print([hex(x) for x in packet])
        self._dir.value = True        
        
    # parameters is list in the form [ID, reg_addr_L, reg_addr_H, nbytes_L, nbytes_H, repeat...]
    # parameters list example : [ID1, reg_addr & 0xFF, reg_addr >> 8 & 0xFF, nbytes & 0xFF, nbytes >> 8 & 0xFF,...]
    def bulk_write(self, parameters):
        length = len(parameters) + 3 # reg_addr and nbytes now in the parameters
                   
        # Prepare a bulk_write instruction packet as protocal 2.0 needs 
        # HEADER PART is "0xFF, 0xFF, 0xFD, 0x00"
        # ID PART is "DYN_BROADCAST_ID"
        # LENGTH PART is "length & 0xFF and length >> 8 & 0xFF", this convert length to LEN_L and LEN_H to fit the protocal 2.0 packet 
        # After LENGTH PART is the INSTRUCTION PART, here is "DYN_INST_BULK_WRITE"
        
        data = [0xFF, 0xFF, 0xFD, 0x00, DYN_BROADCAST_ID, length & 0xFF, length >> 8 & 0xFF,
        DYN_INST_BULK_WRITE]
        data.extend(parameters)
        # Add CRC16 Bytes to the data
        add_crc(0, data)
        array = bytes(data)
        # Write the data
        self._uart.write(array)
        
    # -------------
    # API Functions
    # -------------
    # this has been DP_updated
 
    def set_register(self, dyn_id, reg_addr, data):
        params = [data]
        self.write_data(dyn_id, reg_addr, params)

    def set_register_dual(self, dyn_id, reg_addr, data):
        data1 = data & 0xFF
        data2 = data >> 8
        params = [data1,data2]
        self.write_data(dyn_id, reg_addr, params)

    def get_register(self, dyn_id, reg_addr):
        packet = self.read_data(dyn_id, reg_addr, 1)
        return packet[9]

    def get_register_dual(self, dyn_id, reg_addr):
        packet = self.read_data(dyn_id, reg_addr, 2)
        return packet[9] | (packet[10] << 8)

    # Like get_data, but only returns parameters, not the whole packet
    def get_bytes(self, dyn_id, reg_addr, nbytes):
        packet = self.read_data(dyn_id, reg_addr, nbytes)
        return packet[9:(8+nbytes)]

    def set_speed(self, dyn_id, speed):
        self.set_register_dual(dyn_id, DYN_REG_MOVING_SPEED_L, speed)

    def set_position(self, dyn_id, pos):
        self.set_register_dual(dyn_id, DYN_REG_GOAL_POSITION_L, pos)
        
    def get_position(self, dyn_id):
        return self.get_register_dual(dyn_id, DYN_REG_PRESENT_POSITION_L)

    def get_temp(self, dyn_id):
        return self.get_register(dyn_id, DYN_REG_PRESENT_TEMP)

    def get_error(self, dyn_id):
        return self.ping(dyn_id)

    def parse_error(self, error=0xFF):
        if error == 0xFF:
            error = self.last_error

        if error == DYN_ERR_NONE:
            print("No Errors Reported\n")
        elif error & DYN_ERR_RESULT_FAIL:
            print("Failed to process the sent Instruction Packet\n")
        elif error & DYN_ERR_INST_ERROR:
            print("Undefined Instruction has been used OR Action has been used without Reg Write\n")
        elif error & DYN_ERR_CRC_ERROR:
            print("CRC of the sent Packet does not match\n")
        elif error & DYN_ERR_DATA_RANGE:
            print("Data to be written in the corresponding Address is outside the range of the minimum/maximum value\n")
        elif error & DYN_ERR_DATA_LENGTH:
            print("Attempt to write Data that is shorter than the data length of the corresponding Address\n")
        elif error & DYN_ERR_DATA_LIMIT:
            print("Data to be written in the corresponding Address is outside of the Limit value\n")
        elif error & DYN_ERR_ACCESS:
            print("Attempt to write a value in an Address that is Read Only or has not been defined; Attempt to read a value in an Address that is Write Only or has not been defined; Attempt to write a value in the ROM domain while in a state of Torque Enable(ROM Lock)\n")
        elif error & DYN_ERR_ALERT:
            print ("There is some hard ware issue with the Device. Checking the Hardware error status value of the Control Table can indicate the cause of the problem.")
        elif error & DYN_ERR_INVALID:
            print("No errors available at startup, or for reset or broadcast"\
                   " instructions\n")
                   