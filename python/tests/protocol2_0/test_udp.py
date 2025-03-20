#!/usr/bin/env python
# -*- coding: utf-8 -*-

#*******************************************************************************
# Copyright 2017 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#*******************************************************************************


#*******************************************************************************
#***********************     Broadcast Ping Example      ***********************
#  Required Environment to run this example :
#    - Protocol 2.0 supported DYNAMIXEL(X, P, PRO/PRO(A), MX 2.0 series)
#    - DYNAMIXEL Starter Set (U2D2, U2D2 PHB, 12V SMPS)
#  How to use the example :
#    - Build and Run from proper architecture subdirectory.
#    - For ARM based SBCs such as Raspberry Pi, use linux_sbc subdirectory to build and run.
#    - https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/
#  Author: Ryu Woon Jung (Leon)
#  Maintainer : Zerom, Will Son
# *******************************************************************************

import os
import sys
import threading
import random
import time

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch
    

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src')))
from dynamixel_sdk_udp import *                 # Uses Dynamixel SDK library

# DYNAMIXEL Protocol Version (1.0 / 2.0)
# https://emanual.robotis.com/docs/en/dxl/protocol2/
PROTOCOL_VERSION            = 2.0         

# Define the proper baudrate to search DYNAMIXELs. Note that XL320's baudrate is 1 M bps.
BAUDRATE                = 57600  

# Use UDP comm port.
# ex) "udp://192.168.1.100:1234"
DEVICENAME                  = 'udp://192.168.42.1:6464'    

# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandlerUDP(DEVICENAME)

portHandler.setHost("192.168.42.2", 0) # Bind to any port on the correct network interface

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()


# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()

print("Process ID:", os.getpid())
print("Thread ID:", threading.get_ident())


# # Try to broadcast ping the Dynamixel
# print("Try to broadcast ping the Dynamixel...")

# Define the control table address for Goal Position and Present Position
ADDR_GOAL_POSITION = 116
ADDR_TORQUE_ENABLE = 64
ADDR_HARDWARE_ERROR = 70
ADDR_PRES_VELOCITY = 128
ADDR_PRES_POSITION = 132
ADDR_SHUTDOWN = 63
ADDR_LED = 65

# Define the goal position (0 ~ 1023 for 10-bit resolution)
GOAL_POSITION = 2048

# Define the Dynamixel ID to control (replace with your Dynamixel ID)
DXL_ID = 1

def dxl_shutdown_state(dxl_data):
    # Check for specific error flags in the shutdown register
    if dxl_data & 0x01:  # Bit 0: Input voltage error
        print("Input voltage error shutdown enabled.")
    if dxl_data & 0x04:  # Bit 2: Overheating
        print("Overheating error shutdown enabled.")
    if dxl_data & 0x08:  # Bit 3: Motor encoder error
        print("Motor encoder error shutdown enabled.")
    if dxl_data & 0x10:  # Bit 4: Electrical shock error
        print("Electrical shock error shutdown enabled.")
    if dxl_data & 0x20:  # Bit 5: Overload error
        print("Overload error shutdown enabled.")

def dxl_make_shutdown_flag(enable_overload, enable_shock, enable_encoder_error, enable_overheat, enable_voltage):
    '''Make a shutdown flag'''
    shutdown_flag = 0
    if enable_voltage:
        shutdown_flag |= 0x01
    if enable_overheat:
        shutdown_flag |= 0x04
    if enable_encoder_error:
        shutdown_flag |= 0x08
    if enable_shock:
        shutdown_flag |= 0x10
    if enable_overload:
        shutdown_flag |= 0x20
    return shutdown_flag

def dxl_hw_error_state(dxl_data):
    # Check for specific error flags in the hardware error status register
    if dxl_data & 0x01:  # Bit 0: Input voltage error
        print("Input voltage error detected.")
    if dxl_data & 0x04:  # Bit 2: Overheating
        print("Overheating error detected.")
    if dxl_data & 0x08:  # Bit 3: Motor encoder error
        print("Motor encoder error detected.")
    if dxl_data & 0x10:  # Bit 4: Electrical shock error
        print("Electrical shock error detected.")
    if dxl_data & 0x20:  # Bit 5: Overload error
        print("Overload error detected.")

def dxl_read_shutdown(dxl_id):
    '''Read Shutdown RAM register'''
    print(f"[ID:{dxl_id}] Reading shutdown status register...")
    dxl_data, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, dxl_id, ADDR_SHUTDOWN)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        pass
    dxl_shutdown_state(dxl_data)
    return dxl_data

def dxl_read_torque(dxl_id):
    '''Read Torque Enable RAM register'''
    print(f"[ID:{dxl_id}] Reading torque status register...")
    dxl_data, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        pass
    print("Torque status: %d" % dxl_data)

def dxl_write_torque(dxl_id, value):
    '''Write Torque Enable RAM register'''
    print(f"[ID:{dxl_id}] Writing torque={value}...")
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, value)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Torque set to %d" % value)

def dxl_write_shutdown(dxl_id, value):
    '''Write Shutdown RAM register'''
    print(f"[ID:{dxl_id}] Writing shutdown={value}...")
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_SHUTDOWN, value)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Shutdown set to %d" % value)

def dxl_read_led(dxl_id):
    '''Read LED RAM register'''
    print(f"[ID:{dxl_id}] Reading LED status register...")
    dxl_data, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, dxl_id, ADDR_LED)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        pass
    print("LED status: %d" % dxl_data)

def dxl_write_led(dxl_id, value):
    '''Write LED RAM register'''
    print(f"[ID:{dxl_id}] Writing LED={value}...")
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_LED, value)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("LED set to %d" % value)


def dxl_bcast_ping():
    print(f"[ID:{dxl_id}] Broadcast pinging Dynamixel...")
    dxl_data_list, dxl_comm_result = packetHandler.broadcastPing(portHandler)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

    for dxl_id in dxl_data_list:
        print("[ID:%03d] model version : %d | firmware version : %d" % (dxl_id, dxl_data_list.get(dxl_id)[0], dxl_data_list.get(dxl_id)[1]))

def dxl_ping(dxl_id):
    '''Ping the Dynamixel'''
    print(f"[ID:{dxl_id}] Pinging Dynamixel...")
    dxl_model_number, dxl_comm_result, dxl_error = packetHandler.ping(portHandler, dxl_id)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    
    print(f"[ID:{dxl_id}] model version : {dxl_model_number}")

def dxl_reboot(dxl_id):
    '''Reboot the Dynamixel'''
    print(f"[ID:{dxl_id}] Rebooting Dynamixel...")
    dxl_comm_result, dxl_error = packetHandler.reboot(portHandler, dxl_id)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Dynamixel has been successfully rebooted")

def dxl_read_hardware_error(dxl_id):
    '''Read Hardware Error RAM register'''
    print(f"[ID:{dxl_id}] Reading hardware error...")
    dxl_data, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, dxl_id, ADDR_HARDWARE_ERROR)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        pass
    dxl_hw_error_state(dxl_data)

def dxl_read_present_position(dxl_id):
    '''Read Present Position RAM register'''
    print(f"[ID:{dxl_id}] Reading present position...")
    data, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, dxl_id, ADDR_PRES_POSITION)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        pass
    print(f"[ID:{dxl_id}] Present position: %d" % data)
    return data

def dxl_read_present_velo(dxl_id):
    '''Read Present Velocity RAM register'''
    print(f"[ID:{dxl_id}] Reading present velocity...")
    data, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, dxl_id, ADDR_PRES_VELOCITY)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        pass
    print("Present velocity: %d" % data)

# Write goal position to the Dynamixel
def dxl_write_goal_position(dxl_id, pos):
    '''Write Goal Position RAM register'''
    print(f"[ID:{dxl_id}] Writing goal position={pos}...")
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, dxl_id, ADDR_GOAL_POSITION, pos)
    if dxl_comm_result != COMM_SUCCESS:
        if dxl_comm_result == COMM_RX_TIMEOUT:
            print("Failed to receive status packet.")
            return False
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Goal position set to %d" % GOAL_POSITION)
    return True


# BEGIN EXECUTION
############################################

# Establish connection & enable torque
dxl_write_torque(DXL_ID, 1)
time.sleep(2)

# Starting at position 0, rotate the Dynamixel in 512-step increments indefinitely
pos = 0
while True:  # Adjust the range for the desired number of iterations
    print(f"Moving to position {pos}...")
    while not dxl_write_goal_position(DXL_ID, pos): time.sleep(0.1)
    time.sleep(2)  # Add a delay to allow the Dynamixel to move to the position
    pos = (pos + 512) % 4096

# Close up shop
dxl_write_torque(DXL_ID, 0)


############################################
# END EXECUTION

# Close port
portHandler.closePort()
