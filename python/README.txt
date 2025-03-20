# Dynamixel SDK w/ UDP Python SDK 

This fork of the Dynamixel SDK contains custom code developed by Diffraction Limited to enable communication with Dynamixel devices via UDP.

Example usage:

```Python
from dynamixel_sdk_udp import *                 # Uses Dynamixel SDK library

# DYNAMIXEL Protocol Version (1.0 / 2.0)
# https://emanual.robotis.com/docs/en/dxl/protocol2/
PROTOCOL_VERSION            = 2.0         

# Use UDP com port.
# ex) "udp://192.168.1.100:1234"
DEVICENAME                  = 'udp://127.0.0.1:6464'    

# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandlerUDP(DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)

if not portHandler.openPort():
    print("Failed to open UDP port")
    exit(1)

# Example: broadcast to all devices seen on this PortHandler
dxl_data_list, dxl_comm_result = packetHandler.broadcastPing(portHandler)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))


portHandler.closePort()

```

# Information about Dynamixel SDK 

Description is available at https://github.com/ROBOTIS-GIT/DynamixelSDK