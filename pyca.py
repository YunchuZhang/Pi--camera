# import the necessary packages
import math
from dynamixel import * 
from collections import deque
import numpy as np
from picamera.array import PiRGBArray
from picamera import PiCamera
import argparse
import imutils
import time
import cv2


#from core import *
import numpy as ny
from kinematic1 import *


trans = np.eye(4)
ttrans_mat = np.eye(4)
# from kinematic import goalpos
# HSV parameters
# Hmin 138 Smin 155 Vmin 125
# Hmax 175 Smax 255 Vmax 255
# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video",
	help="path to the (optional) video file")
ap.add_argument("-b", "--buffer", type=int, default=10,
	help="max buffer size")
args = vars(ap.parse_args())


# define the lower and upper boundaries of the "green"
# ball in the HSV color space, then initialize the
# list of tracked points

dis = 100000
theta1 = [0,0]
savet = (0,0,0,0)
savet1 = (0,0,0,0)
theta = [0,0,0,0]
basepoint = [0,0,0]
settheta = [0,0,0,0]
settheta0 = [0,0,0,0]
focalLength = 319
KNOWN_WIDTH = 38.5
redLower = (138, 155, 125)
redUpper = (175, 255, 255)
pts = deque(maxlen=args["buffer"])
ps = deque(maxlen=3)
savetheta = deque(maxlen=2)
savetheta1 = deque(maxlen=3)
counter = 0
clear = 0
begin = 0
stop = 0
s = -1
(dX, dY, dZ) = (0, 0, 0)
(x0,y0,z0) = (0,0,0)
(xa,ya,za) = (0,0,0)
direction = ""
#PID
setx = 320
P = 1
I = 0
D = 0
lasterrx = 0
iaccux = 0
nowerrx = 0
out = 0

dxl_goal =[512,512,512,512] 
settheta =[512,512,512,512] 

# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

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

# Enable Dynamixel Torque
for ID in range(11,15):
	dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
	if dxl_comm_result != COMM_SUCCESS:
		print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
	elif dxl_error != 0:
		print("%s" % packetHandler.getRxPacketError(dxl_error))
	else:
		print("Dynamixel has been successfully connected")

	dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, ID, ADDR_PRO_P_GAIN, 33)
	if dxl_comm_result != COMM_SUCCESS:
		print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
	elif dxl_error != 0:
		print("%s" % packetHandler.getRxPacketError(dxl_error))
	else:
		print("Dynamixel has been successfully connected")

	dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, ID, ADDR_PRO_GOAL_POSITION, dxl_goal[ID-11])
	if dxl_comm_result != COMM_SUCCESS:
		print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
	elif dxl_error != 0:
		print("%s" % packetHandler.getRxPacketError(dxl_error))

	dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, ID, ADDR_PRO_MOVE_SPEED , 180)
	if dxl_comm_result != COMM_SUCCESS:
		print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
	elif dxl_error != 0:
		print("%s" % packetHandler.getRxPacketError(dxl_error))




while True :
	settheta = [512,512,512,512]
	for ID in range(11,15):
		dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, ID, ADDR_PRO_GOAL_POSITION, settheta[ID-11])
		print(settheta[ID-11])
		if dxl_comm_result != COMM_SUCCESS:
			print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
		elif dxl_error != 0:
			print("%s" % packetHandler.getRxPacketError(dxl_error))
	settheta = [512,512,666,512]
	for ID in range(11,15):
		dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, ID, ADDR_PRO_GOAL_POSITION, settheta[ID-11])
		print(settheta[ID-11])
		if dxl_comm_result != COMM_SUCCESS:
			print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
		elif dxl_error != 0:
			print("%s" % packetHandler.getRxPacketError(dxl_error))

	#print("[ID:%03d] GoalPos:%03d  PresPos:%03d" % (DXL_ID, dxl_goal_position, dxl_present_position))

	


	# show the frame to our screen




# Disable Dynamixel Torque
for ID in range(11,15):
	dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
	if dxl_comm_result != COMM_SUCCESS:
		print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
	elif dxl_error != 0:
		print("%s" % packetHandler.getRxPacketError(dxl_error))

# Close port
portHandler.closePort()