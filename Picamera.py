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
from kinematic import *
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
theta = [0,0,0,0]
basepoint = [0,0,0]
settheta = [0,0,0,0]
focalLength = 319
KNOWN_WIDTH = 38.5
redLower = (138, 155, 125)
redUpper = (175, 255, 255)
pts = deque(maxlen=args["buffer"])
counter = 0
clear = 0
begin = 0
(dX, dY, dZ) = (0, 0, 0)
(x0,y0,z0) = (0,0,0)
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
# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 45
rawCapture = PiRGBArray(camera, size=(640, 480))


# Init Goal position
dxl_goal =512 


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

	dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, ID, ADDR_PRO_GOAL_POSITION, dxl_goal)
	if dxl_comm_result != COMM_SUCCESS:
		print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
	elif dxl_error != 0:
		print("%s" % packetHandler.getRxPacketError(dxl_error))

# allow the camera or video file to warm up
time.sleep(0.2)
# capture frames from the camera
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
	# grab the raw NumPy array representing the image, then initialize the timestamp
	# and occupied/unoccupied text
	image = frame.array


	# resize the frame, blur it, and convert it to the HSV
	# color space
	
	blurred = cv2.GaussianBlur(image, (11, 11), 0)
	hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
 
	# construct a mask for the color "green", then perform
	# a series of dilations and erosions to remove any small
	# blobs left in the mask
	mask = cv2.inRange(hsv, redLower, redUpper)
	mask = cv2.erode(mask, None, iterations=2)
	mask = cv2.dilate(mask, None, iterations=2)
        # find contours in the mask and initialize the current
	# (x, y) center of the ball
	cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
		cv2.CHAIN_APPROX_SIMPLE)
	cnts = cnts[0] if imutils.is_cv2() else cnts[1]
	center = None
 
	# only proceed if at least one contour was found
	if len(cnts) > 0:
		# find the largest contour in the mask, then use
		# it to compute the minimum enclosing circle and
		# centroid
		c = max(cnts, key=cv2.contourArea)
		((x, y), radius) = cv2.minEnclosingCircle(c)
		M = cv2.moments(c)
		z = (KNOWN_WIDTH * focalLength) / (2 * radius)
		center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
		posi = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]), int(z))
 		# only proceed if the radius meets a minimum size
		if radius > 5:
			# draw the circle and centroid on the frame,
			# then update the list of tracked points
			
			cv2.circle(image, (int(x), int(y)), int(radius),
				(0, 255, 255), 2)
			cv2.circle(image, center, 5, (0, 0, 255), -1)
			
			pts.appendleft(posi)

	# loop over the set of tracked points
	for i in range(1, len(pts)):
		# if either of the tracked points are None, ignore
		# them
		if pts[i - 1] is None or pts[i] is None:
			continue

		# check to see if enough points have been accumulated in
		# the buffer
		if counter >= 3 and i == 1 and pts[-3] is not None:
			# compute the difference between the x and y
			# coordinates and re-initialize the direction
			# text variables
			for j in range(-3,i+1):
				dX += pts[j][0]
				dY += pts[j][1]
				dZ += pts[j][2]
			dZ = int(dZ/5)
			dX = int(dX/5)
			dY = int(dY/5)
			print("Dx: %3f  Dy: %3f Dz: %3f"%(dX,dY,dZ))
			
			dX = int((dZ *(dX-320))/focalLength)
			dY = int((dZ *(dY-240))/focalLength)
			begin = 1
			

		# z = -x   y = y x = z
	print("Dxt: %3f  Dyt: %3f Dzt: %3f"%(dX,dY,dZ))
		#thickness = int(np.sqrt(args["buffer"] / float(i + 1)) * 2.5)
		#cv2.line(image, pts[i - 1], pts[i], (0, 0, 255), thickness)
	z0 = -dX
	y0 = dY
	x0 = dZ
	print("Dx0: %3f  Dy0: %3f Dz0: %3f"%(x0,y0,z0))
	# show the movement deltas and the direction of movement on
	# the frame
	cv2.putText(image, direction, (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
		0.65, (0, 0, 255), 3)
	cv2.putText(image, "dx: {}, dy: {}, dz:{}".format(dX, dY, dZ),
		(10, image.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX,
		0.35, (0, 0, 255), 1)

	#Motor
	# read angle make trasform
	for ID in range(11,15):
		dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, ID, ADDR_PRO_PRESENT_POSITION)


		if begin == 1 and dxl_present_position != 0 :
			theta[ID-11] = (dxl_present_position / 5.688 - 90) * PI / 180 
		else :
			theta[ID-11] = 0
		print(dxl_present_position)
		if dxl_comm_result != COMM_SUCCESS:
			print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
		elif dxl_error != 0:
			print("%s" % packetHandler.getRxPacketError(dxl_error))

	# write angle
	if begin == 1 and (theta[0]+theta[1]+theta[2]+theta[3]) != 0 :
		position0 = [[x0],[y0],[z0],[1]]
	else :
		position0 = [[200],[0],[0],[1]]
	position0[0][0] = position0[0][0] - 200
	print(position0)
	trans = np.dot(goalpos(theta[0],theta[1],theta[2],theta[3]),position0)
	print(trans)

	# PID
	# nowerrx = dX - setx
	# iaccux += nowerrx
	# out = P * nowerrx + I * iaccux + D * (nowerrx - lasterrx)
	# lasterrx = nowerrx

	# if dX !=0 and dY !=0:
	# 	# if abs(dX -setx) < 3:
	# 	# 	lock = out
			
	# 	# 	dxl_goal_position = lock
	# 	# 	clear = 0
			
			

	# 	# elif abs(dX -setx) > 3 and clear == 0:

	# 	# 	Perr = P * (dX - setx)
	# 	# 	#Read
	# 	# 	out = int (Perr * 0.9656 + 515)

	# 	dxl_goal_position = int (out) 
			
	# 		#print(" Goal: %d" % (dxl_goal_position))
	# 	if dxl_goal_position > 800:
	# 		dxl_goal_position = 800
	# 	elif dxl_goal_position < 230:
	# 		dxl_goal_position = 230
			
	# else :
	# 	dxl_goal_position = 515 

	# if abs(dxl_goal_position - dxl_present_position) > 3:
	#Image feedback	
	# Write
		#x 0~640 y 0~480
		#206 515 824
		# -320 ~320 
		#P = P * (dX * 0.9656 + 206 - dxl_present_position)

	#0 ~512 ~1024
	#-90 ~0 ~ 90
	# IK
	basepoint =[trans[0][0],trans[1][0],trans[2][0]]
	settheta[0] = np.arctan2(basepoint[1], basepoint[0])
	settheta[0] = int (settheta[0]*180/PI)

	n = basepoint[0]**2 + basepoint[1]**2
	m = basepoint[2] - 65	
	n = np.sqrt(n)
	a = np.sqrt(m**2 + n**2)

	x = np.sqrt(65**2 + 83**2 -(2*65*83*np.cos((135/180.0)*PI)))
	temp = np.arccos((129**2-106**2+a**2)/(2*129*a))
	if math.isnan(temp):
		print("aaaaa")

		position0[0][0] = position0[0][0] - 20
		position0[1][0] = position0[1][0] + 20
		position0[2][0] = position0[2][0] - 20
		trans = np.dot(goalpos(theta[0],theta[1],theta[2],theta[3]),position0)
		print(trans)
		basepoint =[trans[0][0],trans[1][0],trans[2][0]]
		settheta[0] = np.arctan2(basepoint[1], basepoint[0])
		settheta[0] = int (settheta[0]*180/PI)

		n = basepoint[0]**2 + basepoint[1]**2
		m = basepoint[2] - 65	
		n = np.sqrt(n)
		a = np.sqrt(m**2 + n**2)

		x = np.sqrt(65**2 + 83**2 -(2*65*83*np.cos((135/180.0)*PI)))
		t1 = np.arccos((129**2-106**2+a**2)/(2*129*a))
		if math.isnan(t1):
			print("bbbbb")
			position0[0][0] = position0[0][0] - 20
			position0[1][0] = position0[1][0] + 20
			position0[2][0] = position0[2][0] - 20
			trans = np.dot(goalpos(theta[0],theta[1],theta[2],theta[3]),position0)
			print(trans)

			basepoint =[trans[0][0],trans[1][0],trans[2][0]]
			settheta[0] = np.arctan2(basepoint[1], basepoint[0])
			settheta[0] = int (settheta[0]*180/PI)

			n = basepoint[0]**2 + basepoint[1]**2
			m = basepoint[2] - 65	
			n = np.sqrt(n)
			a = np.sqrt(m**2 + n**2)

			x = np.sqrt(65**2 + 83**2 -(2*65*83*np.cos((135/180.0)*PI)))
			t1 = np.arccos((129**2-106**2+a**2)/(2*129*a))
			if math.isnan(t1):
				print("ccccc")
				position0[0][0] = position0[0][0] - 20
				position0[1][0] = position0[1][0] + 20
				position0[2][0] = position0[2][0] - 20
				trans = np.dot(goalpos(theta[0],theta[1],theta[2],theta[3]),position0)
				print(trans)

				basepoint =[trans[0][0],trans[1][0],trans[2][0]]
				settheta[0] = np.arctan2(basepoint[1], basepoint[0])
				settheta[0] = int (settheta[0]*180/PI)

				n = basepoint[0]**2 + basepoint[1]**2
				m = basepoint[2] - 65	
				n = np.sqrt(n)
				a = np.sqrt(m**2 + n**2)

				x = np.sqrt(65**2 + 83**2 -(2*65*83*np.cos((135/180.0)*PI)))
				t1 = np.arccos((129**2-106**2+a**2)/(2*129*a))
			else:
				pass
		else:
			pass
	else:
		t1 = temp
	t2 = np.arctan2(m,n)
	print(m)
	print(n)
	print(x)

	print(t1)
	print(t2)


	settheta[1] = 90 - int ((t2 + t1)*180/PI) - 38


	belta = np.arccos((129**2 - a**2 + x**2)/(2*129*x))
	fi = np.arccos((65**2 - 83**2 + x**2)/(2*65*x))
	print(belta)
	print(fi)
	settheta[2] = 180 - int ((belta + fi)*180/PI) -47
	settheta[3] = -settheta[1]-settheta[2]-35
	print(settheta)


	for i in range(0,4):
		settheta[i] = int (settheta[i] * 5.688 + 512)
		if settheta[i] >= 924:
			settheta[i] = 924
		if settheta[i] <= 100:
			settheta[i] = 100
	if settheta[3] >= 520:
		settheta[3] = 512
	for ID in range(11,15):
		dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, ID, ADDR_PRO_GOAL_POSITION, settheta[ID-11])
		print(settheta[ID-11])
		if dxl_comm_result != COMM_SUCCESS:
			print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
		elif dxl_error != 0:
			print("%s" % packetHandler.getRxPacketError(dxl_error))


	#print("[ID:%03d] GoalPos:%03d  PresPos:%03d" % (DXL_ID, dxl_goal_position, dxl_present_position))

	


	# show the frame to our screen
	cv2.imshow("Frame", image)
	key = cv2.waitKey(1) & 0xFF
	counter+=1
        # clear the stream in preparation for the next frame
	rawCapture.truncate(0)
	
	# if the 'q' key is pressed, stop the loop
	if key == ord("q"):
		break
 
# close all windows
cv2.destroyAllWindows()


# Disable Dynamixel Torque
for ID in range(11,15):
	dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
	if dxl_comm_result != COMM_SUCCESS:
		print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
	elif dxl_error != 0:
		print("%s" % packetHandler.getRxPacketError(dxl_error))

# Close port
portHandler.closePort()
