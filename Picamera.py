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
# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 45
rawCapture = PiRGBArray(camera, size=(640, 480))


# Init Goal position
dxl_goal =[512,160,873,339] 


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

# allow the camera or video file to warm up
time.sleep(0.2)
# capture frames from the camera









for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
	# grab the raw NumPy array representing the image, then initialize the timestamp
	# and occupied/unoccupied text

	start = time.time()
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
		if radius > 4:
			# draw the circle and centroid on the frame,
			# then update the list of tracked points
			
			cv2.circle(image, (int(x), int(y)), int(radius),
				(0, 255, 255), 2)
			cv2.circle(image, center, 5, (0, 0, 255), -1)
			
			pts.appendleft(posi)

	# loop over the set of tracked points
	print(pts)
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
			for j in range(-3,i):
				dX += pts[j][0]
				dY += pts[j][1]
				dZ += pts[j][2]
			dZ = int(dZ/4)
			dX = int(dX/4)
			dY = int(dY/4)
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
	if x0 != 0 and y0 != 0 and z0 != 0:
		s = s + 1

	pss = (x0,y0,z0)

	ps.appendleft(pss)
	print(ps)
	for i in range(1, len(ps)):
		# if either of the tracked points are None, ignore
		# them
		if ps[i - 1] is None or ps[i] is None:
			continue

		# check to see if enough points have been accumulated in
		# the buffer

		if  ps[-1] is not None:
			xa = ps[-1][0] - ps[0][0]
			ya = ps[-1][1] - ps[0][1]
			za = ps[-1][2] - ps[0][2]
			print("hsssss")
			print(xa,ya,za)
			if np.abs(xa) == 0 and np.abs(ya) == 0 and np.abs(za) == 0 and begin != 1:
				clear = 0
				
			elif np.abs(xa) < 40 and np.abs(ya) < 40 and np.abs(za) < 50:
				clear = 1
				stop = stop + 1
				
				
			else:
				clear = 0
				
				
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
			theta[ID-11] = (dxl_present_position / 3.413 - 150) * PI / 180 
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
		print(theta)
		theta[1] = theta[1] - (75/180.0)*PI + PI*(178/180.0)
		theta[2] = theta[2] + (30/180.0)*PI - (135/180.0)*PI
		theta[3] = theta[3] + 0.5*PI - (39/180.0)*PI
		position0 = [[position0[0][0]*0.55],[position0[1][0]*0.55],[position0[2][0]*0.55],[1]]
	else :
		position0 = [[0],[0],[0],[1]]
	#position0[0][0] = position0[0][0] - 10
	print(position0)

	trans = np.dot(goalpos1(theta[0],theta[1],theta[2],theta[3]),position0)
	print(theta)
	print(trans)





	basepoint =np.array([trans[0][0],trans[1][0],trans[2][0]])
	settheta[0] = np.arctan2(basepoint[1], basepoint[0])
	settheta[0] = int (settheta[0]*180/PI)



	n = basepoint[0]**2 + basepoint[1]**2
	m = basepoint[2] - 65	
	n = np.sqrt(n)


	a = np.sqrt(m**2 + n**2)
	x = np.sqrt(a**2 + 83**2 - 2*a*83.0*(n/a))

	temp = np.arccos((129**2 + 65**2 - x**2)/(2*129*65))

	t3 = np.arccos((-129**2 + 65**2 + x**2)/(2*65*x))
	t4 = np.arccos((83**2 - a**2 + x**2)/(2*83*x))
	t1 = temp
	t2 = np.arctan2(m,n)
	print(m)
	print(n)
	print(x)

	print(t1)
	print(t2)


	settheta[2] =  180 - int ((t1)*180/PI) -30
	settheta[1] = 90 - 360 + int((t1+t3+t4)*180/PI) - 15
	settheta[3] = 180 - int((t3+t4)*180/PI) - 90


	print(settheta)

	if settheta[2] > 103:
		settheta[2] = 93
		# s2 = 180 - 42 - 63
		# s1 = 90 - settheta[1] - 20 - t2
		settheta[1] = settheta[1] + 20
		# x1 = np.sqrt(a**2 + 129**2 - 2*a*129*np.cos(s1))
		# x2 = np.arccos((a**2 + x1**2 - 129**2)/(2*a*x1))
		# x3 = np.arccos((x1**2 + 83**2 - 65**2)/(2*83*x1))
		for i in range(-140,5,2):
			theta1[0] = settheta[1] - (75/180.0)*PI + PI*(178/180.0)
			theta1[1] = settheta[2] + (30/180.0)*PI - (135/180.0)*PI
			#theta[3] = settheta[3] + 0.5*PI - (61/180.0)*PI
			#print(i)
			tran2 = np.dot(goalpos1(0,theta1[0],theta1[1],i),position0)
			if distance(np.array(tran2),np.array(trans))<dis:

				print(i)
				settheta[3] = i
				dis = distance(np.array(tran2),np.array(trans))
				print(dis)



	print(settheta)

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



	for i in range(0,4):
		settheta[i] = int (settheta[i] * 3.413 + 512)
		if settheta[i] >= 904:
			settheta[i] = 904
		if settheta[i] <= 100:
			settheta[i] = 100
	if settheta[3] >= 520:
		settheta[3] = 512
	if settheta[1] >= 800:
		settheta[1] = 800
	print("askaskjdasd")
	print(settheta)

	savet = (settheta[0],settheta[1],settheta[2],settheta[3])
	savetheta.appendleft(savet)
	print(s,stop)
	print(savetheta)	
	if clear == 1 and savetheta[-1] is not None:
		settheta[0] = savetheta[-1][0] 
		settheta[1] = savetheta[-1][1] 
		settheta[2] = savetheta[-1][2] 
		settheta[3] = savetheta[-1][3]
	savet = (settheta[0],settheta[1],settheta[2],settheta[3])
	savetheta.appendleft(savet)
	print(savetheta)
	#print(savetheta[-1][0] ,savetheta[-1][1] ,savetheta[0][0] ,savetheta[0][1] ,savetheta[-2][0] ,savetheta[-2][1] )
	# if clear == 1 and savetheta[-1] is not None:
	# 	settheta0[0] = savetheta[-1][0] 
	# 	settheta0[1] = savetheta[-1][1] 
	# 	settheta0[2] = savetheta[-1][2] 
	# 	settheta0[3] = savetheta[-1][3]
		
	
	# 	savet1 = (settheta0[0],settheta0[1],settheta0[2],settheta0[3])
	# 	savetheta1.appendleft(savet1)

	# if s == stop:
	# 	settheta[0] = savetheta1[1][0] 
	# 	settheta[1] = savetheta1[1][1] 
	# 	settheta[2] = savetheta1[1][2] 
	# 	settheta[3] = savetheta1[1][3]
	# elif s != stop:
	# 	s = 0
	# 	stop = 0
		



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
	key = cv2.waitKey(10) & 0xFF
	counter+=1
        # clear the stream in preparation for the next frame
	rawCapture.truncate(0)
	elapsed = (time.time() -start)
	print('time',elapsed)
	
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
