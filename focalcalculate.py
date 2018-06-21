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
KNOWN_DISTANCE = 24.0
KNOWN_WIDTH = 11.0
def distance_to_camera(knownWidth, focalLength, perWidth):
	# compute and return the distance from the maker to the camera
	return (knownWidth * focalLength) / perWidth

redLower = (138, 155, 125)
redUpper = (175, 255, 255)
# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 45
rawCapture = PiRGBArray(camera, size=(640, 480))
image = cv2.imread('')
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
	center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
 
	# only proceed if the radius meets a minimum size
	if radius > 5:
		# draw the circle and centroid on the frame,
		# then update the list of tracked points
			
		cv2.circle(image, (int(x), int(y)）, int(radius),(0, 255, 255), 2)
		cv2.circle(image, center, 5, (0, 0, 255), -1)
		focalLength = (2 * radius * KNOWN_DISTANCE) / KNOWN_WIDTH
		print(" The radius is %f focalLength is %f" %(radius,focalLength))
		cv2.putText(image, "focalLength: {}".format(focalLength),
		(10, image.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX,
		0.35, (0, 0, 255), 1)



cv2.destroyAllWindows()