
# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
from time import sleep
# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (640, 480)


# allow the camera to warmup
time.sleep(0.1)

# grab image from the camera
camera.start_preview()
for i in range(3):
    sleep(5)
    camera.capture('/home/pi/Track/image%s.jpg' % i)
camera.stop_preview()
