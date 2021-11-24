# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np


# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
imagesizey = 768
imagesizex = 1024

camera.resolution = (imagesizex, imagesizey)
camera.framerate = 30
# Wait for the automatic gain control to settle
time.sleep(1)
# Now fix the values
camera.shutter_speed = camera.exposure_speed
camera.exposure_mode = 'off'
g = camera.awb_gains
camera.awb_mode = 'off'
camera.awb_gains = g
    
    

rawCapture = PiRGBArray(camera, size=(imagesizex, imagesizey))


def nothing(x):
  pass

cv2.namedWindow("PiCam")
hh='Max'
hl='Min'
wnd = 'Colorbars'

 
# allow the camera to warmup
time.sleep(1.0)
 
# capture frames from the camera
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        # grab the raw NumPy array representing the image, then initialize t$
        # and occupied/unoccupied text
        

        #shifted = cv2.pyrMeanShiftFiltering(cutimage, 21, 51)
        cv2.imshow("frame", frame.array)
     
        key = cv2.waitKey(1) & 0xFF
 
        # clear the stream in preparation for the next frame
        rawCapture.truncate(0)
 
        # if the `q` key was pressed, break from the loop
        if key == ord("q"):
                break
