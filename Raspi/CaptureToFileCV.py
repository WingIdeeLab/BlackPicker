import time
import picamera
from picamera import PiCamera
import cv2

# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
imagesize = 2016

camera.resolution = (imagesize, imagesize)
camera.framerate = 30
# Wait for the automatic gain control to settle
time.sleep(1)

cv2.namedWindow("PiCam")

index=0

    
while True:    
    
    key = cv2.waitKey(1) & 0xFF
    
    if (key == ord('q')):
        break

    if (key == ord('n')):
        index = index + 1
        filename = 'CalibImages/CheckerH'+str(index)+'.jpg'
        print('Grab'+str(index)+'.jpg');
        camera.capture(filename)
        #img = cv2.imread(filename)
        #cv2.imshow(filename,img)
        
        
        
