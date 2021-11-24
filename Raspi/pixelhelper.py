import cv2
import numpy as np
import math

def Intensity(image,x,y,r,i):
    
    if (x>0 and y>0 and r>0):
        h, w = image.shape
        cropboundarysize = r
        outercirclemargin = -11
        innercircleradius = 14
           
        # make black maskimage
        circle_img = np.zeros((h,w), np.uint8)
        
        #draw circle white
        cv2.circle(circle_img,(x,y),r+outercirclemargin,(255,255,255),-1)
        #draw circle black
        cv2.circle(circle_img,(x,y),innercircleradius,(0,0,0),-1)

        masked_data = cv2.bitwise_and(image, image, mask=circle_img)
        cv2.imshow("masked", masked_data)

        #crop image to save memory and speed
        
        crop = masked_data[y-cropboundarysize:y+cropboundarysize, x-cropboundarysize:x+cropboundarysize]
        h, w = crop.shape
        cv2.imshow(str(i), crop)
        
        #colorcrop = cv2.cvtColor(crop, cv2.COLOR_GRAY2BGR)
        #cv2.imshow(str(i)+"-2", colorcrop)
        
        
        brightness =0
        
        for i in range(h):
          for j in range(w):
             k = crop[i,j]
             brightness = brightness + k
        
        return brightness
    
    else: return 0
    
    
def findBoxOrientation(box):
        
        #unpacking
        p1, p2, p3, p4 = box

        vector1 = p2-p1
        vector2 = p3-p2
        if (cv2.norm(vector1) > cv2.norm(vector2)):
                longestvector = vector1
        else:
                longestvector = vector2
        
        x1, y1 = longestvector
        x2 = 0
        y2 = 1

        angle = int(math.atan((y1-y2)/(x2-x1))*180/math.pi)
        
        
        
        return angle, longestvector
        
        
def ResizeImage(img, scale_percent):
        width = int(img.shape[1] * scale_percent / 100)
        height = int(img.shape[0] * scale_percent / 100)
        dim = (width, height)
  
        # resize image
        resized = cv2.resize(img, dim, interpolation = cv2.INTER_AREA)
         
        return resized
        
        
def Remap( x, oMin, oMax, nMin, nMax ):

    #range check
    if oMin == oMax:
        print ("Warning: Zero input range")
        return None

    if nMin == nMax:
        print ("Warning: Zero output range")
        return None

    #check reversed input range
    reverseInput = False
    oldMin = min( oMin, oMax )
    oldMax = max( oMin, oMax )
    if not oldMin == oMin:
        reverseInput = True

    #check reversed output range
    reverseOutput = False   
    newMin = min( nMin, nMax )
    newMax = max( nMin, nMax )
    if not newMin == nMin :
        reverseOutput = True

    portion = (x-oldMin)*(newMax-newMin)/(oldMax-oldMin)
    if reverseInput:
        portion = (oldMax-x)*(newMax-newMin)/(oldMax-oldMin)

    result = portion + newMin
    if reverseOutput:
        result = newMax - portion

    return result

