# Black Picker Tracking Programm
# Ramon Hofer Kraner
# IDEE OST Fachhochschule

# Dieses File wird im Autostart automatisch beim Aufstarten gestartet:
#/etc/xdg/lxsession/LXDE-pi
# to edit: sudo nano /home/pi/.config/lxsession/LXDE-pi/autostart

# import the necessary packages
import transform
import pixelhelper
from picamera.array import PiRGBArray
from picamera import PiCamera
from matplotlib import pyplot as plt
import time
import cv2
import numpy as np
import math 
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import serial
import pickle
import RPi.GPIO as GPIO

from random import seed
from random import randint

# Ausgang um Teensy zu zeigen, ob Raspi bereits ready
OK_PIN = 3
# Da nicht alls GPIOS beim Startup stabil sind, kann man nur bestimte nehmen
# https://www.raspberrypi.org/forums/viewtopic.php?t=162242
#https://www.raspberrypi-spy.co.uk/2012/06/simple-guide-to-the-rpi-gpio-header-and-pins/


#port = '/dev/ttyACM0' # USB  Serial
port = '/dev/ttyS0' # GPIO Serial (UART 0 an Pins 14/15 bzw. 8/10:  https://i.stack.imgur.com/Og4DD.png)

#BCM board pin numbering
GPIO.setmode(GPIO.BCM)
GPIO.setup(OK_PIN, GPIO.OUT)

#Set OK Pin to ON state --> Teensy knows programm is running!
GPIO.output(OK_PIN, GPIO.LOW)
# Damit nicht immer Warinngs kommen
GPIO.setwarnings(False)

#Kommunikationsdaten: Standardformat: <Befehl, Paramter1, Parameter2, Parameter3>
startMarker = '<' 
endMarker = '>'
dataStarted = False
dataBuf = ""
messageComplete = False


# The main parts saving structure
# Part NUmber, x (pixels), y (pixels), standing? (0 or 1), orientation (degrees)
# Für rechtecke(Shapes)
TrackedPartsRect = np.array([[0,0,0,0,0,]])
# Für Kreise(Circles)
TrackedPartsCircle = np.array([[0,0,0,0,0,]])

#calibration data: (muss manuell bestimmt werden)
# Eckpunkte des Maschinenquadrats (in Bildkoordinaten)
# hierzu die Ecken Markieren z.B. an den Stellen x=500, y=600, x= 12500, y=12600

#Bildkoordinaten
KP1 = (72,57) 
KP2 = (1876,125)
KP3 = (1871,1921)
KP4 = (23,1926)

#Maschinenkoordinaten
KP1M = (500,600)
KP2M = (12500, 600)
KP3M = (12500, 12600)
KP4M = (500, 12600)


# Hier werden die Tracking Parameter für jedes Teil eingestellt. Jede Zeile entspricht einem Teil. Es werden nur max. 4 Teile unterstützt.
# lowerThreshold, canny, houghdpEdges, houghmindist, houghcanny, houghCircleness, houghminradius, houghmaxradius, minareasize
TrackingParameterProTeil = np.array([
[230,245,18,32,239,27,32,36,600],
[200,200,18,32,239,27,32,36,600],
[150,150,18,32,239,27,32,36,600],
[100,100,18,32,239,27,32,36,600],
])


#Auflösung des kamerabildes in Pixel
imagesize = 2016
#Grösse des Abdeckkreises
blackcirclemargin = 16
# Farbeinstellungen
colorLying = (0,0, 255)
textcolor = (0,0, 0)
colorStanding = (0,255, 255)
colorContours = (255,0, 0)
#Fenstername
ParaWindowName = "BlackPicker Parameter"

# Zählervariablen und flags
aktuelleTeilenummer = 0;
NOFCircles = 0
NOFRects = 0
nof_failsC = 0
nof_failsR = 0
istoolarge = False

# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.framerate = 30
camera.resolution = (imagesize, imagesize)
# Wait for the automatic gain control to settle
time.sleep(1)
# Now fix the values
camera.shutter_speed = camera.exposure_speed
camera.exposure_mode = 'off'
g = camera.awb_gains
camera.awb_mode = 'off'
camera.awb_gains = g

# Hier wird die Kameraverzerrung herausgerechnet. Für dies wird data von einem bereits berechnetes file eingelesen
# read the calibration data from pickle file
# https://automaticaddison.com/how-to-perform-camera-calibration-using-opencv/
calib_result_pickle = pickle.load(open("camera_calib_pickle.p", "rb" ))
matrix = calib_result_pickle["matrix"]
distortion = calib_result_pickle["distortion"]

#zufallszahlengenerator initialisieren (Bei Fails wird ein weiteres Teil direkt gegriffen, per Zufall)
seed(time.time)

# Seriellen Port auf dem Raspi initialisieren
serial = serial.Serial(port, 9600, timeout=1)
serial.flush()
 
# allow the camera to warmup
time.sleep(1.0)




## Funktionen --------------------------------------------------------------------------------------------


# Die Serielle Schnittstelle nach neuen Daten abfragen und bei neuen Daten den Buffer mit diesen füllen
# Format: <Befehl, Paramter1> ist erweiterbar, dann im code auch auslesen
def CheckForSerialCommand():
        
        global startMarker, endMarker, serialPort, dataStarted, dataBuf, messageComplete
        cmd=""
        mes=""
        
        # Falls daten vorhanden und wir noch keine komplette Message (mit Endmarker) haben
        if serial.inWaiting() > 0 and messageComplete == False:
                          
                x = serial.read().decode("utf-8") # decode needed for Python3
                
                if dataStarted == True:
                    if x != endMarker:
                        dataBuf = dataBuf + x
                    else:
                        dataStarted = False
                        messageComplete = True
                        
                elif x == startMarker:
                    dataBuf = ''
                    dataStarted = True
        # Falls die Daten komplett sind udn wir eien kompletten Befehl erhalten haben
        
        if (messageComplete == True):
                messageComplete = False
                print ("Data Received: ", dataBuf)
                
                # Daten parsen mit ,
                split_string = dataBuf.split(',')
                
                print(split_string)
                # print(split_string[0])
                # print(split_string[1])

                # Alle gesplitteten Objekte zurückgeben
                return split_string
        else:
                split_string = ["none",0]
                return split_string
                   
# Hilfsfunktion macht nix
def nothing(x):
    pass
    
# Lineare Interpolation um  Bildkoordinaten in  Maschinenkoordinaten  umzurechnen 
def interpolate(x1: float, x2: float, y1: float, y2: float, x: float):
    """Perform linear interpolation for x between (x1,y1) and (x2,y2) """

    return ((y2 - y1) * x + x2 * y1 - x1 * y2) / (x2 - x1)


# Alle Parameter updaten. Dies ist nötig, damit die Paramter für unterschiedliche Teile auch im Fenster sichtbar werden
def ParameterFensterUpdaten(TN):
        
        print("Nummer: ", TN)
        TN = TN-1 # da auf Teensy bei 1 startend und hier der array mit 0
        cv2.destroyWindow(ParaWindowName)
        cv2.namedWindow(ParaWindowName)
        
        #Scrollbars definieren mit den verschiedenen Paramtern für das Tracking
        cv2.createTrackbar("lowerThreshold", ParaWindowName,    TrackingParameterProTeil[TN,0], 255, nothing)
        cv2.createTrackbar("canny", ParaWindowName,             TrackingParameterProTeil[TN,1], 255, nothing)
        cv2.createTrackbar("houghdpEdges", ParaWindowName,      TrackingParameterProTeil[TN,2], 50, nothing)
        cv2.createTrackbar("houghmindist", ParaWindowName,      TrackingParameterProTeil[TN,3], 50, nothing)
        cv2.createTrackbar("houghcanny", ParaWindowName,        TrackingParameterProTeil[TN,4], 255, nothing)
        cv2.createTrackbar("houghCircleness", ParaWindowName,   TrackingParameterProTeil[TN,5], 255, nothing)
        cv2.createTrackbar("houghminradius", ParaWindowName,    TrackingParameterProTeil[TN,6], 100, nothing)
        cv2.createTrackbar("houghmaxradius", ParaWindowName,    TrackingParameterProTeil[TN,7], 100, nothing)
        cv2.createTrackbar("minareasize", ParaWindowName,       TrackingParameterProTeil[TN,8], 1500, nothing)

        # Menubild einlesen
        image = cv2.imread('BlackPickerMenu.JPG')
        # Text einblenden
        cv2.putText(image, "Teil Sel:", (250, 40), cv2.FONT_HERSHEY_SIMPLEX,0.5, (255,255,255), 1)
        cv2.putText(image, str(TN+1), (330, 40), cv2.FONT_HERSHEY_SIMPLEX,0.5, (255,255,255), 1)
        # Bild anzeigen
        cv2.imshow(ParaWindowName,image )
        cv2.moveWindow(ParaWindowName, 0,0);




## Hauptloop  --------------------------------------------------------------------------------------------
# Fenster öffnen und Parameter updaten
cv2.namedWindow(ParaWindowName)
ParameterFensterUpdaten(aktuelleTeilenummer)

while True:
    
    # Abfragen des keyboards ob eine taste gedrückt wurde
    key = cv2.waitKey(1) & 0xFF

    # Seriellen Port nach Commandos vom Teensy abfragen
    Command = CheckForSerialCommand()

    
    #Nun alles Verarbeiten:
    
# --  Befehl "q": Beenden ------------------------------------------------     
    # Programm beenden
    #Falls q gedrückt
    if (key == ord('q')):
        break
        
        
# --  Befehl "f" oder failC von Teensy: Fail Cicle------------------------------------------------   
# Noch nicht implemtniert, diese könnte für statistik interessant sein  
    if (key == ord('f') or Command[0] == "failC"):
            nof_failsC = nof_failsC + 1
            print (nof_failsC)
            
            
            
# --  Befehl <teil,Nummer> von Teensy: Aktuell ausgewähltes Teil übertragen ------------------------------------------------  
    if (Command[0] == "teil"):
            aktuelleTeilenummer = int(Command[1])
            print ("Selektierte Teilenummer: ", aktuelleTeilenummer)
            ParameterFensterUpdaten(aktuelleTeilenummer)
            
        

# --  Befehl "s" oder <get,xxxx>: Gefundene Koordinaten an Teensy übetragen------------------------------------------------
    # Koordinaten an Maschine senden    
    #Falls s gedrückt oder Befehl von Maschine erhalten
    if (key == ord('s') or Command[0] == "get"):
            
        Command = ["nope",0] # command zurücksetzen für nächsten durchgang
        
        if (not(NOFShapes == 0)):
                print(" Sending coordinates to machine")
                # Koordinaten Maschine Berechnen
                
                # generate a random integer
                randomPartPicker = randint(0, NOFShapes-1)
                print("Random Value: ", randomPartPicker)
                print("NOFShpaes: ", NOFShapes)
                
                # koordinaten mappen (Bild --> Maschine)
                xmapped = interpolate(0, imagesize, KP1M[0], KP3M[0], TrackedPartsAllClean[randomPartPicker,1])
                ymapped = interpolate(0, imagesize, KP1M[1], KP3M[1], TrackedPartsAllClean[randomPartPicker,2])
                angle = int(TrackedPartsAllClean[randomPartPicker,4])

                print("Bild koordinaten: ", TrackedPartsAllClean[randomPartPicker,1],', ', TrackedPartsAllClean[randomPartPicker,2] )
                print("Maschinen koordinaten: ", xmapped,', ', ymapped)
                
                # aufrecht oder liegend? nachschauen in der Liste
                isstanding = TrackedPartsAllClean[randomPartPicker,3]
                
                # Winkel umrechnen
                if (angle < 0):
                        angle += 90
                elif (angle >= 0):
                        angle = -angle
                        
                if (angle < 0):
                        angle = int(pixelhelper.Remap(angle, -90,0,0,90))
                elif (angle >=0):
                        angle = int(pixelhelper.Remap(angle, 90,0,90,180))
                
                
                # Fertiges Paket zusammenstellen <t,x,y,angle,stand>
                coordinates = "<t,"
                coordinates += str(int(xmapped))
                coordinates += ","
                coordinates += str(int(ymapped))
                coordinates += ","
                coordinates += str(angle)
                coordinates += ","
                coordinates += str(isstanding)
                coordinates += ">"
                
                print("Sending Data Packet: ")
                print(coordinates)
                # Daten senden
                serial.write(coordinates.encode('utf-8'))
                
        else: 
                #Falls nix gefunden wurde dies Teensy mitteilen
                print("NO Parts found!!" )
                coordinates="<n,0,0,0>"
                serial.write(coordinates.encode('utf-8'))




# --  Befehl "c": Kalibrierung------------------------------------------------
    # Kalibrierbild schiessen  
    if (key == ord('c')):
            
        rawCapture = PiRGBArray(camera, size=(imagesize, imagesize))
        
        #grab a frame
        camera.capture(rawCapture, format="bgr")

        # grab the raw NumPy array representing the image, then initialize it
        imagegrab_orig = rawCapture.array
        
        #this for calibration
        imgplot = plt.imshow(imagegrab_orig)
        plt.show()
        
        # image undistortion with the file parameters
        h,  w = imagegrab_orig.shape[:2]
        print(w,h)
        newcameramtx, roi=cv2.getOptimalNewCameraMatrix(matrix,distortion,(w,h),0,(w,h))
        # undistort
        imagegrab = cv2.undistort(imagegrab_orig, matrix, distortion, 1, newcameramtx)
        
        # --------- Warping
        # the warp coordinates
        points = np.array([KP1, KP2, KP3, KP4])
        
        warped = transform.four_point_transform(imagegrab, points, imagesize, imagesize)       
        # Bildrotation damit gleich wie Koordinatensystem der Maschine
        warped = cv2.rotate(warped, cv2.ROTATE_90_CLOCKWISE) 
        
        mapping = warped.copy()
        
        # Gitter für Kalibrierung zeichnen
        for i in range(0,imagesize,250):

                # Bildkoordinaten1, Bildkoordinaten2, Maschinenkoodinaten1, Maschinenkoodinaten2, Eingabe zum umrechene)
                xmapped = interpolate(0, imagesize, KP1M[0], KP3M[0], i)
                ymapped = interpolate(0, imagesize, KP1M[1], KP3M[1], i)

                start_point=(0, i)
                end_point = (imagesize, i)

                mapping = cv2.line(mapping, start_point,end_point, (255,255,255), 3)

                cv2.putText(mapping, str(int(xmapped)), (int(i)+50, 300), cv2.FONT_HERSHEY_SIMPLEX,2, (255,255,255), 3)
                cv2.putText(mapping, str(int(ymapped)), (300, int(i)+50), cv2.FONT_HERSHEY_SIMPLEX,2, (255,255,255), 3)
                
                start_point = (i, 0)
                end_point = (i, imagesize)
                mapping = cv2.line(mapping, start_point,end_point, (255,255,0), 3)
        
        mappingresized = pixelhelper.ResizeImage(mapping,20);
        cv2.imshow("mapping", mappingresized)
        
        
        
        

# --  Befehl "n" oder <pic, xxxx> von Teensy erhalten: Teile erfassen------------------------------------------------
    # Neues Bild schiessen und analysieren 
    # Taste n oder Befehl pic ueber serial    
    if (key == ord('n') or Command[0] == "pic"):
        
        #command wieder zurücksetzten für naächsten Durchgang
        Command[0] = "nope"
        
        #reset the array for the next capture
        TrackedPartsCircle = np.array([[0,0,0,0,0,]])
        TrackedPartsRect = np.array([[0,0,0,0,0,]])
    
        rawCapture = PiRGBArray(camera, size=(imagesize, imagesize))
        
        #grab a frame
        camera.capture(rawCapture, format="bgr")

        # grab the raw NumPy array representing the image, then initialize it
        imagegrab_orig = rawCapture.array
        
        # image undistortion with the file parameters
        h,  w = imagegrab_orig.shape[:2]
        print(w,h)
        newcameramtx, roi=cv2.getOptimalNewCameraMatrix(matrix,distortion,(w,h),0,(w,h))
        # undistort
        imagegrab = cv2.undistort(imagegrab_orig, matrix, distortion, 1, newcameramtx)
        
        #imagegrab = imagegrab_orig

        # create sliders for variables
        lower = cv2.getTrackbarPos("lowerThreshold", ParaWindowName)
        canny = cv2.getTrackbarPos("canny", ParaWindowName)
        houghdp = cv2.getTrackbarPos("houghdpEdges", ParaWindowName)
        houghmindist = cv2.getTrackbarPos("houghmindist", ParaWindowName)
        houghcanny = cv2.getTrackbarPos("houghcanny", ParaWindowName)
        houghparam = cv2.getTrackbarPos("houghCircleness", ParaWindowName)
        houghminradius = cv2.getTrackbarPos("houghminradius", ParaWindowName)
        houghmaxradius = cv2.getTrackbarPos("houghmaxradius", ParaWindowName)
        intensityLevel = cv2.getTrackbarPos("intensityLevel", ParaWindowName)
        minareasize = cv2.getTrackbarPos("minareasize", ParaWindowName)
        
        
        
        # --------- Warping
        # the warp coordinates
        points = np.array([KP1, KP2, KP3, KP4])
        warped = transform.four_point_transform(imagegrab, points, imagesize, imagesize) 
              
        # Bildrotation damit gleichwie Koordinatensystem der Maschine
        warped = cv2.rotate(warped, cv2.ROTATE_90_CLOCKWISE) 
        
        
        #Die Randberreiche mit schwarzen rechtecken abdecken, damit das Tracking nicht gverwirrt ist
        w,h,d = imagegrab.shape
        blackStrip = 30
        warped = cv2.rectangle(warped,(0,0),(w,blackStrip),(0,0,0),-1)
        warped = cv2.rectangle(warped,(0,0),(blackStrip,h),(0,0,0),-1)
        warped = cv2.rectangle(warped,(0,h),(w,h-blackStrip),(0,0,0),-1)
        warped = cv2.rectangle(warped,(w-blackStrip,0),(w,h),(0,0,0),-1)
        

        # copies of the output
        output = warped.copy()
        output2 = warped.copy()   
        # --------- Warping   
        
  
        # uncomment for non warp
        #warped = imagegrab.copy()
        #output = imagegrab.copy()
        #output2 = imagegrab.copy()   
        #points = np.array([(166, 205), (927, 205), (1009, 1007), (13, 1009)])# points = np.array([(166, 205), (927, 205), (1009, 1007), (13, 1009)])# points = np.array([(166, 205), (927, 205), (1009, 1007), (13, 1009)])# points = np.array([(166, 205), (927, 205), (1009, 1007), (13, 1009)])
        #convert to gray
        imgray = cv2.cvtColor(warped,cv2.COLOR_BGR2GRAY)
       
        #apply threshold
        ret,thresh = cv2.threshold(imgray,lower,255,cv2.THRESH_BINARY)


        ##### ------------------------------------ Circles ------------------------------------

        # detect circles in the image
        # a new image for balck circles
        threshblackened = thresh.copy()
        threshblackenedWC = thresh.copy()
        imgraygauss = imgray.copy()
        #imgraygauss = cv2.GaussianBlur(imgray,(5,5),0)
        # houghcircle transform: https://docs.opencv.org/master/dd/d1a/group__imgproc__feature.html#ga47849c3be0d0406ad3ca45db65a25d2d
        circles = cv2.HoughCircles( imgraygauss, cv2.HOUGH_GRADIENT, houghdp/10 ,houghmindist, 40,houghcanny, houghparam, houghminradius, houghmaxradius)
        print("Ich suche nach Kreisen.... ")
        
        
        # ensure at least 1 circle was found
        i=0
                
        if not(circles.shape[0] == 4 and circles.shape[1] == 1):#rows
            
            print("Ich habe ",circles.shape[1],"gefunden")
            print()
            print(circles)
            # convert the (x, y) coordinates and radius of the circles to integers
            circles = np.round(circles[0, :]).astype("int")
            #print(circles)
            # loop over the (x, y) coordinates and radius of the circles
            if circles.shape[0] >= 0:
                    for (x, y, r) in circles:
                        i=i+1
                        
                        # draw the circle in the output image, then draw a rectangle
                        # corresponding to the center of the circle
                        cv2.circle(output, (x, y), r, colorStanding, 2)
                        cv2.circle(output, (x, y), 5, colorStanding, 2)
                        cv2.putText(output, str(i), (x+5, y+5), cv2.FONT_HERSHEY_SIMPLEX,5, colorStanding, 1)
                        
                        
                        # Black circle to cover parts for contour detection (without circles)
                        cv2.circle(threshblackened, (x, y), r+blackcirclemargin, (0,0,0), -1)
                        cv2.circle(threshblackenedWC, (x, y), r+blackcirclemargin, (255,255,255), 1)
                        
                        #Fill the main Array with the data:
                        TrackedPartsCircle = np.append(TrackedPartsCircle, [[i,x,y,1,0]], axis = 0)
        else:
                print(" 0 Kreise gefunden:")
        
        ##### ------------------------------------ Contours ------------------------------------
        
        
        contours, hierarchy = cv2.findContours(threshblackened, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        print("Ich suche nach Kontouren.... ")
        
        irect=0
        
        for c in contours:
            # find bounding box coordinates
            x,y,w,h = cv2.boundingRect(c)
            #cv2.rectangle(output, (x,y), (x+w, y+h), (0, 255, 0), 2)

            # compute the area
            area = cv2.contourArea(c)
            
            if area > minareasize:
                
                irect = irect +1
                i = i +1
                # find minimum area
                rect = cv2.minAreaRect(c)
                # calculate coordinates of the minimum area rectangle
                box = cv2.boxPoints(rect)
                # normalize coordinates to integers
                box = np.int0(box)
                # calculate center and radius of minimum enclosing circle
                (x,y),radius = cv2.minEnclosingCircle(c)
                # cast to integers
                center = (int(x),int(y))
                radius = int(radius)

                #TO DO: IF radius is larger than one part --> cut part into two
                #TO DO: IF radius is larger than one part --> cut part into two
                if radius > 60:
                        # draw contours
                        cv2.drawContours(output, [box], 0, (150,20,20), 3)
                        #mark too large!
                        #istoolarge = True
                        #i=i-1 #we have a lost part
                else:
                        # draw contours
                        cv2.drawContours(output, [box], 0, colorLying, 3) 
                        
                # find the angle and the vector for a line
                angle, vector = pixelhelper.findBoxOrientation(box)

                # mark the parts
                cv2.circle(output, center, 5, colorLying, 2)
                cv2.line(output, center,tuple(center+vector), (155,155,155), 2)
                cv2.putText(output, str(i), center , cv2.FONT_HERSHEY_SIMPLEX,2, textcolor, 2)
                #cv2.putText(output, str(area), (int(x),int(y+50)) , cv2.FONT_HERSHEY_SIMPLEX,2, textcolor, 1) 
                cv2.putText(output, str(angle), (int(x),int(y+50)), cv2.FONT_HERSHEY_SIMPLEX,2, textcolor, 1)


                # This is the MAIN Data Holder for the Partpostions and Orientations
                # Fill the main Array with the data:
                #Wenn der Part nicht zu gross ist, füge ihn dem Dataholder hinzu
                #if (istoolarge == False):
                TrackedPartsRect = np.append(TrackedPartsRect, [[i,int(x),int(y),0,int(angle)]], axis = 0)


        # Show the data
        TrackedPartsRectClean = np.delete(TrackedPartsRect,0,0)
        TrackedPartsCircleClean = np.delete(TrackedPartsCircle,0,0)
        TrackedPartsAllClean = np.concatenate((TrackedPartsCircleClean, TrackedPartsRectClean), axis=0)

        NOFCircles = TrackedPartsCircleClean.shape[0]
        NOFRects = TrackedPartsRectClean.shape[0]
        NOFShapes = TrackedPartsAllClean.shape[0]
        print("Ich habe", NOFCircles ," Kreise gefunden")
        print("Ich habe", NOFRects," Rechtecke gefunden")
        print("Total NOF Shapes = ", NOFCircles+NOFRects)
        print(" [Number, x, y, isCircle?, Orientation]")
        print(TrackedPartsAllClean)
        
        
        # show all captured images und runterskalieren, damit alles auf dem Scrren platz hat
        warpedresized = pixelhelper.ResizeImage(warped,20);
        imagegrabresized = pixelhelper.ResizeImage(imagegrab,20);
        threshresized = pixelhelper.ResizeImage(thresh,20);
        threshblackenedresized = pixelhelper.ResizeImage(threshblackened,20);
        threshblackenedWCresized = pixelhelper.ResizeImage(threshblackenedWC,20);
        outputresized = pixelhelper.ResizeImage(output,20);
        imgraygaussresized = pixelhelper.ResizeImage(imgraygauss,20);

        cv2.imshow("imagegrab", imagegrabresized)
        cv2.moveWindow("imagegrab", 420,50);
        
        cv2.imshow("warped", warpedresized)
        cv2.moveWindow("warped", 870,50);
        
        cv2.imshow("imgraygauss", imgraygaussresized) 
        cv2.moveWindow("imgraygauss", 1320,50);
        
        cv2.imshow("thresh", threshresized)
        cv2.moveWindow("thresh", 0,550);
        
        cv2.imshow("threshblackened", threshblackenedWCresized)
        cv2.moveWindow("threshblackened", 420,550);
        
        cv2.imshow("threshblackenedWC", threshblackenedWCresized)
        cv2.moveWindow("threshblackenedWC", 870,550);
        
        cv2.imshow("output", outputresized)
        cv2.moveWindow("output", 1320,550);
        
        

#abschluss aktionen:

# clear the stream in preparation for the next frame
rawCapture.truncate(0)
#Set OK Pin to OFF state --> Teensy knows prgramm is running!
GPIO.output(OK_PIN, GPIO.LOW)
#alle fenster schliessen
cv2.destroyAllWindows()


