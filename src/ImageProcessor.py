#!/usr/bin/env python


from collections import deque

#from matplotlib import image
from imutils.video import VideoStream
import numpy as np
import argparse
import cv2
import imutils
import time



class ImageProcessor() :
    def __init__(self):
        self.lower = (60,100,100)
        self.upper = (60,255,255)
        self.pts = deque([None]*32 , maxlen = 32)

        #self.image = image
        #self.hsv = cv2.cvtColor( self.image , cv2.COLOR_BGR2HSV)
        #self.mask = cv2.inRange(self.hsv , self.lower , self.upper)

        #self.counter = 0
        #self.dx = 0
        #self.dy = 0
        #self.dirx = ""
        #self.diry = ""
        self.direction = None

        self.centre = None
        self.radius = None


    def object_detector(self , image) :
        self.image = image
        self.hsv = cv2.cvtColor( self.image , cv2.COLOR_BGR2HSV)
        self.mask = cv2.inRange(self.hsv , self.lower , self.upper)

        cnts = cv2.findContours(self.mask.copy() , cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)

        if len(cnts) > 0 :
            c = max(cnts , key=cv2.contourArea)
            ((x,y) , self.radius) = cv2.minEnclosingCircle(c)
            
            M = cv2.moments(c)
            self.centre = (int(M["m10"] / M["m00"]) , int(M["m01"] / M["m00"]))        
            

            cv2.circle(self.image , (int(self.image.shape[0] / 2) , int(self.image.shape[1] / 2) ) , 5 , (255,0,0) , -1)

            if self.radius > 0:
                cv2.circle(self.image, self.centre , int(self.radius) , (0,255,255) , 2)  
                cv2.circle(self.image , self.centre , 5 , (0,0,255) , -1)
            
            

        return [self.image ,self.mask ,  self.centre , self.radius]


if __name__ == "__main__" :
    od = ImageProcessor()
    result = od.object_detector(cv2.imread('sample.png'))
    #print()
    #while True :

    cv2.imshow("result" , result[0])
    cv2.imshow("mask" , result[1])
    print(result[2])
    print(result[3])
    cv2.waitKey(0)



                


'''greenlower = (20,100,100)
greenupper = (30,255,255)

pts = deque([None]*32 , maxlen = 32)
counter = 0
(dx,dy) = (0,0)
(dirx , diry) = ("", "") 

direction = ""


cap = cv2.VideoCapture(1)


while True :

    

    ret , frame = cap.read()
    #frame = cv2.resize (frame)
    #frame = cv2.GaussianBlur (frame , (11,11),0)

    if ret :

        hsv = cv2.cvtColor(frame , cv2.COLOR_BGR2HSV)

        cv2.imshow("hsv" , hsv)
        mask = cv2.inRange(hsv , greenlower , greenupper)
        #mask = cv2.erode(mask , None , iterations = 2)
        #mask = cv2.dilate(mask , None , iterations = 2)

        cnts = cv2.findContours(mask.copy() , cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)  # https://learnopencv.com/contour-detection-using-opencv-python-c/
        cnts = imutils.grab_contours(cnts)      

        print("cnts : " , cnts)                                          

        center=None

        if len(cnts) > 0 :
            c = max(cnts , key=cv2.contourArea)
            ((x,y) , radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            #center = (int(M["m10"] / M["m00"]) , int(M["m01"] / M["m00"]))        # https://math.libretexts.org/Bookshelves/Calculus/Book%3A_Calculus_(OpenStax)/06%3A_Applications_of_Integration/6.6%3A_Moments_and_Centers_of_Mass
            center = (int(x) , int(y) )                                            # Inaccuracy in center is very high in this method

            if radius > 10 :
                cv2.circle(frame , (int(x) , int(y)) , int(radius) , (0,255,255) , 2)  # Circle around the object (last value is the thickness)
                cv2.circle(frame , center , 5 , (0,0,255) , -1)                        # Circle at the centre      ( thickness of -1 will fill the circle)
                pts.appendleft(center)

                for i in np.arange(1,len(pts)) :
                    if pts[i-1] is None or pts[i] is None:
                        continue

                    if counter>=10 and i==1 and pts[-10] is not None :
                        dx = pts[-10][0] - pts[i][0]
                        dy = pts[-10][1] - pts[i][1]
                        (dirx , diry) = ("" , "")

                        if np.abs(dx) > 20  :
                            if np.sign(dx) == 1:
                                dirx = "East"
                            else:
                                dirx = "West"

                        if np.abs(dy) > 20 :
                            if np.sign(dy) == 1 :
                                diry = "North"
                            else:
                                diry = "South"
                        if dirx != "" and diry != "" :
                            direction = "{} - {}".format(diry , dirx)
                    else:
                        if dirx != "":
                            direction = dirx
                        else :
                            direction = diry

            #thickness = 10
            #cv2.line(frame , pts[i - 1] , pts[i] , (0,0,255) , thickness)
        
        cv2.putText(frame , direction , (10,50) , cv2.FONT_HERSHEY_SIMPLEX , 
        2 , (0,0,255),3)

        #cv2.putText(frame , "dx : {} , dy : {}".format(dx,dy) , 
        #(10,frame.shape[0] - 10) , cv2.FONT_HERSHEY_SIMPLEX , 
        #0.35 , (0,0,255) , -1)

        cv2.imshow("Masked" , mask)
        cv2.imshow("Frame" , frame)
        counter += 1

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break 




cap.release()                       

cv2.destroyAllWindows()
'''