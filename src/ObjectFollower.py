#!/usr/bin/env python

from ImageProcessor import ImageProcessor
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
import numpy as np


class Object_Follower: 
    def __init__(self):        
 
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/rrbot/camera1/image_raw",Image,self.callback)
        self.cv_image = None

        self.angle_tolerance = 150
        self.radius_tolerance = 130

        self.xcentre = None
        self.result = None
        self.velocity_msg = Twist()

        #self.odom = OdomSubscriber()
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.angular_text = ""
        self.linear_text = ""

        
        
 
    def callback(self,data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.control_loop()            
 
        except CvBridgeError as e:
            print(e)

    def control_loop(self) :
        ip = ImageProcessor()
        self.result = ip.object_detector(self.cv_image)  

        if self.result[-1] == None :
            self.velocity_msg.linear.x = 0
            self.velocity_msg.angular.z = 0

            self.angular_text = "No object Found !"
            self.linear_text = " "
        else:
            self.mask = self.result[1]

            self.radius = self.result[-1]      
        
        

            self.xcentre = self.result[0].shape[0] / 2
            

            cond1 = (self.xcentre > self.result[2][0] + self.radius) or (self.xcentre < self.result[2][0] - self.radius)
            cond2 = self.radius < self.radius_tolerance

            if cond1 and cond2 :                
                self.velocity_msg.angular.z = (-1) * np.sign(self.result[2][0] - self.xcentre) * (0.3)            
                self.velocity_msg.linear.x = 0.75             
                
            elif (not cond1) and (cond2) :                
                self.velocity_msg.angular.z = 0            
                self.velocity_msg.linear.x = 0.75                

            elif (not cond2) and (cond1) :                
                self.velocity_msg.angular.z = (-1) * np.sign(self.result[2][0] - self.xcentre + self.radius ) * (0.3)            
                self.velocity_msg.linear.x = 0.0         
            
            else :                
                self.velocity_msg.linear.x = 0
                self.velocity_msg.angular.z = 0



            if self.velocity_msg.angular.z < 0 :
                self.angular_text = "Go Right =====>>>"
            elif self.velocity_msg.angular.z > 0:
                self.angular_text = "Go Left <<<====="
            else:
                self.angular_text = "Centre" 


            if self.velocity_msg.linear.x > 0:
                self.linear_text = "Forward"
            else:
                self.linear_text = "Stop"
        
            
        self.pub.publish(self.velocity_msg)    

        

               
        cv2.putText(self.result[0] , self.angular_text , (350,50) , cv2.FONT_HERSHEY_SIMPLEX , 1 , (255, 0, 0) , 2 , cv2.LINE_AA)
        cv2.putText(self.result[0] , self.linear_text , (350,700) , cv2.FONT_HERSHEY_SIMPLEX , 1 , (0, 255, 0) , 2 , cv2.LINE_AA)

        


        #cv2.line(self.result[1] , (int(self.xcentre) , 0) , (int(self.xcentre) ,self.result[0].shape[1] ) , (255,0,0) , 5)
        
        coloured_mask = self.cv_image.copy()
        coloured_mask[self.result[1] > 0] = (0,255,0)
        coloured_mask[self.result[1] == 0] = [0,0,0]

        cv2.line(coloured_mask , (int(self.xcentre) , 0) , (int(self.xcentre) ,self.result[0].shape[1] ) , (255,0,0) , 5)

        try :
            cv2.putText(coloured_mask , "Area of circle : {}".format(np.round(np.pi*(self.result[-1]**2)),2), (50,50) , cv2.FONT_HERSHEY_SIMPLEX , 1 ,(255,0,0) , 2 , cv2.LINE_AA)
        except :
            pass
        cv2.imshow("Positions_Detector" , coloured_mask) 
        cv2.imshow("Detected_Objects" , self.result[0])         
        
                   
        cv2.waitKey(1)

            

        
        

    


    
 
if __name__ == '__main__':
    rospy.init_node('obj_follower', anonymous=True)
    of = Object_Follower()        
    

    try:        
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
    



