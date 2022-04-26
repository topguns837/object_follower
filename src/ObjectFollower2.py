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
        self.radius_tolerance = 1

        self.xcentre = None
        self.result = None
        self.velocity_msg = Twist()

        #self.odom = OdomSubscriber()
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
 
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
            pass
        else:
            self.velocity_msg.angular.z = self.give_angular_error()
            self.velocity_msg.linear.x = self.give_linear_error()         

   

        self.pub.publish(self.velocity_msg)

        cv2.imshow("Detected_Objects" , self.result[0])   
        cv2.imshow("Positions_Detector" , self.result[1])            
        cv2.waitKey(1)
        
        #if self.result[-1] < self.radius_tolerance :
            #self.move_forward()

    def give_angular_error(self) :
        


    def move_forward(self) :
        if self.result[-1] < self.radius_tolerance :
            self.velocity_msg.linear.x = 0.1
            self.pub.publish(self.velocity_msg)
        

    def fix_yaw(self) :

        self.velocity_msg.angular.z = (-1) * np.sign(self.result[2][0] - self.xcentre) * (0.1)
        self.pub.publish(self.velocity_msg)
 
        '''if self.result[2][0] > self.xcentre + self.angle_tolerance :
            self.velocity_msg.angular.z = - 0.1
            self.pub.publish(self.velocity_msg)
            #self.move(0,self.P*-1*orien_error)


        elif self.result[2][0] < self.xcentre - self.angle_tolerance :
            #print("move left")
            self.velocity_msg.angular.z = 0.1
            self.pub.publish(self.velocity_msg)

        else :
            self.velocity_msg.angular.z = 0.0
            self.pub.publish(self.velocity_msg)
        '''

 
        
        
def main(args):
    rospy.init_node('obj_follower2', anonymous=True)
    of = Object_Follower()        
    

    try:        
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
 
if __name__ == '__main__':
     main(sys.argv)



