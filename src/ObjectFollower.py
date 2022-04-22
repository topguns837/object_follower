#!/usr/bin/env python

from __future__ import print_function



import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class Object_Follower: 
    def __init__(self):
        
 
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/rrbot/camera1/image_raw",Image,self.callback)
 
    def callback(self,data):
        try:

            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

            #while cv_image is not None :

            #hsv = cv2.cvtColor(cv_image , cv2.COLOR_BGR2HSV)

            cv2.imshow("Image" , cv_image)
            cv2.waitKey(1)
               
 
        except CvBridgeError as e:
            print(e)
 
        
        
def main(args):
    ic = Object_Follower() 
       
    rospy.init_node('image_converter', anonymous=True)

    try:
        
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
 
if __name__ == '__main__':
     main(sys.argv)



