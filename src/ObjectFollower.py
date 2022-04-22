#!/usr/bin/env python


from ImageProcessor import ImageProcessor
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
        self.cv_image = None
        
 
    def callback(self,data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.control_loop()            
 
        except CvBridgeError as e:
            print(e)

    def control_loop(self) :
        ip = ImageProcessor()
        result = ip.object_detector(self.cv_image)    
        
        
        cv2.imshow("result" , result[0])               
        cv2.waitKey(1)

        xcentre = result[0].shape[0] / 2

        if result[2][0] > xcentre :
            print("right")
        else:
            print("left")

 
        
        
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



