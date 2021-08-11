#!/usr/bin/env python

import rospy, cv2, cv_bridge, numpy, os, threading, time, sys
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

class image_converter:

  def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber("/camera/color/image_raw",Image, self.callback)

  def callback(self,msg):
    cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    #converting bgr to hsv in order to identify the red color
    hsv_cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
   
    lower_red = numpy.array([0, 55, 55])
    upper_red = numpy.array([10, 255, 255]) 

    masking = cv2.inRange(hsv_cv_image, lower_red, upper_red)
    M = cv2.moments(masking)
   #cv2.imshow("traffic", cv_image)
    cv2.imshow("traffic", masking)

    if M['m00'] > 0:
      cx = int(M['m10']/M['m00'])
      cy = int(M['m01']/M['m00'])
      print cx
      print cy

    cv2.waitKey(3)

def main(args):
    ic = image_converter()
    rospy.init_node('camera', anonymous =True)
   #pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 2)


    try:
     rospy.spin()
    except KeyboardInterrupt:
     print("Shutting down")
    cv2.destroyAllWindows()
   

 #  rate = rospy.Rate(10) # 10hz
    # Spin until ctrl + c
#   rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
