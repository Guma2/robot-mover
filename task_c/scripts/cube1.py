#!/usr/bin/env python
#from __future__ import print_function

import sys, rospy, cv2, cv_bridge, numpy
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
import time

class image_converter:

  def __init__(self):
    self.cmd_vel_pub = rospy.Publisher("/cmd_vel",Twist, queue_size=2)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/color/image_raw",Image, self.callback)
    self.twist = Twist()

  def callback(self,data):
    image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    #converting bgr to hsv in order to identify the green color
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # inserting the range for the green color in hsv format
    lower_green = numpy.array([45, 100, 50])
    upper_green = numpy.array([70, 255, 255]) 

    # inserting the range for the green color in hsv format
    lower_red = numpy.array([0, 55, 55])
    upper_red = numpy.array([10, 255, 255]) 

    mask1 = cv2.inRange(hsv, lower_green, upper_green)
    mask2 = cv2.inRange(hsv, lower_red, upper_red)
    M1 = cv2.moments(mask1)
    M2 = cv2.moments(mask2)
    cv2.imshow("traffic Image mask", mask1)

    h, w, d = image.shape
    search_top = 3*h/4
    search_bot = 3*h/4 + 20
    mask1[0:search_top, 0:w] = 0
    mask1[search_bot:h, 0:w] = 0
    mask2[0:search_top, 0:w] = 0
    mask2[search_bot:h, 0:w] = 0

    if M2['m00'] > 0:
      cx = int(M2['m10']/M2['m00'])
      cy = int(M2['m01']/M2['m00'])
      err = cx - w/2

      if cy < 180 and cy > 152:
        self.twist.linear.x = 0.1
        self.twist.angular.z = 0.0
	self.cmd_vel_pub.publish(self.twist)
      elif cy < 152: 
        self.twist.linear.x = 0.1
        self.twist.angular.z = 0.0
      	self.cmd_vel_pub.publish(self.twist)
      else:
        self.twist.linear.x = 0.4
        self.twist.angular.z = 0.0
      	self.cmd_vel_pub.publish(self.twist)

  # Display bgr8 on a window entitled traffic Image bgr8
   
  # Display masking on a window entitled traffic Image status window
    cv2.imshow("traffic Image", image)
  # cv2.imshow("traffic Image status", crop)
    cv2.waitKey(3)

def main(args):
  ic = image_converter()
# rospy.init_node('image_converter', anonymous=True)
  rospy.init_node('traffic', anonymous = True)

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
    
