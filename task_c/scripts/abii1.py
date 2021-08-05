#!/usr/bin/env python

import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist


class Follower:
  def __init__(self):
    self.bridge = cv_bridge.CvBridge()
    
    self.image_sub = rospy.Subscriber('/camera/color/image_raw',Image, self.image_callback)

    self.cmd_vel_pub = rospy.Publisher('/cmd_vel',Twist, queue_size=1)
    #self.twist = Twist()

  def image_callback(self, msg):
    image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)#convert to grayscale

    lower_yellow = numpy.array([ 10,  10,  10])
    upper_yellow = numpy.array([255, 255, 250])

   #lower_black = numpy.array([ 255,  255,  0])
   #upper_black = numpy.array([0, 0, 180])

    lower_gray = numpy.array([0, 5, 50], numpy.uint8)
    upper_gray = numpy.array([179, 50, 255], numpy.uint8)

    lower_white = numpy.array([0,0,180])
    upper_white = numpy.array([255,255,255])

    lower_green = numpy.array([36,25,25])
    upper_green = numpy.array([70,255,255])
    
    lower_red = numpy.array([0, 55, 55])
    upper_red = numpy.array([10, 255, 255]) 
    
   #mask = cv2.inRange(hsv, lower_black, upper_black)
    mask1 = cv2.inRange(hsv, lower_white, upper_white)
    mask2 = cv2.inRange(hsv, lower_green, upper_green)
    mask3 = cv2.inRange(hsv, lower_red, upper_red)
    mask4 = cv2.inRange(hsv, lower_yellow, upper_yellow)
    mask5 = cv2.inRange(hsv, lower_gray, upper_gray)

    cv2.imshow("grey_mask",mask5)
    cv2.imshow("output", image)
 
    h, w, d = image.shape
    search_top = 3*h/4
    search_bot = 3*h/4 + 20

    mask1[0:search_top, 0:w] = 0
    mask1[search_bot:h, 0:w] = 0
    mask2[0:search_top, 0:w] = 0
    mask2[search_bot:h, 0:w] = 0
    mask3[0:search_top, 0:w] = 0
    mask3[search_bot:h, 0:w] = 0
    mask4[0:search_top, 0:w] = 0
    mask4[search_bot:h, 0:w] = 0

    M1 = cv2.moments(mask1)#calculate moments of binary image
    M2 = cv2.moments(mask2)
    M3 = cv2.moments(mask3)
    M4 = cv2.moments(mask4)
    M5 = cv2.moments(mask5)


    if M3['m00'] > 0:
      cx = int(M3['m10']/M3['m00'])
      cy = int(M3['m01']/M3['m00'])

      
      #cv2.circle(image,center,radius,(0,255,0),2)

      #cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
      err = cx - w/2
      #self.twist.linear.x = 0.2
      #self.twist.angular.z = -float(err) / 100
      #self.cmd_vel_pub.publish(self.twist)
      print M3['m00']
      print M3['m10']
      print center
      print rad
  
    #cv2.imshow("mask1",mask1)
    #cv2.imshow("mask3",mask3)
    #cv2.imshow("output", image)
    cv2.waitKey(3)

rospy.init_node('follower')
follower = Follower()
rospy.spin()
