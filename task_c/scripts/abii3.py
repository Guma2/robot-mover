#!/usr/bin/env python

import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist


class black:
  def __init__(self):
    self.bridge = cv_bridge.CvBridge()
    
    self.image_sub = rospy.Subscriber('/camera/color/image_raw',Image, self.image_callback)

    self.cmd_vel_pub = rospy.Publisher('/cmd_vel',Twist, queue_size=1)

    self.twist = Twist()

  def image_callback(self, msg):
    image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    lower_black = numpy.array([ 0, 0, 0])
    upper_black = numpy.array([180, 255, 30])

    mask = cv2.inRange(hsv, lower_black, upper_black)
 
    h, w, d = image.shape
    search_top = 3*h/4
    search_bot = 3*h/4 + 20
 
    mask[0:search_top, 0:w] = 0
    mask[search_bot:h, 0:w] = 0
    M = cv2.moments(mask)

    if M['m00'] > 0:
      cx = int(M['m10']/M['m00'])
      cy = int(M['m01']/M['m00'])
      err = cx - w/2

      self.twist.linear.x = 0.2
      self.twist.angular.z = (-float(err) / 100) - 0.2
      
      
      if cx > 400:
        #self.twist.linear.x = 0.2
        self.twist.angular.z = -float(err) / 100
       

      elif cx < 250:
	#self.twist.linear.x = 0.2
        self.twist.angular.z = float(err) / 100
       
      self.cmd_vel_pub.publish(self.twist)
      
    #cv2.imshow("mask",mask)
    #cv2.imshow("output", image)
    cv2.waitKey(3)

rospy.init_node('black_node')
black = black()
rospy.spin()
