#!/usr/bin/env python

import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist


class dodge_cube:
  def __init__(self):
    self.bridge = cv_bridge.CvBridge()
    
    self.image_sub = rospy.Subscriber('/camera/color/image_raw',Image, self.image_callback)

    self.cmd_vel_pub = rospy.Publisher('/cmd_vel',Twist, queue_size=1)

    self.twist = Twist()

  def image_callback(self, msg):
    image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    lower_white = numpy.array([0,0,180])
    upper_white = numpy.array([255,255,255])

    # inserting the range for the green color in hsv format
    lower_red = numpy.array([0, 55, 55])
    upper_red = numpy.array([10, 255, 255]) 

    mask1 = cv2.inRange(hsv, lower_white, upper_white)
    mask2 = cv2.inRange(hsv, lower_red, upper_red)

    
    h, w, d = image.shape
    search_top = 3*h/4
    search_bot = 3*h/4 + 20
 
    mask1[0:search_top, 0:w] = 0
    mask1[search_bot:h, 0:w] = 0
    mask2[0:search_top, 0:w] = 0
    mask2[search_bot:h, 0:w] = 0
    

    M1 = cv2.moments(mask1)
    M2 = cv2.moments(mask2)

    if M1['m00'] > 0:
      cx1 = int(M1['m10']/M1['m00'])
      cy1 = int(M1['m01']/M1['m00'])
      err = cx1 - w/2
      
      self.twist.linear.x = 0.3
      self.twist.angular.z = -float(err) / 100
      self.cmd_vel_pub.publish(self.twist)

    elif M2['m00'] > 0:
      cx2 = int(M2['m10']/M2['m00'])
      cy2 = int(M2['m01']/M2['m00'])
      err = cx2 - w/2

      if cy2 < 180 and cy2 > 152:
        self.twist.linear.x = 0.1
        self.twist.angular.z = 0.0
	self.cmd_vel_pub.publish(self.twist)
      elif cy2 < 152: 
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
      	self.cmd_vel_pub.publish(self.twist)
      else:
        self.twist.linear.x = 0.3
        self.twist.angular.z = 0.0
      	self.cmd_vel_pub.publish(self.twist)
      
    #cv2.imshow("white_mask",mask2)
    #cv2.imshow("black_mask",mask1)
    #cv2.imshow("output", image)
    cv2.waitKey(3)

rospy.init_node('cube_node')
dodge_cube = dodge_cube()
rospy.spin()
