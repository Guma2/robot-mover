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
    
    # convert image to grayscale image
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # convert the grayscale image to binary image
    ret,thresh = cv2.threshold(hsv,127,255,0)

    lower_red = numpy.array([0, 55, 55])
    upper_red = numpy.array([10, 255, 255]) 

    mask = cv2.inRange(hsv, lower_red, upper_red)

    contours, hierarchy = cv2.findContours(mask, 1, 2)
    cnt = contours[0]

    # calculate moments of binary image
    M1 = cv2.moments(mask)

    # calculate x,y coordinate of center
    cX = int(M["m10"] / M["m00"])
    cY = int(M["m01"] / M["m00"])

    # put text and highlight the center
    cv2.circle(image, (cX, cY), 5, (255, 255, 255), -1)
    cv2.putText(image, "centroid", (cX - 25, cY - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

    # display the image
    cv2.imshow("Image", image)
    cv2.waitKey(0)


rospy.init_node('follower')
follower = Follower()
rospy.spin()
