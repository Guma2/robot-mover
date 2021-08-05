#!/usr/bin/env python

import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import LaserScan #import library for lidar sensor
from nav_msgs.msg import Odometry #import library for position and orientation data
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist


class Follower:
  def __init__(self):
    self.bridge = cv_bridge.CvBridge()
    
    self.image_sub = rospy.Subscriber('/camera/color/image_raw',Image, self.image_callback)

    self.cmd_vel_pub = rospy.Publisher('/cmd_vel',Twist, queue_size=1)
    self.twist = Twist()

  def image_callback(self, msg):
    image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

   #lower_yellow = numpy.array([ 10,  10,  10])
   #upper_yellow = numpy.array([255, 255, 250])

    lower_black = numpy.array([ 255,  255,  0])
    upper_black = numpy.array([0, 0, 180])

    lower_white = numpy.array([0,0,180])
    higher_white = numpy.array([255,255,255])

    mask = cv2.inRange(hsv, lower_black, upper_black)
    #mask = cv2.inRange(hsv, lower_white, higher_white)
    
    h, w, d = image.shape
    search_top = 3*h/4
    search_bot = 3*h/4 + 20
    mask[0:search_top, 0:w] = 0
    mask[search_bot:h, 0:w] = 0
    M = cv2.moments(mask)

    if M['m00'] > 0:
      cx = int(M['m10']/M['m00'])
      cy = int(M['m01']/M['m00'])
      #cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
      
      err = cx - w/2
      self.twist.linear.x = 0.2
      self.twist.angular.z = -float(err) / 100
      self.cmd_vel_pub.publish(self.twist)
      
    cv2.imshow("mask",mask)
    cv2.imshow("output", image)
    cv2.waitKey(3)

    Circling() #run class


class Circling(): #main class
   
    def __init__(self): #main function
        global circle
        circle = Twist() #create object of twist type  
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10) #publish message
        self.sub = rospy.Subscriber("/scan", LaserScan, self.callback) #subscribe message 
        self.sub = rospy.Subscriber("/odom", Odometry, self.odometry) #subscribe message

    def callback(self, msg): #function for obstacle avoidance
        print '-------RECEIVING LIDAR SENSOR DATA-------'
        print 'Front: {}'.format(msg.ranges[0]) #lidar data for front side
        print 'Left:  {}'.format(msg.ranges[90]) #lidar data for left side
        print 'Right: {}'.format(msg.ranges[270]) #lidar data for right side
        print 'Back: {}'.format(msg.ranges[180]) #lidar data for back side
      
      	#Obstacle Avoidance
        self.distance = 0.7
        if msg.ranges[0] > self.distance and msg.ranges[15] > self.distance and msg.ranges[345] > self.distance: 
        #when no any obstacle near detected
            circle.linear.x = 0.2 # go (linear velocity)
            circle.angular.z = 0.0 # rotate (angular velocity)
            rospy.loginfo("Circling") #state situation constantly
        else: #when an obstacle near detected
            rospy.loginfo("An Obstacle Near Detected") #state case of detection
            circle.linear.x = 0.0 # stop
            circle.angular.z = 0.5 # rotate counter-clockwise
            if msg.ranges[0] > self.distance and msg.ranges[15] > self.distance and msg.ranges[345] > self.distance and msg.ranges[45] > self.distance and msg.ranges[315] > self.distance:
                #when no any obstacle near detected after rotation
                circle.linear.x = 0.2 #go
                circle.angular.z = -0.2 #rotate
	print circle.linear.x
	print circle.linear.z
        self.pub.publish(circle) # publish the move object

    def odometry(self, msg): #function for odometry
	pass
        #print msg.pose.pose #print position and orientation of turtlebot

if __name__ == '__main__':
    rospy.init_node('follow_lane') #initilize node
    follower = Follower()
    
    rospy.spin() #loop it
