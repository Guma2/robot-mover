#!/usr/bin/env python

import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import LaserScan #import library for lidar sensor
from nav_msgs.msg import Odometry #import library for position and orientation data
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist


class Laser_act(): #laser scan class
   
    def __init__(self): #main function
        global movee
        movee = Twist() #create object of twist type  
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10) #publish message
        self.sub = rospy.Subscriber("/scan", LaserScan, self.laser_callback) #subscribe message 
        self.sub = rospy.Subscriber("/odom", Odometry, self.odometry) #subscribe message

    def laser_callback(self, msg):
        # 120 degrees into 3 regions
        regions = {
          'left':  min(min(msg.ranges[0:143]), 10),
          'front':  min(min(msg.ranges[144:287]), 10),
          'right':   min(min(msg.ranges[288:431]), 10),
        }
        take_action(regions)

    def take_action(regions):

	  threshold_dist = 0.3
	  linear_speed = 0.5
	  angular_speed = 0.2

	  msg = Twist()
	  linear_x = 0
	  angular_z = 0
	  
	  state_description = ''
	  
	  if regions['front'] > threshold_dist and regions['left'] > threshold_dist and regions['right'] > threshold_dist:
	    state_description = 'no-obstacle'
	    linear_x = 0.5
	    angular_z = 0.0

	  elif regions['front'] < threshold_dist and regions['left'] < threshold_dist and regions['right'] < threshold_dist:
	    state_description = 'ft-lt-rt-obstacles'
	    linear_x = -0.5
	    angular_z = 0.5 

	  elif regions['front'] < threshold_dist and regions['left'] > threshold_dist and regions['right'] > threshold_dist:
	    state_description = 'ft-obstacle'
	    linear_x = -0.5
	    angular_z = 0.5

	  elif regions['front'] > threshold_dist and regions['left'] > threshold_dist and regions['right'] < threshold_dist:
	    state_description = 'rt-obstacle'
	    linear_x = 0.5
	    angular_z = -0.5

	  elif regions['front'] > threshold_dist and regions['left'] < threshold_dist and regions['right'] > threshold_dist:
	    state_description = 'lt-obstacle'
	    linear_x = 0.5
	    angular_z = 0.5

	  elif regions['front'] < threshold_dist and regions['left'] > threshold_dist and regions['right'] < threshold_dist:
	    state_description = 'ft-rt-obstacles'
	    linear_x = -0.5
	    angular_z = -0.5

	  elif regions['front'] < threshold_dist and regions['left'] < threshold_dist and regions['right'] > threshold_dist:
	    state_description = 'ft-lt-obstacles'
	    linear_x = -0.5
	    angular_z = 0.5

	  elif regions['front'] > threshold_dist and regions['left'] < threshold_dist and regions['right'] < threshold_dist:
	    state_description = 'lt-rt-obstacles'
	    linear_x = 0.5
	    angular_z = 0

	  else:
	    state_description = 'unknown case'
            linear_x = 0.0
	    angular_z = 0.0
	    rospy.loginfo(regions)

	  rospy.loginfo(state_description)
	  msg.linear.x = linear_x
	  msg.angular.z = angular_z
	  pub.publish(msg)


    def odometry(self, msg): #function for odometry
	pass
        #print msg.pose.pose #print position and orientation of turtlebot


class Traffic_act: # camera class

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

    lower_white = numpy.array([0,0,180])
    upper_white = numpy.array([255,255,255])

    mask1 = cv2.inRange(hsv, lower_black, upper_black)
    mask2 = cv2.inRange(hsv, lower_white, upper_white)
    
    h, w, d = image.shape
    search_top = 3*h/4
    search_bot = 3*h/4 + 20
 
    mask1[0:search_top, 0:w] = 0
    mask1[search_bot:h, 0:w] = 0
    mask2[0:search_top, 0:w] = 0
    mask2[search_bot:h, 0:w] = 0

    M1 = cv2.moments(mask1)
    M2 = cv2.moments(mask2)

    if M2['m00'] > 0:
      cx = int(M2['m10']/M2['m00'])
      cy = int(M2['m01']/M2['m00'])
      err = cx - w/2
      print cx
      print cy
      print err
      
      self.twist.linear.x = 0.2
      self.twist.angular.z = -float(err) / 100
      self.cmd_vel_pub.publish(self.twist)
      laser_act.laser_callback(msg)
    
    #cv2.imshow("white_mask",mask2)
    #cv2.imshow("black_mask",mask1)
    #cv2.imshow("output", image)
    cv2.waitKey(3)

if __name__ == '__main__':
    rospy.init_node('dodge_track') #initilize node

    traffic_act = Traffic_act() #traffic_act object

    laser_act = Laser_act() # laser_act object
    
    rospy.spin() #loop it#!/usr/bin/env python


