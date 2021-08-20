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

    def laser_callback(self, msg): #function for obstacle avoidance
        print '-------RECEIVING LIDAR SENSOR DATA-------'
        print 'Front: {}'.format(msg.ranges[0]) #lidar data for front side
        print 'Left:  {}'.format(msg.ranges[90]) #lidar data for left side
        print 'Right: {}'.format(msg.ranges[270]) #lidar data for right side
        print 'Back: {}'.format(msg.ranges[180]) #lidar data for back side
      
      	#Obstacle Avoidance
        self.distance = 0.3
        if msg.ranges[0] > self.distance and msg.ranges[90] > self.distance and msg.ranges[270] > self.distance:
            movee.linear.x = 0.5 
            movee.angular.z = 0.0 
            rospy.loginfo("no-obstacle") #state situation constantly
	if msg.ranges[0] < self.distance and msg.ranges[90] > self.distance and msg.ranges[270] > self.distance:
            movee.linear.x = -0.5 
            movee.angular.z = 0.5 
            rospy.loginfo("ft-obstacle") 
	if msg.ranges[0] > self.distance and msg.ranges[90] < self.distance and msg.ranges[270] > self.distance:
            movee.angular.z = -0.5
	    movee.linear.x = 0.5 
            rospy.loginfo("rt-obstacle") 
        if msg.ranges[0] > self.distance and msg.ranges[90] > self.distance and msg.ranges[270] < self.distance:
            movee.angular.z = 0.5
	    movee.linear.x = 0.5 
            rospy.loginfo("lt-obstacle")
        if msg.ranges[0] < self.distance and msg.ranges[90] < self.distance and msg.ranges[270] > self.distance:
            movee.linear.x = -0.5 
            movee.angular.z = -0.2 
            rospy.loginfo("ft-rt-obstacle")  
	if msg.ranges[0] < self.distance and msg.ranges[90] > self.distance and msg.ranges[270] < self.distance:
            movee.linear.x = -0.5 
            movee.angular.z = 0.2 
            rospy.loginfo("ft-lt-obstacle")  
	if msg.ranges[0] > self.distance and msg.ranges[90] < self.distance and msg.ranges[270] < self.distance:
            movee.angular.z = 0.0
	    movee.linear.x = 0.5  
            rospy.loginfo("lt-rt-obstacle") 
	if msg.ranges[0] < self.distance and msg.ranges[90] < self.distance and msg.ranges[270] < self.distance:
            movee.linear.x = -0.5 
            movee.angular.z = 0.2 
            rospy.loginfo("ft-rt-lt-obstacle")   
 	else: 
            rospy.loginfo("UnKnown-case")
            movee.linear.x = 0.0 # stop
            movee.angular.z = 0.0 
             
	print movee.linear.x
	print movee.linear.z
        self.pub.publish(movee) # publish the move object

    def odometry(self, msg): #function for odometry
	pass
        #print msg.pose.pose #print position and orientation of turtlebot


if __name__ == '__main__':
    rospy.init_node('dodge_track') #initilize node

    laser_act = Laser_act() # laser_act object
    
    rospy.spin() #loop it#!/usr/bin/env python


