#! /usr/bin/env python

import rospy

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
pub = None


def callback_laser(msg):
  # 120 degrees into 3 regions
  regions = {
    'left':  min(min(msg.ranges[0:143]), 10),
    'front':  min(min(msg.ranges[144:287]), 10),
    'right':   min(min(msg.ranges[288:431]), 10),
  }
  
  take_action(regions)
  
def take_action(regions):
  threshold_dist = 0.5
  linear_speed = 0.2
  angular_speed = 0.2

  msg = Twist()
  linear_x = 0
  angular_z = 0
  
  state_description = ''
  
  if regions['front'] > threshold_dist and regions['left'] > threshold_dist and regions['right'] > threshold_dist:
    state_description = 'no-obstacle'
    linear_x = 0.2
    angular_z = 0

  elif regions['front'] < threshold_dist and regions['left'] < threshold_dist and regions['right'] < threshold_dist:
    state_description = 'ft-lt-rt-obstacles'
    linear_x = -0.2
    angular_z = 0.5 # Increase this angular speed for avoiding obstacle faster

  elif regions['front'] < threshold_dist and regions['left'] > threshold_dist and regions['right'] > threshold_dist:
    state_description = 'ft-obstacle'
    linear_x = 0.1
    angular_z = 0.3

  elif regions['front'] > threshold_dist and regions['left'] > threshold_dist and regions['right'] < threshold_dist:
    state_description = 'rt-obstacle'
    linear_x = 0.1
    angular_z = -0.3

  elif regions['front'] > threshold_dist and regions['left'] < threshold_dist and regions['right'] > threshold_dist:
    state_description = 'lt-obstacle'
    linear_x = 0.1
    angular_z = angular_speed

  elif regions['front'] < threshold_dist and regions['left'] > threshold_dist and regions['right'] < threshold_dist:
    state_description = 'ft-rt-obstacles'
    linear_x = 0.1
    angular_z = -0.3

  elif regions['front'] < threshold_dist and regions['left'] < threshold_dist and regions['right'] > threshold_dist:
    state_description = 'ft-lt-obstacles'
    linear_x = 0.1
    angular_z = angular_speed

  elif regions['front'] > threshold_dist and regions['left'] < threshold_dist and regions['right'] < threshold_dist:
    state_description = 'lt-rt-obstacles'
    linear_x = linear_speed
    angular_z = 0

  else:
    state_description = 'unknown case'
    rospy.loginfo(regions)

  rospy.loginfo(state_description)
  msg.linear.x = linear_x
  msg.angular.z = angular_z
  pub.publish(msg)

def main():
  global pub
  
  rospy.init_node('reading_laser', anonymous = True)
  
  pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
  
  sub = rospy.Subscriber('/scan', LaserScan, callback_laser)
 #sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)
  rate = rospy.Rate(10)
    #publish rate is set at 10 Hz
 
  rospy.spin()

if __name__ == '__main__':
  main()
