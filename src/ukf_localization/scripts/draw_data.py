#!/usr/bin/env python


from nav_msgs.msg import Odometry
import rospy
import math
import matplotlib.pyplot as plt
import numpy as np
def callback(data):
    w=data.pose.pose.orientation.w
    x=data.pose.pose.orientation.x
    y=data.pose.pose.orientation.y
    z=data.pose.pose.orientation.z

    r = math.atan2(2*(w*x+y*z),1-2*(x*x+y*y))
    p = math.asin(2*(w*y-z*x))
    y = math.atan2(2*(w*z+x*y),1-2*(z*z+y*y))
    
def listener():

    rospy.init_node('draw_data', anonymous=True)

    rospy.Subscriber('/ins550d/imu/data', Odometry, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
  

if __name__ == '__main__':
    listener()