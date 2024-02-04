#!/usr/bin/env python

import rospy
import sys
import math
import tf
import numpy as np
from numpy.polynomial.polynomial import polyfit 
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from std_msgs.msg import Float64

#Code borrowed from my code I wrote when I took this class in undergrad

#Creates text marker that says current angle to wall
def set_text_marker(angle):
    angle_text = Marker() 
    angle_text.header.frame_id = "laser_link"
    angle_text.type = angle_text.TEXT_VIEW_FACING
    angle_text.id = 2 
    angle_text.action = angle_text.ADD
    angle_text.color.r = 0
    angle_text.color.g = 1
    angle_text.color.b = 0
    angle_text.color.a = 1.0
    angle_text.scale.z = .35
    angle_text.pose.position.x = -0.8 
    angle_text.pose.position.y = 0.0
    angle_text.pose.position.z = 0.0
    angle_text.text = str(round(angle, 4))
    return angle_text

def callback(msg):
    count = 0
    x = []
    y = []
    #for every range that isnt inf for the filtered laserscan it is converted to an x,y coordinate and added to the respective array
    for i in msg.ranges: 
        if(i != float("inf")):
            #keeps track of the current angle 
            laser_angle = msg.angle_min + (msg.angle_increment * count)         
            x.append(i * math.cos(laser_angle))    
            y.append(i * math.sin(laser_angle))    
        count+=1
    #fits a line to the arrays we just created and returns the slope and intercept 
    slope, intercept = polyfit(x, y, 1)
    
    #we take the arctan of the slope to find the angle the wall is to us, we either subtract or add 90deg in rads so that facing forward is 0 rad
    #publishes the text marker with our value and the base float value
    if(math.atan(slope) > 0):
        pub_marker.publish(set_text_marker(math.atan(slope) - 1.5708))
        pub_angle.publish(math.atan(slope) - 1.5708)
    else:
        pub_marker.publish(set_text_marker(math.atan(slope) + 1.5708))
        pub_angle.publish(math.atan(slope) + 1.5708)
     

if __name__ == "__main__":
    rospy.init_node("line_fit", argv=sys.argv)
    sub = rospy.Subscriber("base_scan_front", LaserScan, callback, queue_size = None)
    pub_marker = rospy.Publisher("angle_marker", Marker, queue_size=10)
    pub_angle = rospy.Publisher("angle_to_wall", Float64, queue_size=10)
    rospy.spin()