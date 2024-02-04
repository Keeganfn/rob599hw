#!/usr/bin/env python3

import rospy
import sys 

import math
from sensor_msgs.msg import LaserScan


class LaserControl():
    
    def __init__(self) -> None:
        self.laser_sub = rospy.Subscriber("base_scan", LaserScan, self.laser_callback)
        self.front_laser_pub = rospy.Publisher("base_scan_front", LaserScan, queue_size=10)
        
        self.angle_min_front = -.15
        self.angle_max_front = .15


    def laser_callback(self, msg):
        cutoff = int((msg.angle_max - self.angle_max_front) // msg.angle_increment)
    

        msg.angle_min = self.angle_min_front
        msg.angle_max = self.angle_max_front
        msg.ranges = msg.ranges[cutoff:-cutoff]



        self.front_laser_pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node('laser_filter', argv=sys.argv)
    lc = LaserControl()
    rospy.spin()