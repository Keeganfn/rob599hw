#!/usr/bin/env python3

# Some code like marker display and action server taken/modified from my code from when I took class in undergrad

import rospy
import sys 
import math
import actionlib
import numpy as np
import tf


from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from rob599_hw1.srv import StoppingDistance, StoppingDistanceResponse
from rob599_hw1.msg import ForwardWallMoveAction, ForwardWallMoveGoal, ForwardWallMoveFeedback, ForwardWallMoveResult

class LaserControl():
    
    def __init__(self) -> None:
        self.laser_sub = rospy.Subscriber("base_scan_front", LaserScan, self.laser_callback)
        self.twist_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.distance_srv = rospy.Service("stopping_distance", StoppingDistance, self.distance_callback)
        self.pub_marker = rospy.Publisher("closest_point_marker", Marker, queue_size=10)

        self.MAX_VEL = .4
        self.stopping_point = 1
        self.action_active = False

        self.action_server = actionlib.SimpleActionServer("set_laser_goal", ForwardWallMoveAction, auto_start=False)
        self.action_server.register_goal_callback(self.action_callback)
        self.action_server.start()

    def move_forward(self, speed):
        cmd = Twist()
        cmd.linear.x = speed
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0
        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = 0.0
        self.twist_pub.publish(cmd)

    def laser_callback(self, msg):
        closest_point = min(msg.ranges) 
        speed = 0

        if(self.action_active == True):
            self.action_server.publish_feedback(ForwardWallMoveFeedback(closest_point = closest_point))
            if closest_point < self.stopping_point + .03:
                self.action_active = False
                self.action_server.set_succeeded(ForwardWallMoveResult(result = True))

        if closest_point >= self.stopping_point + .03:
            if closest_point <= self.stopping_point + 1:
                speed = max(self.MAX_VEL * (closest_point - self.stopping_point), .05)
            else:
                speed = self.MAX_VEL
            self.move_forward(speed)
        
        pub_arrow_marker = self.set_arrow_marker(msg.angle_min + (msg.angle_increment * np.argmin(msg.ranges)), closest_point)
        pub_text_marker = self.set_text_marker(closest_point)

        rospy.loginfo("Closest Obstacle: " + str(closest_point) + " Current Speed: " + str(speed))

    def distance_callback(self, msg):
        if not self.action_active:
            if msg.distance >= .5 and isinstance(msg.distance, float):
                self.stopping_point = msg.distance
                return StoppingDistanceResponse(True)
        rospy.loginfo("Not a valid distance.")
        return StoppingDistanceResponse(False)
    
    def action_callback(self):
        self.action_active = True
        goal = self.action_server.accept_new_goal()

        if(isinstance(goal.distance, float) and goal.distance > .5):
            self.stopping_point = goal.distance 
        else:
            rospy.loginfo("Not a valid distance.")
            self.action_active = False
            self.action_server.set_aborted(ForwardWallMoveResult(result = False))

    def preempt_callback(self):
        self.action_active = False
        self.action_server.set_preempted(ForwardWallMoveResult(result = False))

    def set_arrow_marker(self, angle, point):
        rotation = tf.transformations.quaternion_from_euler(0,0,angle)
        arrow = Marker()
        arrow.header.frame_id = "laser_link"
        arrow.type = arrow.ARROW
        arrow.id = 0
        arrow.action = arrow.ADD
        arrow.scale.x = point
        arrow.scale.y = .05
        arrow.scale.z = .05
        arrow.color.r = 0
        arrow.color.g = 0
        arrow.color.b = 1.0
        arrow.color.a = 1.0
        arrow.pose.position.x = 0.0
        arrow.pose.position.y = 0.0
        arrow.pose.position.z = 0.0
        arrow.pose.orientation.x = rotation[0]
        arrow.pose.orientation.y = rotation[1]
        arrow.pose.orientation.z = rotation[2]
        arrow.pose.orientation.w = rotation[3]
        self.pub_marker.publish(arrow)

    def set_text_marker(self, point):
        distance_text = Marker() 
        distance_text.header.frame_id = "laser_link"
        distance_text.type = distance_text.TEXT_VIEW_FACING
        distance_text.id = 1 
        distance_text.action = distance_text.ADD
        distance_text.color.r = 0
        distance_text.color.g = 0
        distance_text.color.b = 1.0
        distance_text.color.a = 1.0
        distance_text.scale.z = .25
        distance_text.pose.position.x = point - .2
        distance_text.pose.position.y = 0.4
        distance_text.pose.position.z = 0.0
        distance_text.text = str(point)[:5]
        self.pub_marker.publish(distance_text)

if __name__ == "__main__":
    rospy.init_node('laser_control', argv=sys.argv)
    lc = LaserControl()
    rospy.spin()