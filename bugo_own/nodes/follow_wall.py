#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_srvs.srv import *
pub=0
region ={
        "right" : 0 ,
        "center" : 0,
        "left" :  0,
        "right_down": 0,
        "right_up":0 
        }

def callback(scan_data):
    global region
    ranges = scan_data.ranges
    front_range = ranges[350:359] + ranges[0:20]
    left_range = ranges[80:90]
    right_range = ranges[269:270]
    right_range = right_range[0]
    right_down = ranges[239:240]
    right_up = ranges[309:310]
    right_down = right_down[0]
    right_up=right_up[0]
    min_value_left = min(left_range)
    # min_value_right = min(right_range)
    min_value_front = min(front_range)
    # region["right"] = min(min_value_right,2)
    region["right"] = right_range
    region["center"] = min(min_value_front,2)
    region["left"] = min(min_value_left,2)
    region["right_down"] = right_down
    region["right_up"] = right_up

    
def turn_left():
    global region
    vel.linear.x=0
    pub.publish(vel)
    while region["right"] > 0.40:
        vel.angular.z = 0.1
        pub.publish(vel)  
        print("turning_left")  
        print("right",region["right"],"right_up",region["right_up"],"right_down",region["right_down"])
    vel.linear.z=0
    pub.publish(vel)


def go_straight():
    vel.linear.x = 0.1
    pub.publish(vel)
    print("......................Going_straight........................")

def turn_left():
    vel.angular.z = 0.2
    pub.publish(vel)
    print("..............................turning_left..........................................")

def turn_right():
    vel.angular.z= -0.2
    pub.publish(vel)
    print("...........................turning_right....................................")
    

def follow_wall():
    global region
    while region["right"] < 0.7:
        vel.linear.x = 0.05
        vel.angular.z=0
        pub.publish(vel)
        print("right",region["right"],"right_up",region["right_up"],"right_down",region["right_down"])
        #condition 1 if bot goes away from wall

        if region["right_up"] > 0.5 and region["right_down"] < 0.5:
            turn_right()
        
        #condition2 if bot goes near to the wall
        if region["right_up"] < 0.5 and region["right_down"] > 0.5:
            turn_left() 

        #condition3 used at cornors
        if region["right_up"] < 0.5 and region["right_down"] > 1:
            go_straight()
        
        #condition4 at cornors

        if region["right_up"] > 1 and region["right_down"] > 0.5:
            go_straight()

        #condition5 to stay parrallel to the wall

        if region["right_up"] < 0.5 and region["right_down"] < 0.5:
            turn_left()

        #condition6 used at L shaped walls

        if region["center"] < 0.5:
            turn_left()
            rospy.sleep(5)
        
    vel.linear.x =0
    vel.angular.z =0
    pub.publish(vel)

def conditions():
    thresh = 0.5
    if region["center"] < thresh:
        turn_left()
    if region ["center"] > thresh and region["right"] < 0.5 :
        follow_wall()
    if region ["center"] > thresh:
        go_straight() # used as an go straight function
    

if __name__ == "__main__":
    while not rospy.is_shutdown():
        rospy.init_node("wall_follow")
        pub = rospy.Publisher("/cmd_vel" , Twist , queue_size=1)
        rospy.Subscriber("/scan" , LaserScan , callback)
        rospy.sleep(1)
        vel = Twist()
        conditions()
        print("right",region["right"],"right_up",region["right_up"],"right_down",region["right_down"])
        