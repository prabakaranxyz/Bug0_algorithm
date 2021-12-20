#! /usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Twist , Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import *
import roscpp

# Global variables
robot_pos = Point()
velocity = Twist()
threh_distance = 0.05
thresh_yaw = math.pi / 80
yaw = 0
pub =0
region ={
        "right" : 0 ,
        "center" : 0,
        "left" :  0,
        "right_down": 0,
        "right_up":0 
        }

#Goal_pose
Goal_pose = Point()


#flags
state = 0
  

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

def odom_callback(msg):
    global robot_pos , yaw
    robot_pos = msg.pose.pose.position
    Q = ([msg.pose.pose.orientation.x,
         msg.pose.pose.orientation.y,
         msg.pose.pose.orientation.z,
         msg.pose.pose.orientation.w
                ])
    euler = transformations.euler_from_quaternion(Q)
    yaw = euler[2]

def correct_heading():
    global yaw, thresh_yaw, robot_pos, Goal_pose , state
    
    yaw_needed = math.atan2(Goal_pose.y - robot_pos.y, Goal_pose.x - robot_pos.x )
    turning_angle = yaw_needed - yaw
    rospy.loginfo("-----Correcting the heading towards goal--------")
    if turning_angle > math.pi :
        turning_angle = turning_angle - ((2*math.pi*turning_angle)/turning_angle)

    if (math.fabs(turning_angle) > thresh_yaw):
        if turning_angle > 0 :
            velocity.angular.z = 0.6
            velocity.linear.x = 0
            pub.publish(velocity)
        else : 
            velocity.angular.z =  -0.6
            velocity.linear.x = 0
            pub.publish(velocity)
    else :
        change_flag(1)

def allign_wall():
    global region
    rospy.loginfo("..........Alliging and following with wall............")
    while region["right"] < 0.7: 
        vel.linear.x = 0.05
        vel.angular.z=0
        pub.publish(vel)
        # rospy.loginfo("right",region["right"],"right_up",region["right_up"],"right_down",region["right_down"])
        #condition 1 if bot goes away from wall

        if region["right_up"] > 0.5 and region["right_down"] < 0.5:
            turn_right()
        
        #condition2 if bot goes near to the wall
        if region["right_up"] < 0.5 and region["right_down"] > 0.5:
            turn_left() 

        #condition3 used at cornors
        if region["right_up"] < 0.5 and region["right_down"] > 1:
            go_straight_local()
        
        #condition4 at cornors

        if region["right_up"] > 1 and region["right_down"] > 0.5:
            go_straight_local()

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

def go_straight():
    global yaw, thresh_yaw, robot_pos, Goal_pose , state , threh_distance
    yaw_needed = math.atan2(Goal_pose.y - robot_pos.y, Goal_pose.x - robot_pos.x )
    distance = math.sqrt(math.pow((Goal_pose.y - robot_pos.y) , 2) + math.pow((Goal_pose.x - robot_pos.x) , 2))
    turning_angle = yaw_needed - yaw
    rospy.loginfo("-------------Going straighnt towards goal----------------")
    if turning_angle > math.pi :
        turning_angle = turning_angle - ((2*math.pi*turning_angle)/turning_angle)
    
    if ( distance > threh_distance):
        velocity.linear.x = 0.1
        velocity.angular.z = 0
        pub.publish(velocity)
    else:
        change_flag(2)
        rospy.loginfo("---------------Goal attained---------------")
    
    if (math.fabs(turning_angle) > thresh_yaw):
        change_flag(0)

def complete_left():
    global region
    vel.linear.x=0
    pub.publish(vel)
    left_turn = 0.43 # important parameter // turns until it becomes true
    rospy.loginfo("..................Found wall turning_left............")  
    while region["right"] > left_turn:
        vel.angular.z = 0.1
        pub.publish(vel)  
        # rospy.loginfo("right",region["right"],"right_up",region["right_up"],"right_down",region["right_down"])
    
    vel.linear.z=0
    pub.publish(vel)

def turn_right():
    vel.angular.z= -0.2
    pub.publish(vel)
    rospy.loginfo("...........................turning_right....................................")
    
def go_straight_local():
    vel.linear.x = 0.05
    pub.publish(vel)
    rospy.loginfo("......................Going_straight........................")

def turn_left():
    vel.angular.z = 0.2
    pub.publish(vel)
    rospy.loginfo("..............................turning_left..........................................")

def goal_attained():
    global pub
    velocity.linear.x =0
    velocity.angular.z=0
    pub.publish(velocity)

def change_flag(n):
    global state
    state = n

def follow_wall():
    global region , state
    thresh = 0.5
    while True:
        rospy.sleep(1)
        # rospy.loginfo(region)
        if region["center"] < thresh:
            complete_left()
        if region ["center"] > thresh and region["right"] < 0.5 :
            allign_wall()
        if region ["center"] > thresh:
            state = 0
            break # used as an go straight function

if __name__ == "__main__":
    rospy.init_node("goal_pos")
    pub = rospy.Publisher("/cmd_vel",Twist,queue_size=10)
    odom = rospy.Subscriber("/odom",Odometry, odom_callback)
    laser = rospy.Subscriber("/scan" , LaserScan , callback)
    rospy.sleep(1)
    vel = Twist()
    while not rospy.is_shutdown():

        if rospy.get_param("Activate") == True:

            if rospy.get_param("x_goal") > 0 and rospy.get_param("y_goal") > 0:
                
                Goal_pose.x = rospy.get_param("x_goal")
                Goal_pose.y = rospy.get_param("y_goal")
        
                if region["center"] < 0.5:
                    follow_wall()
                if (state == 0):
                    correct_heading()
                elif(state == 1):
                    go_straight()
                elif(state == 2):
                    goal_attained()
                    break
                else:
                    rospy.logerr("Unknown State")
            else:
                rospy.loginfo("Give x,y param")
        else:
            rospy.loginfo("Activate the param")
            vel.linear.x =0
            vel.angular.x =0
            pub.publish(vel)
        
