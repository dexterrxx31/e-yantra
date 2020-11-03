#!/usr/bin/env python

##########################Essential Import Libraries###########################
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math
import numpy as np   #for using linspace() function and finding wave points
###############################################################################

#########################Declaring Global Variables############################
pi = math.pi
pose = [0,0,0]
goal = [12.5,0,0]
regions = {
        'bright':0,
        'fright':0,
        'front':0,
        'fleft':0,
        'bleft':0,
        }
################################################################################

###############################Odom's Callback Function#########################
def odom_callback(data):
    global pose 
    x = data.pose.pose.orientation.x;
    y = data.pose.pose.orientation.y;
    z = data.pose.pose.orientation.z;
    w = data.pose.pose.orientation.w;
    pose = [data.pose.pose.position.x, data.pose.pose.position.y, euler_from_quaternion([x,y,z,w])[2]]
################################################################################

################################Laser's Callback Function#######################
def laser_callback(msg):
    global regions
    regions = {
        'bright':  min(min(msg.ranges[0:143]) , 5),
        'fright': min(min(msg.ranges[144:287]), 5),
        'front':  min(min(msg.ranges[288:431]), 5),
        'fleft':  min(min(msg.ranges[432:575]), 5),
        'bleft':   min(min(msg.ranges[576:719]), 5),
    }
#################################################################################

###################################Way Point Function############################
def Waypoints(t):
    x  = t
    y  = 2*math.sin(x)*math.sin(x/2)  #Y condinates of wave points
    return [x,y]
#################################################################################

##############################Main Controll Function#############################
def control_loop():

    #Declaring Global Variables
    global pose
    global regions

    rospy.init_node('ebot_controller')  #node initialization

    ################Publishers and Subscriber######################
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/ebot/laser/scan', LaserScan, laser_callback)
    rospy.Subscriber('/odom', Odometry, odom_callback)
    
    rate = rospy.Rate(10) 

    velocity_msg = Twist()
    velocity_msg.linear.x = 0
    velocity_msg.angular.z = 0
    pub.publish(velocity_msg)

    #finding wave points
    list_of_way_points = list()
    x_co_of_way_points = np.linspace(0,2*pi,15)
    for item in x_co_of_way_points:
        list_of_way_points.append(Waypoints(item))
    
    p_const = 0.5  #Proportionality Constant for wave points


    while not rospy.is_shutdown():

        #Logic for tracing wave points
        for item in range (1,15):
            cordi_curr = list_of_way_points[item-1]
            cordi_next = list_of_way_points[item]
            theta_goal = math.atan((cordi_next[1]-cordi_curr[1])/(cordi_next[0]-cordi_curr[0]))
            error_theta = theta_goal - pose[2]
            while True :
                velocity_msg.linear.x = 0.1
                velocity_msg.angular.z = p_const * error_theta
                pub.publish(velocity_msg)
    	        print("Controller message pushed at {}".format(rospy.get_time()))
                if pose[0] >= cordi_next[0] :
                    break

        #Algorithm to tackle Concave Obstacle and reach Goal             
        while True:
            velocity_msg.angular.z = 0
            velocity_msg.linear.x = 0.1
            pub.publish(velocity_msg)
            if regions["front"] <= 1.5 :
                velocity_msg.linear.x = 0 
                velocity_msg.angular.z = 1 
                pub.publish(velocity_msg)
            if regions['fright'] >= 5 :
                anl = math.atan((0-pose[1])/(12.5-pose[0]))
                err = anl - pose[2]
                velocity_msg.angular.z = 18.5 * err
                velocity_msg.linear.x = 0.08
                pub.publish(velocity_msg)
                if pose[0]>=12.5:
                    velocity_msg.linear.x =0
                    velocity_msg.angular.z =0
                    pub.publish(velocity_msg)
                    print("Goal Reached")
                break
    	print("Controller message pushed at {}".format(rospy.get_time()))
        rate.sleep() 
    	
#main function
if __name__ == '__main__':
    try:
        control_loop()
    except rospy.ROSInterruptException:
        pass
