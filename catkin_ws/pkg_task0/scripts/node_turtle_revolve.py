#!/usr/bin/env python
#-----------------------------------------------------------------#
#------Importing important libraries , topics and message---------#
#-----------------------------------------------------------------# 
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
pi = 3.1415926535
#-----------------------------------------------------------------#

#-----------------------------------------------------------------#
#---------------defination of callback function-------------------#
#-----------------------------------------------------------------#
def pose_callback(msg):
    global theta       #setting theta as Global varible to use it in 
    theta = msg.theta  #revolve() function 
#-----------------------------------------------------------------#

#-----------------------------------------------------------------#
#----------------Defination of revolve funtion--------------------#
#-----------------------------------------------------------------#
def revolve():

    #initialising revolution node 
    rospy.init_node('turtle_revolve', anonymous=True)

    #initialing publisher on topic /turtle1/cmd_vel with type
    #geometry_msgs/Twistwist and queue_size = 1
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel',
                                          Twist, 
                                          queue_size=1)

    #initialising Subcriber on topic /turtle1/pose with message
    #Pose and pose_callback function
    rospy.Subscriber("/turtle1/pose", Pose, pose_callback)

    #Setting some rate for Publishing 
    rate = rospy.Rate(10)

    #How to get circle ?
    #--linear velocity in x direction should be constant
    #--velocites in y and z direction should be zero
    #--angular velocity in z direction should be some constant
    #--angular velocity in y and z should be zero
  
    #Setting Linear and Angular velocity
    linear_velocity = 1    #unit is meter/second
    angular_velocity = 1    #units is radian/second

    #Calculation of radius , circumference and time required
    radius = linear_velocity / angular_velocity
    
    #running while to make turtle move in circular path
    while not rospy.is_shutdown():
        vel_msg = Twist()
        vel_msg.linear.x =linear_velocity
        vel_msg.angular.z =angular_velocity
        vel_msg.linear.y = 0.00
        vel_msg.linear.z = 0.00
        vel_msg.angular.x = 0.00
        vel_msg.angular.y = 0.00
        
        #publishing velocities to topic 
        velocity_publisher.publish(vel_msg)
        rate.sleep()

        #Setting logic to deal with negative values of radian
        #and select proper distance travelled
        x1 = abs(theta)
        x2 = (2 * pi) - abs(theta)
        distance = x1 * radius
        # Recording value of distance travelled after half revolution
        if theta < 0 :
            distance = x2 * radius  

        #printing Distance travelled
        rospy.loginfo("Turtle moving in circle...")
        rospy.loginfo("Distance travelled {:.2f} .".format(distance))

        #Condition to stop Turtle after one revolution
        if distance >= 6.1:   #where 6.1 is circumference = 2*pi*radius
            break
        
    #Force stoping your turtle when condition is fulfilled   
    vel_msg.linear.x=0.0
    vel_msg.angular.z=0.0
    velocity_publisher.publish(vel_msg)
    rospy.loginfo("Goal Reached...")

#Defination of main 
if __name__ == '__main__':
    revolve()
