#! /usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import atan2, sqrt, pi
import random
import time

    
def callback(data):
    global position
    position=data

def gaussian_noise():
    noisy_pose=Pose()
    noisy_pose.x=position.x+random.gauss(0, 2)
    noisy_pose.y=position.y+random.gauss(0, 2)
    noisy_pose.theta=position.theta+random.gauss(0, 2)
    noisy_pose.linear_velocity=position.linear_velocity+random.gauss(0, 2)
    noisy_pose.angular_velocity=position.angular_velocity+random.gauss(0, 2)
    noisy_pose_pub.publish(noisy_pose)
#Draws circles from defined radius and velocity. 				
def circles(vel,radius):
    vel_msg=Twist()
    vel_msg.linear.x=vel
    vel_msg.angular.z=vel_msg.linear.x/radius     		   # v=rw
    vel_prev=Twist()
    ti=time.time()
    zero_vel=Twist()
    while(not rospy.is_shutdown()):

        rospy.loginfo("Radius = %f",radius)
        vel_pub.publish(vel_msg)
        rate.sleep()



if __name__ == '__main__':
    try:
        
        rospy.init_node('circle_turtle')
        position=Pose()
        vel_pub=rospy.Publisher('/turtle1/cmd_vel',Twist,queue_size=10)
        pose_pub=rospy.Publisher('/rt_real_pose',Pose,queue_size=10)
        noisy_pose_pub=rospy.Publisher('/rt_noisy_pose',Pose,queue_size=10)
        rate=rospy.Rate(10)		
        pose_sub=rospy.Subscriber('/turtle1/pose',Pose,callback)
            
        while not rospy.is_shutdown():      
            v=3   #velocity
            r=3   #Radius
            circles(v,r)

       
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")