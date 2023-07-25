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

def step_vel(vf,vi,time_sent):
    K_accel=0.1
    K_decel=0.3
    max_accel_linear=1500   
    max_decel_linear=-1500
    max_accel_angular=6000
    max_decel_angular=-6000
    if(vf.linear.x-vi.linear.x>0):  
        k=K_accel
        max_steps_linear=max_accel_linear 
        max_steps_angular=max_accel_angular
    else: 
        k=K_decel
        max_steps_linear=max_decel_linear
        max_steps_angular=max_decel_angular

    step_vel=Twist()
    step=k*(vf.linear.x-vi.linear.x)		#Incrementing in steps to keep accel in limit
    t_now=time.time()
    dt=t_now-time_sent
    if(abs(step/dt)>abs(max_steps_linear)):
        step=dt*max_steps_linear			#set step corresponding to max accel
        
    stepangular=4*k*(vf.angular.z-vi.angular.z)		#Incrementing in steps to keep accel in limit
    if(abs(stepangular/dt)>abs(max_steps_angular)):		
        stepangular=dt*max_steps_angular			#set step corresponding to max accel
        
    vi.linear.x=vi.linear.x+step
    vi.angular.z=vi.angular.z+stepangular
    vel_pub.publish(vi)
    timer=time.time()
    return(vi,timer)		


#Adding Gaussian Noise using random.gauss function. 
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

            tf=time.time()
            vel_prev,tf=step_vel(vel_msg,vel_prev,tf)		#gradually increase velocity to trace circles. Hence spiral can be seen initially.
            rate.sleep()
            if(time.time()-ti>5):
                pose_pub.publish(position)
                ti=time.time()
                gaussian_noise()


if __name__ == '__main__':
    try:
        
        rospy.init_node('turtle_in_circle')
        position=Pose()
        vel_pub=rospy.Publisher('/turtle1/cmd_vel',Twist,queue_size=10)
        pose_pub=rospy.Publisher('/rt_real_pose',Pose,queue_size=10)
        noisy_pose_pub=rospy.Publisher('/rt_noisy_pose',Pose,queue_size=10)
        rate=rospy.Rate(10)		
        pose_sub=rospy.Subscriber('/turtle1/pose',Pose,callback)
            
        while not rospy.is_shutdown():      
            v=2   #velocity
            r=2   #Radius
            circles(v,r)

       
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")