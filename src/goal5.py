#! /usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import atan2, sqrt, pi
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
    v_max=1					#Setting max velocity as 1units/s
    if(vf.linear.x-vi.linear.x>0):  #Setting constants according to acceleration or deceleration situation
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
        
    
    stepangular=4*k*(vf.angular.z-vi.angular.z)		#Incrementing in steps to keep accel in limit. Can be faster than linear component
    if(abs(stepangular/dt)>abs(max_steps_angular)):
        stepangular=dt*max_steps_angular			#set step corresponding to max accel

    vi.linear.x=vi.linear.x+step
    vi.angular.z=vi.angular.z+stepangular
    if(vi.linear.x>v_max):				#velocity should not exceed v_max
        vi.linear.x=v_max
        
    vel_pub.publish(vi)
    timer=time.time()
    return(vi,timer)				


def go_to_goal(goal):	
    start_time=time.time()
    vel=Twist()
    Kp=1
    Kd=0.1
    Ki=0.0001
    dp=sqrt((position.x-goal.x)**2+(position.y-goal.y)**2)
    es=0					#Error Terms	
    vel_prev=Twist()
    tf=time.time()
    while(True):
        es=es+dp
        dist=sqrt((position.x-goal.x)**2+(position.y-goal.y)**2)
        dedt=dist-dp	
        vel.linear.x=Kp*(dist)+Kd*dedt+Ki*es		#PID Implementation
        vel.angular.z=6*Kp*(atan2(goal.y - position.y, goal.x - position.x)-position.theta)
        
        
        
        vel_prev,tf=step_vel(vel,vel_prev,tf)
        ti=tf
        rate.sleep()
        dp=dist

        if(dist<0.1):
            stop_time=time.time()
            print("Reached goal")
            break
    
#This function is called on receiving rt_real_pose. It checks distance. If not in range, goes towards target.
def chase(target):
    
    dist=sqrt((position.x-target.x)**2+(position.y-target.y)**2)
    print("Current distance:",dist)
    if(dist<3):
        print('My Position:',position.x,position.y)
        print('Target Position:',target.x,target.y)
        print("Distance less than 3. Caught RT\n")
        rt_pose.unregister()
        rospy.set_param('caughtStatus',True)
    else:
        go_to_goal(target)


			




if __name__ == '__main__':
    try:
    
        rospy.init_node('chaser_turtle_velocity')
        position=Pose()
        vel_pub=rospy.Publisher('/turtle2/cmd_vel',Twist,queue_size=10)
        rt_pose=rospy.Subscriber('/rt_real_pose',Pose,chase)
        rate=rospy.Rate(8)
        pose_sub=rospy.Subscriber('/turtle2/pose',Pose,callback)
        time.sleep(10)
        rospy.spin()


        
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")