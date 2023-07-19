#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import atan2, sqrt, pi
import time


def callback(data):
    global position                                 
    position = data

def step_vel(vf,vi,tf,ti):
    
    K_accel=0.1
    K_decel=0.3
    if(vf.linear.x-vi.linear.x>0):
        k=K_accel
    else: 
        k=K_decel
    step_vel=Twist()
    
    step=k*(vf.linear.x-vi.linear.x)

    stepangular=4*k*(vf.angular.z-vi.angular.z)

    vi.linear.x=vi.linear.x+step
    vi.angular.z=vi.angular.z+stepangular
    
    velocity_pub.publish(vi)
    return (vi)		


def rotate(angle):
    vel=Twist()
    i=0
    while(abs((angle*pi/180)-(position.theta)))>0.05:
        vel.angular.z=(angle*pi/180)-(position.theta)
        velocity_pub.publish(vel)
			

def go_to_goal(goal):
    global position
    vel=Twist()
    Kp=0.5
    Kd=0.01
    Ki=0.0001
    dp=(position.x-goal.x)**2+(position.y-goal.y)**2
    es=0
    vp=Twist()
    ti=0
    while(True):
        es=es+dp
        dist=sqrt((position.x-goal.x)**2+(position.y-goal.y)**2)
        dedt=dist-dp	
        vel.linear.x=Kp*(dist)+Kd*dedt+Ki*es
        vel.angular.z=6*Kp*(atan2(goal.y - position.y, goal.x - position.x)-position.theta)
        
        tf=time.time()
        
        vp=step_vel(vel,vp,tf,ti)
        #vel_pub.publish(vel)				
        ti=tf
        rate.sleep()
        dp=dist

        
        
        if(dist<0.1):
            stop_time=time.time()
            print("Reached goal")
            break

def grid():
    pos=Pose()
    grid_points=[(1,1,0),(10,1,90),(10,3,180),(1,3,90),(1,5,0),(10,5,90),(10,7,180),(1,7,90),(1,9,0),(10,9,0)] #feed Custom Value of X, Y, Theta
    for i in range(len(grid_points)):
        pos.x=grid_points[i][0]
        pos.y=grid_points[i][1]
        go_to_goal(pos)
        rotate(grid_points[i][2])
        #time.sleep(1)


if __name__ == '__main__':
    try:
        
        rospy.init_node('grid_turtle')
        position=Pose()
        velocity_pub=rospy.Publisher('/turtle1/cmd_vel',Twist,queue_size=10)
        rate=rospy.Rate(10)                         # set at 10 Hz
        pose_sub=rospy.Subscriber('/turtle1/pose',Pose,callback)
        
        while not rospy.is_shutdown():
            grid()
       
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")