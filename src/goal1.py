#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import atan2, sqrt


def callback(data):
    global position                                 
    position = data

def go_to_goal():
    global position                                 
    pos=Pose()
    pos.x=int(input("x:"))
    pos.y=int(input("y:"))
    vel=Twist()
    Kp=1
    Kd=0.1
    Ki=0.0001
    dp=sqrt((position.x-pos.x)**2+(position.y-pos.y)**2)
    es=0           										
    ss=0
    SP=(atan2(pos.y - position.y, pos.x - position.x)-position.theta) #Prv Angle Calc
    while(True):
        es=es+dp
        dist=sqrt((position.x-pos.x)**2+(position.y-pos.y)**2)
        dedt=dist-dp	
        vel.linear.x=Kp*(dist)+Kd*dedt+Ki*es							#PID Function
        steer=(atan2(pos.y - position.y, pos.x - position.x)-position.theta)   #Current AngleCalc
        dsteer=steer-SP
        vel.angular.z=6*Kp*steer+Kd*dsteer+Ki*ss						#PID Funct for Ang Vel
        velocity_pub.publish(vel)
        rate.sleep()
        dp=dist
        SP=steer
        ss=ss+SP
        if(dist<0.5):
            print("Reached goal")
            break




if __name__ == '__main__':
    try:
        
        rospy.init_node('move_to_goal')
        position=Pose()
        velocity_pub=rospy.Publisher('/turtle1/cmd_vel',Twist,queue_size=10)
        rate=rospy.Rate(10)                         # publish rate 10 msgs/s
        pose_sub=rospy.Subscriber('/turtle1/pose',Pose,callback)
        
        while not rospy.is_shutdown():
            go_to_goal()
       
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")