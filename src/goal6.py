#! /usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import atan2, sqrt, pi, sin, cos, degrees, radians, ceil
import time
import numpy as np
i=0
a=[]

	
def callback(data):
	position=data

def step_vel(v_final,v_initial,time_sent):
	K_accel=0.1
	K_decel=0.3
	max_accel_linear=1500   
	max_decel_linear=-1500
	max_accel_angular=6000
	max_decel_angular=-6000

	if(v_final.linear.x-v_initial.linear.x>0):  #Setting constants according to acceleration or deceleration situation
		k=K_accel
		max_steps_linear=max_accel_linear
		max_steps_angular=max_accel_angular
	else: 
		k=K_decel
		max_steps_linear=max_decel_linear
		max_steps_angular=max_decel_angular

	step_vel=Twist()
	
	step=k*(v_final.linear.x-v_initial.linear.x)				#Incrementing in steps to keep accel in limit
	t_now=time.time()
	dt=t_now-time_sent
	if(abs(step/dt)>abs(max_steps_linear)):
		
		step=dt*max_steps_linear				#set step corresponding to max accel
		
	stepangular=4*k*(v_final.angular.z-v_initial.angular.z)			#Incrementing in steps to keep accel in limit
	if(abs(stepangular/dt)>abs(max_steps_angular)):
		stepangular=dt*max_steps_angular			#set step corresponding to max accel
		
	v_initial.linear.x=v_initial.linear.x+step
	v_initial.angular.z=v_initial.angular.z+stepangular
	if(v_initial.linear.x>v_max):
		v_initial.linear.x=v_max
		
	vel_pub.publish(v_initial)
	timer=time.time()
	
	return(v_initial,timer)				

#This finds distance between self and a tuple	
def distance(point):
	return(sqrt((position.x-point[0])**2+(position.y-point[1])**2))

#This finds distance between self and goal pose	
def euclidean_distance(goal):
	return(sqrt((position.x-goal.x)**2+(position.y-goal.y)**2))	


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
		vel.linear.x=Kp*(dist)+Kd*dedt+Ki*es			#PID Implementation
		vel.angular.z=6*Kp*(atan2(goal.y - position.y, goal.x - position.x)-position.theta)
		vel_prev,tf=step_vel(vel,vel_prev,tf)
		#vel_pub.publish(vel)				
		ti=tf
		rate.sleep()
		dp=dist

		if(dist<0.1):
			stop_time=time.time()
			print("Reached goal")
			break
	print("Total time taken:",start_time-stop_time)
	
#This function plans the future points, and moves to them.	
def plan(target):
	predicted_pos=Pose()
	if(np.shape(a)[0]<3):
		a.append([target.x,target.y])		
	print(a)
	if(np.shape(a)[0]>=3):			#Need minimum of 3 points to define circle	
		center,radius=define_circle(a[0],a[1],a[2])
		next_point=next_point(a[1],a[2],center,radius)		#Get position of RT in next 5 seconds 
		a.append(next_point)
		#print("next_point:",next_point)
		plan_length=1
		while(True):		#Loop until reachable point is found
			if(distance(next_point)/v_max>(5*plan_length)):			#check if target is reachable in timeslot
				next_point=next_point(a[-2],a[-1],center,radius)
				a.append(next_point)
				plan_length=plan_length+1
				
			else:
				predicted_pos.x=next_point[0]
				predicted_pos.y=next_point[1]
				go_to_goal(predicted_pos)
				break
				
#This function takes 3 points and finds circle passing all of them, then returning its radius and center	
def define_circle(p1, p2, p3):
	temp = p2[0] * p2[0] + p2[1] * p2[1]
	bc = (p1[0] * p1[0] + p1[1] * p1[1] - temp) / 2
	cd = (temp - p3[0] * p3[0] - p3[1] * p3[1]) / 2
	det = (p1[0] - p2[0]) * (p2[1] - p3[1]) - (p2[0] - p3[0]) * (p1[1] - p2[1])

	if abs(det) < 1.0e-6:
		return (None, np.inf)

	# Center of circle
	cx = (bc*(p2[1] - p3[1]) - cd*(p1[1] - p2[1])) / det
	cy = ((p1[0] - p2[0]) * cd - (p2[0] - p3[0]) * bc) / det

	radius = np.sqrt((cx - p1[0])**2 + (cy - p1[1])**2)
	return ((cx, cy), radius)
#Calculate next point in series using previous two points	
def next_point(p2,p3,center,radius):
	theta2=atan2(p2[1]-center[1],p2[0]-center[0])
	theta3=atan2(p3[1]-center[1],p3[0]-center[0])
	dtheta=degrees(theta2-theta3)
	theta4=degrees(theta3)-dtheta
	nextval=([0,0])
	if(theta4<0):
		theta4=theta4+360
	nextval[0]=center[0]+radius*cos(radians(theta4))
	nextval[1]=center[1]+radius*sin(radians(theta4))
	return (nextval)

#This function recieves true position and keeps checking if gap is close enough to end chase
def actual_gap(pos):
	if(euclidean_distance(pos)<3):
		print('My Position:',position.x,position.y)
		print('Target Position:',pos.x,pos.y)
		print("Distance less than 3. Caught RT\n")
		rt_pose.unregister()
		rospy.set_param('caughtStatus',True)

	
			
if __name__ == '__main__':
    try:
        rospy.init_node('chase_turtle_noisy')
        position=Pose()
        v_max=1				#Setting v_max
        vel_pub=rospy.Publisher('/turtle2/cmd_vel',Twist,queue_size=10)
        rt_pose=rospy.Subscriber('/rt_real_pose',Pose,actual_gap)
        rt_noise_pose=rospy.Subscriber('/rt_noisy_pose',Pose,plan)
        rate=rospy.Rate(8)
        pose_sub=rospy.Subscriber('/turtle2/pose',Pose,callback)
        time.sleep(5)
        rospy.spin()

        
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")
