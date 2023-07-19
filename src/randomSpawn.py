#! /usr/bin/env python3
import rospy
import rosservice
import random
from math import pi

#If required, enter custom values to test system
x=random.randrange(11)
y=random.randrange(11)
theta=random.random()*pi

print(x,y,theta)
rospy.wait_for_service('/spawn')
rosservice.call_service("/spawn",[x, y, theta, ""])

