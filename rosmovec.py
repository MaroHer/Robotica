#! /usr/bin/env python
#Esto es para el cuadrado

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import time
from std_srvs.srv import Empty

pi=3.141592
x = 0
y = 0
z = 0
theta = 0

def poseCallback(pose_message):
    global x
    global y
    global z
    global theta
    
    x = pose_message.x
    y = pose_message.y
    theta = pose_message.theta

def orientate (xgoal, ygoal):
    global x
    global y
    global theta

    velocity_message = Twist()
    cmd_vel_topic = '/turtle1/cmd_vel'

    while(True):
        ka = 4.0
        desired_angle_goal = math.atan2(ygoal-y, xgoal-x)
	dtheta = desired_angle_goal-theta 
	if(dtheta > 0.0):
		dtheta=dtheta
	else:
		print ('--------------------------------------------')
		if(abs(dtheta)> ((2*math.pi)+dtheta)):
			dtheta= (2*math.pi ) +dtheta 

	angular_speed = ka * (dtheta)
        velocity_message.linear.x = 0.0
        velocity_message.angular.z = angular_speed
        velocity_publisher.publish(velocity_message)
        print ('x=', x, 'y=', y)

        if (dtheta < 0.01):
            break

def go_to_goal (xgoal, ygoal):
    global x
    global y
    global theta

    velocity_message = Twist()
    cmd_vel_topic = '/turtle1/cmd_vel'

    while(True):
        kv = 0.5				
        distance = abs(math.sqrt(((xgoal-x)**2)+((ygoal-y)**2)))
        linear_speed = kv * distance

        ka = 4.0
        desired_angle_goal = math.atan2(ygoal-y, xgoal-x)
	dtheta = desired_angle_goal-theta 
	if(dtheta > 0.0):
		dtheta=dtheta
	else:
		print ('--------------------------------------------')
		if(abs(dtheta)> ((2*math.pi)+dtheta)):
			dtheta= (2*math.pi ) +dtheta 

	angular_speed = ka * (dtheta)
	#print('dtheta: ',dtheta)
	#print('distance: ',distance)


        velocity_message.linear.x = linear_speed
        velocity_message.angular.z = angular_speed
        velocity_publisher.publish(velocity_message)
        print ('x=', x, 'y=', y)

        if (distance < 0.01):
            break

if __name__ == '__main__':
    try:
	
        rospy.init_node('turtlesim_motion_pose', anonymous = True)

        cmd_vel_topic = '/turtle1/cmd_vel'
        velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size = 10)

        position_topic = "/turtle1/pose"
        pose_subscriber = rospy.Subscriber(position_topic, Pose, poseCallback)
        time.sleep(2)     

	#time.sleep(2.0)
	for i in range(360):
		s=math.radians(i)
		#El radio es 4
		rx=(math.cos(s) * 4) +5.5
		ry=(math.sin(s) * 4) +5.5
		go_to_goal(rx,ry)
		#time.sleep(1.0)	

	time.sleep(1.0)
	

    except rospy.ROSInterruptException:        
	pass
