#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
import sys
import select
import termios
import tty
import numpy as np
import math

settings = termios.tcgetattr(sys.stdin)

msg = """
Reading from the keyboard and Publishing to JointTrajectory!
------------------------------------------------------------
Note that the controls for each joint are under the joint numbers on the keyboard (1, 2, 3, 4, 5, 6).
The top key increases the angle and the bottom key decreases the angle.

*All joints have set default speeds for the time being, and only move one joint at a time.
==========================================
Moving joints:
1:	2:	3:	4:	5:	6:
------------------------------------------
q	w	e	r	t	y
a	s	d	f	g	h

CTRL-C to quit
"""

moveBindings = {
	'q':(1,0,0,0,0,0),
	'a':(-1,0,0,0,0,0),
	'w':(0,1,0,0,0,0),
	's':(0,-1,0,0,0,0),
	'e':(0,0,1,0,0,0),
	'd':(0,0,-1,0,0,0),
	'r':(0,0,0,1,0,0),
	'f':(0,0,0,-1,0,0),
	't':(0,0,0,0,1,0),
	'g':(0,0,0,0,-1,0),
	'y':(0,0,0,0,0,1),
	'h':(0,0,0,0,0,-1),
	}
	
def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key
	
def main(args=None):
	rclpy.init(args = args)
	node = rclpy.create_node('teleop_joint_keyboard')
	
	pub = node.create_publisher(JointTrajectory, '/joint_trajectory', 10)
	
	joints = [np.pi]*6
	step = 0.05 # radians per key press
	
	joint_names = [
		"joint1", "joint2", "joint3",
		"joint4", "joint5", "joint6"
		]
		
	print(msg)
	
	try:
		while(1):
			key = getKey()
			if key in moveBindings.keys():
				increments = moveBindings[key]
				for i in range(6):
					joints[i] += increments[i]*step
					
				# build tranectory message
				traj = JointTrajectory()
				traj.joint_names = joint_names
				
				point = JointTrajectoryPoint()
				point.positions = joints
				
				default_velocity = 0.5 # rad/s
				point.velocities = [default_velocity]*6
				
				point.time_from_start.sec = 1
				
				traj.points.append(point)
				
				pub.publish(traj)
				
			elif key == '\x03': # CTRL-C
				break
	except Exception as e:
		print(e)
		
	finally:
		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
		node.destroy_node()
		rclpy.shutdown()
		
if __name__ == '__main__':
	main()

	
	
