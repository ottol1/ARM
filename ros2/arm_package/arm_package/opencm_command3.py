# Author: Bruce Noble
# Team: Articulated Removable Manipulator (ARM), 2025-2026
# Part of the ARM project is the integration of a force sensor and serial communications for each of the dynamixel servos

import rclpy
from rclpy.node import Node
# from control_msgs.action import FollowJointTrajectory
# from trajectory_msgs.msg import JointTrajectoryPoint
# from rclpy.action import ActionClient
# from rclpy.qos import QoSProfile
from control_msgs.msg import JointTrajectoryControllerState
from sensor_msgs.msg import JointState
import serial
import math

# this should allow us to start opencm_command before or after the joint trajectory has been sent
# pos_deg = [154, 343, 165, 188, 157, 0] # default starting configuration
# vel_rpm = [30, 30, 30, 30, 30, 30] # safe velocity to move at



class opencmCommandNode(Node):
	def __init__(self):
		super().__init__('opencm_command_node')
		
		# start the serial connection
		try:
			self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1) # this needs to be changed based on the device :(
			#self.get_logger().info("Connected to serial /dev/ttyACM0")
		except Exception as e:
			self.get_logger().error(f"Serial Error: {e}")
			raise e
		
		#subscribe to joint_trajectory topic
		self.subscription = self.create_subscription(
			# FollowJointTrajectory.Goal,
			#JointTrajectory
			JointTrajectoryControllerState,
			#'/joint_trajectory', # possible alternatives: /arm_controller/controller_state
			'/arm_controller/controller_state',
			# will also need /gripper_controller/
			#'/arm_controller/joint_trajectory'
			# '/arm_controller/follow_joint_trajectory/_action/goal'
			self.listener_callback,
			10)
		self.publisher = self.create_publisher(
			JointState,
			'/joint_state',
			#self.listener_callback,
			10)
		self.prev_posCommand_int = [2048, 1024, 2048, 614, 614, 0] # initial positions
		self.joint_positions_int = [2048, 1024, 2048, 614, 614, 0]
		self.posCommand_int = [2048, 1024, 2048, 614, 614, 0]
		self.velCommand_int = [131, 131, 131, 273, 273, 273]
		self.joint_tolerance = [11, 3, 102] # joint tolerance of 1 degree (in steps respective to motor) and 0.1 N before processing next command
			
	def vector_compare(self, a, b):
		# Check if the values are within 5 degrees each other
		# if it is not true for a single value, return false
		# if it is true for every value, return true
		for i in range(6):
			if i < 3:
				if (a[i] > (b[i] - self.joint_tolerance[0])) and (a[i] < (b[i] + self.joint_tolerance[0])):
					return True
			elif i < 5:
				if (a[i] > (b[i] - self.joint_tolerance[1])) and (a[i] < (b[i] + self.joint_tolerance[1])):
					return True
			elif i == 5:
				if (a[i] > (b[i] - self.joint_tolerance[2])) and (a[i] < (b[i] + self.joint_tolerance[2])):
					return True
		return False

	def listener_callback(self, msg):
	
		joint_states = JointState()
		joint_positions_rad = [0.0]*6
		# joint_velocities = [0.0]*6
		posCommand_rad = [0.0]*6
		velCommand_rad = [0.0]*6

		# check if the current joint positions match the previous joint command
		# if self.vector_compare(self.prev_posCommand_int, self.joint_positions_int):


			# # get the most recent trajectory point
			# if msg.desired: # if there is a new position and velocity, update it!
			# 	# get target positions and velocity from the first point in the trajectory, could be adapted to cycle through trajectory points?
			# 	# print(f"Recieved: {msg.points[-1]}")
			# 	# trajectory = msg.goal.trajectory.points[-1]
			# 	# print(f"Recieved: {trajectory}")	
			# 	# format commands to send over serial
			# 	# posCommand_rad = list(msg.points[-1].positions)
			# 	# velCommand_rad = list(msg.points[-1].velocities)
			# 	print(f"Recieved: {msg.desired}")	
			# 	posCommand_rad = list(msg.desired.positions)
			# 	velCommand_rad = list(msg.desired.velocities) # leave velocity in rpm
			# 	# posCommand_rad = list(trajectory.positions)
			# 	# velCommand_rad = list(trajectory.velocities)




			# 	# quick math for the wrist joint
			# 	# joint4_pos = posCommand_rad[3] - posCommand_rad[4]
			# 	# joint5_pos = (2*math.pi - posCommand_rad[3]) + posCommand_rad[4]

			# 	# joint4_vel = velCommand_rad[3] - posCommand_rad[4]
			# 	# joint5_vel = -posCommand_rad[3] + velCommand_rad[4]

			# 	# posCommand_rad[3] = joint4_pos
			# 	# posCommand_rad[4] = joint5_pos

			# 	# velCommand_rad[3] = joint4_vel
			# 	# velCommand_rad[4] = joint5_vel
			
			# 	for i in range(6):
			# 		if i < 3:
			# 			# convert to integers based on dynamixel protocol 2
			# 			self.posCommand_int[i] = int((posCommand_rad[i]+math.pi)*4095/math.pi) # 0.088 deg per pulse, in rad | 0 - 4095 pulses
			# 			self.velCommand_int[i] = int(abs(velCommand_rad[i])/0.229) # 0.229 rev/min per pulse | 0 - 2047 pulses # CONVERT FROM RADIANS PER SECOND, NOT RPM
			# 		elif i < 5:
			# 			# convert to integers based on dynamixel protocol 1
			# 			self.posCommand_int[i] = int((posCommand_rad[i]+math.pi)*1023/(math.pi-0.5235987756)) # 0.293 deg per pulse, in rad (only to 300 deg) | 0 - 1023 pulses
			# 			self.velCommand_int[i] = int(abs(velCommand_rad[i])/0.11) # 0.110 rev/min per pulse | 0 - 1023 pulses # CONVERT FROM RADIANS PER SECOND, NOT RPM

			# 	# NEED TO FIGURE OUT WHERE TO PULL THE DESIRED FORCE SENSOR VALUE FROM
			# 	self.posCommand_int[5] = 0
			# 	self.velCommand_int[5] = 273
		
		# get the most recent trajectory point
		if msg.desired: # if there is a new position and velocity, update it!

			print(f"Recieved: {msg.desired}")	
			posCommand_rad = list(msg.desired.positions)
			velCommand_rad = list(msg.desired.velocities) # leave velocity in rpm


			# quick math for the wrist joint
			joint4_pos = posCommand_rad[3] - posCommand_rad[4]
			joint5_pos = (2*math.pi - posCommand_rad[3]) + posCommand_rad[4]

			joint4_vel = velCommand_rad[3] - velCommand_rad[4]
			joint5_vel = -velCommand_rad[3] + velCommand_rad[4]

			posCommand_rad[3] = joint4_pos
			posCommand_rad[4] = joint5_pos

			velCommand_rad[3] = joint4_vel
			velCommand_rad[4] = joint5_vel
		
			for i in range(6):
				if i < 3:
					# convert to integers based on dynamixel protocol 2
					self.posCommand_int[i] = int((posCommand_rad[i]+math.pi)*4095/(2*math.pi)) # 0.088 deg per pulse, in rad | 0 - 4095 pulses
					self.velCommand_int[i] = int((abs(velCommand_rad[i])*60/(2*math.pi))/0.229) + 1 # 0.229 rev/min per pulse | 0 - 2047 pulses # CONVERT FROM RADIANS PER SECOND, NOT RPM
				elif i < 5:
					# convert to integers based on dynamixel protocol 1
					self.posCommand_int[i] = int((posCommand_rad[i]+math.pi)*1023/((2*math.pi)*(300/360))) # 0.293 deg per pulse, in rad (only to 300 deg) | 0 - 1023 pulses
					self.velCommand_int[i] = int((abs(velCommand_rad[i])*60/(2*math.pi))/0.11) + 1 # 0.110 rev/min per pulse | 0 - 1023 pulses # CONVERT FROM RADIANS PER SECOND, NOT RPM

			# NEED TO FIGURE OUT WHERE TO PULL THE DESIRED FORCE SENSOR VALUE FROM
			self.posCommand_int[5] = 0
			self.velCommand_int[5] = 273
			
		
		# format string for broadcast: P1,P2,P3,P4,P5,P6,V1,V2,V3,V4,V5,V6\n
		command = f"{','.join(map(str, self.posCommand_int))},{','.join(map(str, self.velCommand_int))}\n"
		print(f"Command: {command}")

		# try to send command
		try:
			#self.get_logger().info(f"Sending Command: {command}")
			self.ser.write(command.encode('utf-8'))
			self.ser.flush() # wait for transmission to finish
			#self.get_logger().info("Command Sent!")
			
		except Exception as e:
			self.get_logger().error(f"Write failed: {e}")
		
		
		try:
			#self.get_logger().info("Waiting to recieve joint data")
			joint_data = self.ser.readline().decode('utf-8').rstrip().split(',') # convert utf-8 status message into a vector
			#self.get_logger().info(f"Recieved Data: {jointData}")
			print(f"Feedback: {joint_data}")
		
		except Exception as e:
			self.get_logger().error("Failed to recieve joint data: {e}")
		
		joint_states.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
		
		for i in range(6):
			self.joint_positions_int[i] = int(float(joint_data[i]))
			if i < 3:
				joint_positions_rad[i] = float(joint_data[i])*math.pi/4095.0
			# joint_velocities[i] = float(jointData[i+6]) # new system will only recieve positions
			elif i < 5:
				joint_positions_rad[i] = float(joint_data[i])*(math.pi-0.5235987756)/1023.0
			elif i == 5:
				joint_positions_rad[i] = float(joint_data[i])/1023.0
		joint_states.position = joint_positions_rad
		# joint_states.velocity = joint_velocities
		self.publisher.publish(joint_states)
		# print(self.joint_positions_int)

	def destroy_node(self):
		self.ser.close()
		super().destroy_node()
		
def main():
	rclpy.init()
	node = opencmCommandNode()
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	finally:
		node.destroy_node()
		#rclpy.shutdown()
	
if __name__ == '__main__':
	main()
