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
			self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1) # this needs to be changed based on the device :( - austin has this that we can try using
			#self.get_logger().info("Connected to serial /dev/ttyACM0")
		except Exception as e:
			self.get_logger().error(f"Serial Error: {e}")
			raise e
		
		# subscribe to the controller topics
		self.arm_subscription = self.create_subscription(
			# FollowJointTrajectory.Goal,
			#JointTrajectory
			JointTrajectoryControllerState,
			#'/joint_trajectory', # possible alternatives: /arm_controller/controller_state
			'/arm_controller/controller_state',
			# will also need /gripper_controller/
			#'/arm_controller/joint_trajectory'
			# '/arm_controller/follow_joint_trajectory/_action/goal'
			self.arm_listener,
			10)
		self.gripper_subscription = self.create_subscription(
			JointTrajectoryControllerState,
			'gripper_controller/controller_state',
			self.gripper_listener,
			10)
		
		# publish to joint_state topic
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
		self.prev_posCommand_rad = [0.0]*6
		self.prev_velCommand_rad = [0.0]*6
		self.posCommand_rad = [0.0]*6
		self.velCommand_rad = [0.0]*6

		self.timer = self.create_timer(0.02, self.listener_callback) # needed to spin the node
		
	
	def gripper_listener(self, msg):
		if msg.desired:
			print(f"Gripper msg recieved: {msg.desired}")
			self.posCommand_rad[5] = list(msg.desired.positions)
			self.velCommand_rad[5] = list(msg.desired.velocities)
			

	def arm_listener(self, msg):
		if msg.desired:
			print(f"Arm msg recieved: {msg.desired}")
			arm_posCommand_rad = list(msg.desired.positions)
			arm_velCommand_rad = list(msg.desired.velocities)
			for i in range(5):
				self.posCommand_rad[i] = arm_posCommand_rad[i]
				self.velCommand_rad[i] = arm_velCommand_rad[i]
		

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

	# convert wrist attitude and rotation to joint
	def wrist_math(self):
		# quick math for the wrist joint
		wristAtt_pos = self.posCommand_rad[3]
		wristRot_pos = self.posCommand_rad[4]
		wristAtt_vel = self.velCommand_rad[3]
		wristRot_vel = self.velCommand_rad[4]

		servo4_pos = 0.0
		servo5_pos = 0.0
		servo4_vel = 0.0
		servo5_vel = 0.0

		if wristAtt_pos > self.prev_posCommand_rad[3]:  # if wrist angle is increasing
			servo4_pos += wristAtt_pos - self.prev_posCommand_rad[3]
			servo5_pos -= wristAtt_pos - self.prev_posCommand_rad[3]
		if wristAtt_pos < self.prev_posCommand_rad[3]:  # if wrist angle is decreasing
			servo4_pos -= self.prev_posCommand_rad[3] - wristAtt_pos
			servo5_pos += self.prev_posCommand_rad[3] - wristAtt_pos
		if wristRot_pos > self.prev_posCommand_rad[4]:  # if frame angle is increasing
			servo4_pos -= wristRot_pos - self.prev_posCommand_rad[4]
			servo5_pos -= wristRot_pos - self.prev_posCommand_rad[4]
		if wristRot_pos < self.prev_posCommand_rad[4]:  # if frame angle is decreasing
			servo4_pos += self.prev_posCommand_rad[4] - wristRot_pos
			servo5_pos += self.prev_posCommand_rad[4] - wristRot_pos

		if wristAtt_vel > self.prev_velCommand_rad[3]:  # if wrist angle is increasing
			servo4_vel += wristAtt_vel - self.prev_velCommand_rad[3]
			servo5_vel -= wristAtt_vel - self.prev_velCommand_rad[3]
		if wristAtt_vel < self.prev_velCommand_rad[3]:  # if wrist angle is decreasing
			servo4_vel -= self.prev_velCommand_rad[3] - wristAtt_vel
			servo5_vel += self.prev_velCommand_rad[3] - wristAtt_vel
		if wristRot_vel > self.prev_velCommand_rad[4]:  # if frame angle is increasing
			servo4_vel -= wristRot_vel - self.prev_velCommand_rad[4]
			servo5_vel -= wristRot_vel - self.prev_velCommand_rad[4]
		if wristRot_vel < self.prev_velCommand_rad[4]:  # if frame angle is decreasing
			servo4_vel += self.prev_velCommand_rad[4] - wristRot_vel
			servo5_vel += self.prev_velCommand_rad[4] - wristRot_vel

		self.posCommand_rad[3] = servo4_pos
		self.posCommand_rad[4] = servo5_pos
		self.velCommand_rad[3] = servo4_vel
		self.velCommand_rad[4] = servo5_vel

		
	def rad_to_int(self):
		# convert to integers for dynamixel to use
		for i in range(6):
			if i < 3:
				# convert to integers based on dynamixel protocol 2
				self.posCommand_int[i] = int((self.posCommand_rad[i]+math.pi)*4095/(2*math.pi)) # 0.088 deg per pulse, in rad | 0 - 4095 pulses
				self.velCommand_int[i] = int((abs(self.velCommand_rad[i])*60/(2*math.pi))/0.229) + 1 # 0.229 rev/min per pulse | 0 - 2047 pulses
			elif i < 5:
				# convert to integers based on dynamixel protocol 1
				self.posCommand_int[i] = int((self.posCommand_rad[i]+math.pi)*1023/((2*math.pi)*(300/360))) # 0.293 deg per pulse, in rad (only to 300 deg) | 0 - 1023 pulses
				self.velCommand_int[i] = int((abs(self.velCommand_rad[i])*60/(2*math.pi))/0.11) + 1 # 0.110 rev/min per pulse | 0 - 1023 pulses
			elif i == 5:
				# force sensor works a bit differently
				self.posCommand_int[5] = self.posCommand_rad[5]*1023 # up to 1 N
				self.velCommand_int[5] = int((abs(self.velCommand_rad[5])*60/(2*math.pi))/0.229) + 1 # 0.229 rev/min per pulse | 0 - 2047 pulses


	def listener_callback(self):
		
		joint_states = JointState()
		joint_positions_rad = [0.0]*6
		# joint_velocities_rad = [0.0]*6


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

			# posCommand_rad[3], posCommand_rad[4], velCommand_rad[3], velCommand_rad[4] = self.wrist_math(posCommand_rad[3], posCommand_rad[4], velCommand_rad[3], velCommand_rad[4])
			
			# self.rad_to_int(posCommand_rad, velCommand_rad)
		
		# get the most recent trajectory point
		# if msg.desired: # if there is a new position and velocity, update it!

		# 	print(f"Recieved: {msg.desired}")	
		# 	posCommand_rad = list(msg.desired.positions)
		# 	velCommand_rad = list(msg.desired.velocities) # leave velocity in rpm

		# 	# quick math for the wrist joint
		# 	posCommand_rad[3], posCommand_rad[4], velCommand_rad[3], velCommand_rad[4] = self.wrist_math(posCommand_rad[3], posCommand_rad[4], velCommand_rad[3], velCommand_rad[4])
			
		# 	#convert to integers, and save to self variables
		# 	self.rad_to_int(posCommand_rad, velCommand_rad)

		# print("Starting arm_listener")
		# self.arm_listener() # should be able to listen to both topics now
		# print("Exited arm_listener, starting gripper_listener")
		# self.gripper_listener()	
		# print("Exited gripper_listener, starting wrist_math")
		print("Starting wrist_math")
		self.wrist_math()
		print("Exited wrist_math, starting rad_to_int")
			
		#convert to integers, and save to self variables
		self.rad_to_int()
		print("Exited rad_to_int")
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

		self.prev_posCommand_rad = self.posCommand_rad
		self.prev_velCommand_rad = self.velCommand_rad

	def destroy_node(self):
		self.ser.close()
		super().destroy_node()
		
def main():
	print("Entered main")
	rclpy.init()
	node = opencmCommandNode()
	print("Assigned node")
	try:
		print("Trying to spin node")
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	finally:
		node.destroy_node()
		#rclpy.shutdown()
	
if __name__ == '__main__':
	main()
