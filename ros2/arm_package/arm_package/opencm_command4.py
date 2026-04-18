# Author: Bruce Noble
# Team: Articulated Removable Manipulator (ARM), 2025-2026
# Part of the ARM project is the integration of a force sensor and serial communications for each of the dynamixel servos

import rclpy
from rclpy.node import Node
from control_msgs.msg import JointTrajectoryControllerState
from sensor_msgs.msg import JointState
import serial
import math
import subprocess


class opencmCommandNode(Node):
	def __init__(self):
		super().__init__('opencm_command_node')

		port = "/dev/ttyACM0"
		grep = subprocess.run("udevadm info -q property /dev/ttyACM0 | grep 'ID_MODEL_ID='", shell=True,
							  capture_output=True, text=True)
		if grep.returncode != 0:
			self.get_logger().error(f"No ARM found at {port}")

		model_id_split = grep.stdout.strip().split("=")
		if len(model_id_split) != 2:
			self.get_logger().error(f"udevadm or grep did something weird: {grep.stdout}")

		if model_id_split[1] != "ff48":  # This means ODD is at /ttyACM0, probably
			port = "/dev/ttyACM1"  # So ARM should be here.

		# start the serial connection
		try:
			self.ser = serial.Serial(port, 115200, timeout=1) # this needs to be changed based on the device :( - austin has this that we can try using
			self.get_logger().info(f"Connected to ARM at {port}")
		except Exception as e:
			self.get_logger().error(f"Serial Error: {e}")
			raise e

		# subscribe to the controller topics
		self.arm_subscription = self.create_subscription(
			JointTrajectoryControllerState,
			'/arm_controller/controller_state',
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
			'/joint_states', # consider /rviz/moveit/update_custom_goal_state/joint_state
			10)
		
		self.posCommand_int = [2048, 1024, 2048, 614, 614, 0]
		self.velCommand_int = [131, 131, 131, 273, 273, 1297]
		self.posCommand_rad = [0.0]*6
		self.velCommand_rad = [0.0]*6
		self.prev_posCommand_rad = [0.0]*6
		self.prev_velCommand_rad = [0.0]*6

		self.timer = self.create_timer(0.02, self.listener_callback) # needed to spin the node
		
	
	def gripper_listener(self, msg):
		# check for new gripper messages
		if msg.desired:
			print(f"Gripper msg received: {msg.desired}")
			self.posCommand_rad[5] = list(msg.desired.positions)[0]
			self.velCommand_rad[5] = list(msg.desired.velocities)[0]
			

	def arm_listener(self, msg):
		# check for new arm messages
		if msg.desired:
			print(f"Arm msg received: {msg.desired}")
			arm_posCommand_rad = list(msg.desired.positions)
			arm_velCommand_rad = list(msg.desired.velocities)
			for i in range(5):
				self.posCommand_rad[i] = arm_posCommand_rad[i]
				self.velCommand_rad[i] = arm_velCommand_rad[i]
			self.wrist_math()
		

	# convert wrist attitude and rotation to joint
	def wrist_math(self):
		# quick math for the wrist joint
		wristAtt_pos = self.posCommand_rad[3]
		wristRot_pos = self.posCommand_rad[4]
		wristAtt_vel = self.velCommand_rad[3]
		wristRot_vel = self.velCommand_rad[4]

		servo4_pos = wristAtt_pos - wristRot_pos
		servo5_pos = -wristAtt_pos - wristRot_pos

		servo4_vel = (-wristAtt_vel - wristRot_vel)/2
		servo5_vel = (wristAtt_vel + wristRot_vel)/2

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
				if self.posCommand_rad[5] != 0:
					self.posCommand_int[5] = 1022 # up to ~1 N for now
				else:
					self.posCommand_int[5] = 0 # just open for now
				self.velCommand_int[5] = int((abs(self.velCommand_rad[5])*60/(2*math.pi))/0.229) + 1025 # 0.229 rev/min per pulse | 1025 - 2047 pulses for reversed direction

	def int_to_rad(self, joint_data):
		joint_positions_rad = [0.0]*6
		joint_velocities_rad = [0.0]*6
		for i in range(6):
			if i < 3:
				joint_positions_rad[i] = float(joint_data[i])*2*math.pi/4095 - math.pi
				joint_velocities_rad[i] = float(joint_data[i+6])*(2*math.pi*0.229)/60
			elif i < 5:
				joint_positions_rad[i] = float(joint_data[i])*((2*math.pi)*(300/360))/1023 - math.pi
				joint_velocities_rad[i] = float(joint_data[i+6])*(2*math.pi*0.11)/60
			elif i == 5:
				joint_positions_rad[i] = float(joint_data[i])/1023.0
				joint_velocities_rad[i] = float(joint_data[i+6])*(2*math.pi*0.11)/60
		
		return joint_positions_rad, joint_velocities_rad


	def listener_callback(self):
		
		joint_states = JointState()
		joint_positions_rad = [0.0]*6
		joint_velocities_rad = [0.0]*6

			
		#convert to integers, and save to self variables
		self.rad_to_int()
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
			#self.get_logger().info("Waiting to receive joint data")
			joint_data = self.ser.readline().decode('utf-8').rstrip().split(',') # convert utf-8 status message into a vector
			#self.get_logger().info(f"Received Data: {jointData}")
			print(f"Feedback: {joint_data}")
		
		except Exception as e:
			self.get_logger().error("Failed to receive joint data: {e}")
		
		if joint_data != ['']:
			joint_states.name = ['shoulder_joint', 'upperarm_joint', 'forearm_joint', 'wrist_joint', 'frame_joint', 'worm_joint']

			joint_positions_rad, joint_velocities_rad = self.int_to_rad(joint_data)

			joint_states.position = joint_positions_rad
			joint_states.velocity = joint_velocities_rad
			self.publisher.publish(joint_states)
			# print(self.joint_positions_int)

			self.prev_posCommand_rad = self.posCommand_rad
			self.prev_velCommand_rad = self.velCommand_rad


	def destroy_node(self):
		self.ser.close()
		super().destroy_node()
		
def main():
	# print("Entered main")
	rclpy.init()
	node = opencmCommandNode()
	# print("Assigned node")
	try:
		# print("Trying to spin node")
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	finally:
		node.destroy_node()
	
if __name__ == '__main__':
	main()
