# Author: Bruce Noble
# Team: Articulated Removable Manipulator (ARM), 2025-2026
# Part of the ARM project is the integration of a force sensor and serial communications for each of the dynamixel servos

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
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
			self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1) # try to make more adaptive to different serial ports
			#self.get_logger().info("Connected to serial /dev/ttyACM0")
		except Exception as e:
			self.get_logger().error(f"Serial Error: {e}")
			raise e
		
		#subscribe to joint_trajectory topic
		self.subscription = self.create_subscription(
			JointTrajectory,
			'/joint_trajectory', # possible alternatives: /arm_controller/controller_state
			# will also need /gripper_controller/
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
				if (a[i] > (b[i] - self.joint_tolerance[0])) & (a[i] < (b[i] + self.joint_tolerance[0])):
					return False
			elif i < 5:
				if (a[i] > (b[i] - self.joint_tolerance[1])) & (a[i] < (b[i] + self.joint_tolerance[1])):
					return False
			elif i == 5:
				if (a[i] > (b[i] - self.joint_tolerance[2])) & (a[i] < (b[i] + self.joint_tolerance[2])):
					return False
		return True

	def listener_callback(self, msg):
	
		joint_states = JointState()
		joint_positions = [0]*6
		# joint_velocities = [0]*6
		posCommand_rad = [0]*6
		velCommand_rad = [0]*6

		# check if the current joint positions match the previous joint command
		if self.vector_compare(self.prev_posCommand_int, self.joint_positions_int):


			# get the most recent trajectory points
			if msg.points: # if there is a new position and velocity, update it!
				# get target positions and velocity from the first point in the trajectory, could be adapted to cycle through trajectory points?
				point = msg.points[0]
			
				# format commands to send over serial
				posCommand_rad = point.positions
				velCommand_rad = point.velocities # leave velocity in rpm
			
				for i, p, v in [len(posCommand_rad), posCommand_rad, velCommand_rad]:
					if i < 3:
						# convert to integers based on dynamixel protocol 2
						self.posCommand_int[i] = int(p*4095/math.pi) # 0.088 deg per pulse, in rad | 0 - 4095 pulses
						self.velCommand_int[i] = int(v/0.229) # 0.229 rev/min per pulse | 0 - 2047 pulses # CONVERT FROM RADIANS PER SECOND, NOT RPM
					elif i < 7:
						# convert to integers based on dynamixel protocol 1
						self.posCommand_int[i] = int(p*1023/(math.pi-0.5235987756)) # 0.293 deg per pulse, in rad (only to 300 deg) | 0 - 1023 pulses
						self.velCommand_int[i] = int(v/0.11) # 0.110 rev/min per pulse | 0 - 1023 pulses # CONVERT FROM RADIANS PER SECOND, NOT RPM
			
		
		# format string for broadcast: $P1,P2,P3,P4,P5,P6,V1,V2,V3,V4,V5,V6
		command = f"${','.join(map(str, self.posCommand_int))},{','.join(map(str, self.velCommand_int))}\n"
		
		# try to send command
		try:
			#self.get_logger().info(f"Sending Command: {command}")
			self.ser.write(command.encode('utf-8'))
			self.ser.flush() # wait for transmission to finish
			#self.get_logger().info("Command Sent!")
			
		except Exception as e:
			self.get_logger().error(f"Write failed: {e}")
		
		
		try:
			# IMPLEMENT self.joint_positions_int
			#self.get_logger().info("Waiting to recieve joint data")
			jointData = self.ser.readline().decode('utf-8').rstrip().split(',') # convert utf-8 status message into a vector
			#self.get_logger().info(f"Recieved Data: {jointData}")
		except Exception as e:
			self.get_logger().error("Failed to recieve joint data: {e}")
		joint_states.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
		for i in range(6):
			joint_positions[i] = float(jointData[i])
			# joint_velocities[i] = float(jointData[i+6]) # new system will only recieve positions
		joint_states.position = joint_positions
		# joint_states.velocity = joint_velocities
		self.publisher.publish(joint_states)

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
