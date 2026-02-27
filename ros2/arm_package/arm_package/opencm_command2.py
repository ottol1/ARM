# Author: Bruce Noble
# opencm_command2 is a component of the arm_package. This script subscribes to the joint_trajectory
# ROS2 topic, actuates the dynamixel servos, and publishes to the joint_state topic.
# https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/sample_code/python_protocol_combined/#python-protocol-combined

# ----- IMPORTS -----
# ROS2
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState
import math

# DYNAMIXEL
import os
import sys

if os.name == 'nt':
	import msvcrt
	def getch():
		return msvcrt.getch().decode()
else:
	import tty
	import termios
	fd = sys.stdin.fileno()
	old_settings = termios.tcgetattr(fd)
	tty.setraw(sys.stdin.fileno())
	def getch():
		return sys.stdin.read(1)
		
os.sys.path.append('../dynamixel_functions_py') # path setting

import dynamixel_functions as dynamixel # uses dynamixel sdk library

# ----- CLASS DEFINITION -----
class opencmCommandNode(Node):

	def __init__(self):
		super().__init__('opencm_command_node')
		
		# initial position and velocities
		self.pos_deg = [154, 343, 165, 188, 157, 0]
		self.vel_rpm = [30, 30, 30, 30, 30, 30]
		
		# ROS2
		# subscribe to joint_trajectory topic
		self.sub = self.create_subscription(
			JointTrajectory,
			'/joint_trajectory',
			self.listener_callback,
			10)
			
		# publish to joint_state topic
		self.pub = self.create_publisher(
			JointState,
			'/joint_state',
			10)
			
		# DYNAMIXEL
		# constrol table address for dynamixel MX
		self.ADDR_MX_TORQUE_ENABLE = 24
		self.ADDR_MX_GOAL_POSITION = 30
		self.ADDR_MX_MAX_VELOCITY = 32
		self.ADDR_MX_PRESENT_POSITION = 36
		self.ADDR_MX_PRESENT_VELOCITY = 38
		
		# control table address for dynamixel PRO
		self.ADDR_PRO_TORQUE_ENABLE = 562
		self.ADDR_PRO_GOAL_POSITION = 596
		self.ADDR_PRO_MAX_VELOCITY = 112
		self.ADDR_PRO_PRESENT_POSITION = 611
		self.ADDR_PRO_PRESENT_VELOCITY = 615
		
		# protocol version
		self.PROTOCOL_VERSION1 = 1
		self.PROTOCOL_VERSION2 = 2
		
		# default setting
		self.DXL1_ID = 1
		self.DXL2_ID = 2
		self.DXL3_ID = 3
		self.DXL4_ID = 4
		self.DXL5_ID = 5
		self.DXL6_ID = 6
		self.BAUDRATE = 1000000
		self.DEVICENAME = "/dev/ttyUSB0".encode('utf-8') # check which port you need to use
		
		self.TORQUE_ENABLE = 1
		self.TORQUE_DISABLE = 0
		self.DXL1_MINIMUM_POSITION_VALUE = 100 # what are the units for these?
		self.DXL1_MAXIMUM_POSITION_VALUE = 4000
		self.DXL2_MINIMUM_POSITION_VALUE = -150000
		self.DXL2_MAXIMUM_POSITION_VALUE = 150000
		self.DXL1_MOVING_STATUS_THRESHOLD = 10
		self.DXL2_MOVING_STATUS_THRESHOLD = 20
		
		self.ESC_ASCII_VALUE = 0x1b
		
		self.COMM_SUCCESS = 0
		self.COMM_TX_FAIL = -1001
		
		# initialize port handler structure
		self.port_num = self.dynamixel.portHandler(self.DEVICENAME)
		
		# initialize packet handler structure
		self.dynamixel.packetHandler()
		
		self.index = 0
		self.dxl_comm_results = self.COMM_TX_FAIL
		self.dxl1_goal_position = [self.DXL1_MINIMUM_POSITION_VALUE, self.DXL1_MAXIMUM_POSITION_VALUE]
		self.dxl2_goal_position = [self.DXL2_MINIMUM_POSITION_VALUE, self.DXL2_MAXIMUM_POSITION_VALUE]
		self.dxl_error = 0
		self.dxl1_present_position = 0
		self.dxl2_present_position = 0
		
		# Open port
		if dynamixel.openPort(port_num):
			self.get_logger().info("Opened U2D2 Port")
		else:
			self.get_logger().error("Failed to Open U2D2 Port")
			print("Failed to open the port!")
			print("Press any key to terminate...")
			getch()
			quit()


		# Set port baudrate
		if dynamixel.setBaudRate(port_num, BAUDRATE):
			self.get_logger().info(f"Set Baudrate {self.BAUDRATE}")
		else:
			self.get_logger().error("Failed to set Baudrate")
			print("Failed to change the baudrate!")
			print("Press any key to terminate...")
			getch()
			quit()
			
			
		# protocol 2 servos
		# enable dynamixel 1 torque
		self.dynamixel.write1ByteTxRx(self.port_num, self.PROTOCOL_VERSION2, self.DXL1_ID, self.ADDR_PRO_TORQUE_ENABLE, self.TORQUE_ENABLE)
		if self.dynamixel.getLastTxRexResult(self.port_num, self.PROTOCOL_VERSION2) != COMM_SUCCESS:
			self.dynamixel.printTxRxResult(self.PROTOCOL_VERSION2, self.dynamixel.getLastTxRxResult(self.port_num, self.PROTOCOL_VERSION2))
		elif  self.dynamixel.getLastRxPacketError(self.port_num, self.PROTOCOL_VERSION2) != 0:
			self.dynamixel.printRxPacketError(self.PROTOCOL_VERSION2, self.dynamixel.getLastRxPackerError(self.port_num, self.PROTOCOL_VERSION2))
		else:
			print(f"Dynamixel {self.DXL1_ID} has been successfully connected")
		
		# enable dynamixel 2 torque
		self.dynamixel.write1ByteTxRx(self.port_num, self.PROTOCOL_VERSION2, self.DXL2_ID, self.ADDR_PRO_TORQUE_ENABLE, self.TORQUE_ENABLE)
		if self.dynamixel.getLastTxRexResult(self.port_num, self.PROTOCOL_VERSION2) != COMM_SUCCESS:
			self.dynamixel.printTxRxResult(self.PROTOCOL_VERSION2, self.dynamixel.getLastTxRxResult(self.port_num, self.PROTOCOL_VERSION2))
		elif  self.dynamixel.getLastRxPacketError(self.port_num, self.PROTOCOL_VERSION2) != 0:
			self.dynamixel.printRxPacketError(self.PROTOCOL_VERSION2, self.dynamixel.getLastRxPackerError(self.port_num, self.PROTOCOL_VERSION2))
		else:
			print(f"Dynamixel {self.DXL1_ID} has been successfully connected")
		
		# enable dynamixel 3 torque
		self.dynamixel.write1ByteTxRx(self.port_num, self.PROTOCOL_VERSION2, self.DXL3_ID, self.ADDR_PRO_TORQUE_ENABLE, self.TORQUE_ENABLE)
		if self.dynamixel.getLastTxRexResult(self.port_num, self.PROTOCOL_VERSION2) != COMM_SUCCESS:
			self.dynamixel.printTxRxResult(self.PROTOCOL_VERSION2, self.dynamixel.getLastTxRxResult(self.port_num, self.PROTOCOL_VERSION2))
		elif  self.dynamixel.getLastRxPacketError(self.port_num, self.PROTOCOL_VERSION2) != 0:
			self.dynamixel.printRxPacketError(self.PROTOCOL_VERSION2, self.dynamixel.getLastRxPackerError(self.port_num, self.PROTOCOL_VERSION2))
		else:
			print(f"Dynamixel {self.DXL1_ID} has been successfully connected")
		

		
		# protocol 1 servos
		# enable dynamixel 4 torque
		self.dynamixel.write1ByteTxRx(self.port_num, self.PROTOCOL_VERSION1, self.DXL4_ID, self.ADDR_MX_TORQUE_ENABLE, self.TORQUE_ENABLE)
		if self.dynamixel.getLastTxRxResult(self.port_num, self.PROTOCOL_VERSION1) != self.COMM_SUCCESS:
			self.dynamixel.printTxRxResult(self.PROTOCOL_VERSION1, self.dynamixel.getLastTxRxResult(self.port_num, self.PROTOCOL_VERSION1))
		elif self.dynamixel.getLastRxPacketError(self.port_num, self.PROTOCOL_VERSION1) != 0:
			self.dynamixel.printRxPackerError(self.PROTOCOL_VERSION1, self.dynamixel.getLastRxPacketError(self.port_num, self.PROTOCOL_VERSION1))
		else:
			print(f"Dynamixel {self.DXL4_ID} has been successfully connected")
			
		# enable dynamixel 5 torque
		self.dynamixel.write1ByteTxRx(self.port_num, self.PROTOCOL_VERSION1, self.DXL5_ID, self.ADDR_MX_TORQUE_ENABLE, self.TORQUE_ENABLE)
		if self.dynamixel.getLastTxRxResult(self.port_num, self.PROTOCOL_VERSION1) != self.COMM_SUCCESS:
			self.dynamixel.printTxRxResult(self.PROTOCOL_VERSION1, self.dynamixel.getLastTxRxResult(self.port_num, self.PROTOCOL_VERSION1))
		elif self.dynamixel.getLastRxPacketError(self.port_num, self.PROTOCOL_VERSION1) != 0:
			self.dynamixel.printRxPackerError(self.PROTOCOL_VERSION1, self.dynamixel.getLastRxPacketError(self.port_num, self.PROTOCOL_VERSION1))
		else:
			print(f"Dynamixel {self.DXL5_ID} has been successfully connected")

		# enable dynamixel 6 torque
		self.dynamixel.write1ByteTxRx(self.port_num, self.PROTOCOL_VERSION1, self.DXL6_ID, self.ADDR_MX_TORQUE_ENABLE, self.TORQUE_ENABLE)
		if self.dynamixel.getLastTxRxResult(self.port_num, self.PROTOCOL_VERSION1) != self.COMM_SUCCESS:
			self.dynamixel.printTxRxResult(self.PROTOCOL_VERSION1, self.dynamixel.getLastTxRxResult(self.port_num, self.PROTOCOL_VERSION1))
		elif self.dynamixel.getLastRxPacketError(self.port_num, self.PROTOCOL_VERSION1) != 0:
			self.dynamixel.printRxPackerError(self.PROTOCOL_VERSION1, self.dynamixel.getLastRxPacketError(self.port_num, self.PROTOCOL_VERSION1))
		else:
			print(f"Dynamixel {self.DXL6_ID} has been successfully connected")
			
			
			
	def listener_callback(self, msg):
		joint_states = JointState()
		joint_positions = [0]*6
		joint_velocities = [0]*6
		
		# get the most recent trajectory points
		if msg.points:
			point = msg.points[0]
			self.pos_deg = [int(math.degrees(p)) for p in point.positions] # see about switching to radians
			self.vel_rpm = point.velocities
			self.get_logger().info(f"Recieved JointTrajectory: {points}")

		# send position and velocity to dynamixel servos			
		try:
			# ----- DYNAMIXEL 1 -----
			self.dynamixel.write4ByteTxRx(self.port_num, self.PROTOCOL_VERSION2, self.DXL1_ID, self.ADDR_PRO_GOAL_POSITION, self.pos_deg[0])
			self.dynamixel.write4ByteTxRx(self.port_num, self.PROTOCOL_VERSION2, self.DXL1_ID, self.ADDR_PRO_MAX_VELOCITY, self.vel_rpm[0])
			
				
			# ----- DYNAMIXEL 2 -----
			self.dynamixel.write4ByteTxRx(self.port_num, self.PROTOCOL_VERSION2, self.DXL2_ID, self.ADDR_PRO_GOAL_POSITION, self.pos_deg[1])
			self.dynamixel.write4ByteTxRx(self.port_num, self.PROTOCOL_VERSION2, self.DXL2_ID, self.ADDR_PRO_MAX_VELOCITY, self.vel_rpm[1])
			
			
			# ----- DYNAMIXEL 3 -----
			self.dynamixel.write4ByteTxRx(self.port_num, self.PROTOCOL_VERSION2, self.DXL3_ID, self.ADDR_PRO_GOAL_POSITION, self.pos_deg[2])
			self.dynamixel.write4ByteTxRx(self.port_num, self.PROTOCOL_VERSION2, self.DXL3_ID, self.ADDR_PRO_MAX_VELOCITY, self.vel_rpm[2])
			
			
			# ----- DYNAMIXEL 4 -----
			self.dynamixel.write2ByteTxRx(self.port_num, self.PROTOCOL_VERSION1, self.DXL4_ID, self.ADDR_MX_GOAL_POSITION, self.pos_deg[3])
			self.dynamixel.write2ByteTxRx(self.port_num, self.PROTOCOL_VERSION1, self.DXL4_ID, self.ADDR_MX_MAX_VELOCITY, self.vel_rpm[3])

				
			# ----- DYNAMIXEL 5 -----
			self.dynamixel.write2ByteTxRx(self.port_num, self.PROTOCOL_VERSION1, self.DXL5_ID, self.ADDR_MX_GOAL_POSITION, self.pos_deg[4])
			self.dynamixel.write2ByteTxRx(self.port_num, self.PROTOCOL_VERSION1, self.DXL5_ID, self.ADDR_MX_MAX_VELOCITY, self.vel_rpm[4])

				
			# ----- DYNAMIXEL 6 -----
			self.dynamixel.write2ByteTxRx(self.port_num, self.PROTOCOL_VERSION1, self.DXL6_ID, self.ADDR_MX_GOAL_POSITION, self.pos_deg[5])
			self.dynamixel.write2ByteTxRx(self.port_num, self.PROTOCOL_VERSION1, self.DXL6_ID, self.ADDR_MX_MAX_VELOCITY, self.vel_rpm[5])

		except Exception as e:
			self.get_logger().error(f"Servo Write Error: {e}")
			raise e
		
		# read position and velocity from dynamixel servos
		try:
			# ----- DYNAMIXEL 1 -----
			joint_positions[0] = self.dynamixel.read4ByteTxRx(self.port_num, self.PROTOCOL_VERSION2, DXL1_ID, self.ADDR_PRO_PRESENT_POSITION)
			joint_velocities[0] = self.dynamixel.read4ByteTxRx(self.port_num, self.PROTOCOL_VERSION2, DXL1_ID, self.ADDR_PRO_PRESENT_VELOCITY)
			
			# ----- DYNAMIXEL 2 -----
			joint_positions[1] = self.dynamixel.read4ByteTxRx(self.port_num, self.PROTOCOL_VERSION2, DXL2_ID, self.ADDR_PRO_PRESENT_POSITION)
			joint_velocities[1] = self.dynamixel.read4ByteTxRx(self.port_num, self.PROTOCOL_VERSION2, DXL2_ID, self.ADDR_PRO_PRESENT_VELOCITY)
			
			# ----- DYNAMIXEL 3 -----
			joint_positions[2] = self.dynamixel.read4ByteTxRx(self.port_num, self.PROTOCOL_VERSION2, DXL3_ID, self.ADDR_PRO_PRESENT_POSITION)
			joint_velocities[2] = self.dynamixel.read4ByteTxRx(self.port_num, self.PROTOCOL_VERSION2, DXL3_ID, self.ADDR_PRO_PRESENT_VELOCITY)
			
			# ----- DYNAMIXEL 4 -----
			joint_positions[3] = self.dynamixel.read2ByteTxRx(self.port_num, self.PROTOCOL_VERSION1, DXL4_ID, self.ADDR_MX_PRESENT_POSITION)
			joint_velocities[3] = self.dynamixel.read2ByteTxRx(self.port_num, self.PROTOCOL_VERSION1, DXL4_ID, self.ADDR_MX_PRESENT_POSITION)
			
			# ----- DYNAMIXEL 5 -----
			joint_positions[4] = self.dynamixel.read2ByteTxRx(self.port_num, self.PROTOCOL_VERSION1, DXL5_ID, self.ADDR_MX_PRESENT_POSITION)
			joint_velocities[4] = self.dynamixel.read2ByteTxRx(self.port_num, self.PROTOCOL_VERSION1, DXL5_ID, self.ADDR_MX_PRESENT_POSITION)
			
			# ----- DYNAMIXEL 6 -----
			joint_positions[5] = self.dynamixel.read2ByteTxRx(self.port_num, self.PROTOCOL_VERSION1, DXL6_ID, self.ADDR_MX_PRESENT_POSITION)
			joint_velocities[5] = self.dynamixel.read2ByteTxRx(self.port_num, self.PROTOCOL_VERSION1, DXL6_ID, self.ADDR_MX_PRESENT_POSITION)
						
			
		except Exception as e:
			self.get_logger().error(f"Servo Write Error: {e}")
			raise e
			
		# publish joint states
		joint_states.position = joint_positions
		joint_states.valocity = joint_velocities
		self.pub.publish(joint_states)
			
	def destroy_node(self):
		# disable dynamixel torques
		# dynamixel 1
		self.dynamixel.write1ByteTxRx(self.port_num, self.PROTOCOL_VERSION2, self.DXL1_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
		# dynamixel 2
		self.dynamixel.write1ByteTxRx(self.port_num, self.PROTOCOL_VERSION2, self.DXL2_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
		# dynamixel 3
		self.dynamixel.write1ByteTxRx(self.port_num, self.PROTOCOL_VERSION2, self.DXL3_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
		# dynamixel 4
		self.dynamixel.write1ByteTxRx(self.port_num, self.PROTOCOL_VERSION1, self.DXL4_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
		# dynamixel 5
		self.dynamixel.write1ByteTxRx(self.port_num, self.PROTOCOL_VERSION1, self.DXL5_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
		# dynamixel 6
		self.dynamixel.write1ByteTxRx(self.port_num, self.PROTOCOL_VERSION1, self.DXL6_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
		
		# close dynamixel port
		self.dynamixel.closePort(self.port_num)
		
		# destroy the node
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
		rclpy.shutdown()
		
if __name__ == '__main__':
	main()	
		
		
