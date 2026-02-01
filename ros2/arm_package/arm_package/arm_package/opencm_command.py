import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState
import serial
import math

# this should allow us to start opencm_command before or after the joint trajectory has been sent
pos_deg = [154, 343, 165, 188, 157, 0] # default starting configuration
vel_rpm = [30, 30, 30, 30, 30, 30] # safe velocity to move at


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
			'/joint_trajectory',
			self.listener_callback,
			10)
		self.publisher = self.create_publisher(
			JointState,
			'/joint_state',
			#self.listener_callback,
			10)
			
	def listener_callback(self, msg):
	
		joint_states = JointState()
		joint_positions = [0]*6
		joint_velocities = [0]*6
	
		#get the most recent trajectory points
		if msg.points: # if there is a new position and velocity, update it!
			#get target positions and velocity from the first point in the trajectory, could be adapted to cycle through trajectory points?
			point = msg.points[0]
			
			# format commands to send over serial
			pos_deg = [int(math.degrees(p)) for p in point.positions]
			vel_rpm = point.velocities # leave velocity in rpm
			
			
		# check that it is allowed to send to the opencm and is not waiting to hear the current configuration
		
		# format string for broadcast: $P1,P2,P3,P4,P5,P6,V1,V2,V3,V4,V5,V6
		command = f"${','.join(map(str, pos_deg))},{','.join(map(str, vel_rpm))}\n"
		
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
			jointData = self.ser.readline().decode('utf-8').rstrip().split(',') # convert utf-8 status message into a vector
			#self.get_logger().info(f"Recieved Data: {jointData}")
		except Exception as e:
			self.get_logger().error("Failed to recieve joint data: {e}")
		joint_states.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
		for i in range(6):
			joint_positions[i] = float(jointData[i])
			joint_velocities[i] = float(jointData[i+6])
		joint_states.position = joint_positions
		joint_states.velocity = joint_velocities
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
