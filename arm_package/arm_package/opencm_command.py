import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
import serial
import json
import math

curr_pos = []
curr_vel = []

class opencm_command(Node):
    def __init__(self):
        super().__init__('opencm_command_node')
        self.subscription = self.create_subscription(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            self.listener_callback,
            10)
        self.ser = serial.Serial('/dev/ttyACM0', 115200) # make sure the OpenCM is set as ttyACM0, or find a way to make this more adaptive

    def listener_callback(self, msg):
        if not msg.points: return
        
        # take the most recent point in the trajectory
        point = msg.points
        
        # this needs to be changed to actually match the units the servos use
        # Convert radians (ROS) to degrees
        pos_deg = [int(math.degrees(p)) for p in point.positions]
        vel_deg = [int(math.degrees(v)) for v in point.velocities]

        # Format string for broadcast frame: $P1,P2,P3,P4,P5,P6,V1,V2,V3,V4,V5,V6\n
        command = f"${','.join(map(str, pos_deg))},{','.join(map(str, vel_deg))}\n"
        self.ser.write(command.encode())
        
        #read from serial to get current joint data
        #joint_data = self.ser.read()

def main(args=None):
    rclpy.init(args=args)
    node = opencm_command()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
