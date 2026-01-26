import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
import serial
import math

class TrajectoryToSerialNode(Node):
    def __init__(self):
        super().__init__('trajectory_to_serial_node')
        
        # 1. Persistent Serial Connection
        try:
            self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
            self.get_logger().info("Connected to /dev/ttyACM0")
        except Exception as e:
            self.get_logger().error(f"Serial Error: {e}")
            raise e

        # 2. Subscribe to Joint Trajectory
        self.subscription = self.create_subscription(
            JointTrajectory,
            '/joint_trajectory',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        # 3. Check if trajectory has points
        if msg.points:
            # Get the target positions from the first point in the trajectory
            point = msg.points[0]
            
            # 4. Format and Send (comma-separated string)
            # this needs to be changed to actually match the units the servos use
            # Convert radians (ROS) to degrees
            pos_deg = [int(math.degrees(p)) for p in point.positions]
            vel_deg = [int(math.degrees(v)) for v in point.velocities]
            
            # Format string for broadcast frame: $P1,P2,P3,P4,P5,P6,V1,V2,V3,V4,V5,V6\n
            command = f"${','.join(map(str, pos_deg))},{','.join(map(str, vel_deg))}\n"
            #self.ser.write(command.encode())
            
            
            try:
                self.ser.write(command.encode('utf-8'))
                self.ser.flush() # Force immediate transmission
                # self.get_logger().info(f"Sent: {data_str.strip()}")
            except Exception as e:
                self.get_logger().error(f"Write failed: {e}")

    def destroy_node(self):
        self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryToSerialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
