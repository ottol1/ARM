import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
import serial
import math

class TrajectoryToSerialNode(Node):
    def __init__(self):
        super().__init__('trajectory_to_serial_node')
        
        # persistent serial connection
        try:
            self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1) # try to make more adaptive?
            self.get_logger().info("Connected to serial /dev/ttyACM0")
        except Exception as e:
            self.get_logger().error(f"Serial Error: {e}")
            raise e

        # subscribe to joint_trajectory
        self.subscription = self.create_subscription(
            JointTrajectory,
            '/joint_trajectory',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        # get the treajectory points
        if msg.points:
            # Get the target positions from the first point in the trajectory
            point = msg.points[0]
            
            # format the commands
            # this needs to be changed to actually match the units the servos use
            # convert radians to degrees
            pos_deg = [int(math.degrees(p)) for p in point.positions]
            vel_deg = [int(math.degrees(v)) for v in point.velocities]
            
            # format string for broadcast: $P1,P2,P3,P4,P5,P6,V1,V2,V3,V4,V5,V6\n, but what if there is accidentally more than 6 values for each?
            command = f"${','.join(map(str, pos_deg))},{','.join(map(str, vel_deg))}\n"
            self.get_logger().info(f"Sending joint command {command}")
            
            # change to send initial command, but only 
            try:
                self.ser.write(command.encode('utf-8'))
                self.ser.flush() # Force immediate transmission
                self.get_logger().info(f"Sent: {command}")
                
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
