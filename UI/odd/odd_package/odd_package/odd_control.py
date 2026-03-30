import rclpy
from rclpy.logging import LoggingSeverity
from rclpy.node import Node
from odd_package_interfaces.msg import RPM, EncoderTicks, BumpSensors, Voltage, PID, Temperature
from odd_python.control import ODDRobot
from geometry_msgs.msg import Vector3
from std_srvs.srv import Trigger
from serial import SerialException

class ODDNode(Node):
    def __init__(self):
        super().__init__('odd_node')
        self.running = True
        self.robot = ODDRobot(self.get_logger())
        self.communicate_timer = self.create_timer(1/10, self.communicate)
        self.command_subscription = self.create_subscription(
            RPM,
            'rpm_command',
            self.process_command,
            10
        )
        
        self.pid_left_subscription = self.create_subscription(
            PID,
            'pid_left',
            self.process_pid_left,
            10
        )
        
        self.pid_right_subscription = self.create_subscription(
            PID,
            'pid_right',
            self.process_pid_right,
            10
        )
        
        self.rpm_publisher = self.create_publisher(RPM, 'rpm_actual', 10)
        self.encoder_publisher = self.create_publisher(EncoderTicks, 'encoders', 10)
        self.orientation_publisher = self.create_publisher(Vector3, 'orientation_deg', 10)
        self.acceleration_publisher = self.create_publisher(Vector3, 'linear_accel', 10)
        self.bump_publisher = self.create_publisher(BumpSensors, 'bump_sensors', 10)
        self.voltage_publisher = self.create_publisher(Voltage, 'battery_voltage', 10)
        self.temperature_publisher = self.create_publisher(Temperature, 'imu_temperature', 10)
        
        self.shutdown_service = self.create_service(Trigger, 'shutdown_control', self.shutdown_request)
        
    
    def shutdown_request(self, request, response):
        self.get_logger().info("Shutting down.")
        self.running = False
        return response
    
        
    def process_command(self, msg):
        self.robot.motor_commands[0] = msg.theta_dot_left
        self.robot.motor_commands[1] = msg.theta_dot_right
    
    
    def process_pid_left(self, msg):
        self.robot.motor_left_pid[0] = msg.p
        self.robot.motor_left_pid[1] = msg.i
        self.robot.motor_left_pid[2] = msg.d


    def process_pid_right(self, msg):
        self.robot.motor_right_pid[0] = msg.p
        self.robot.motor_right_pid[1] = msg.i
        self.robot.motor_right_pid[2] = msg.d
    
    
    def communicate(self):
        self.robot.communicate()
	    
        rpm_msg = RPM()
        rpm_msg.theta_dot_left = self.robot.rpm[0]
        rpm_msg.theta_dot_right = self.robot.rpm[1]
        self.rpm_publisher.publish(rpm_msg)
        
        encoder_msg = EncoderTicks()
        encoder_msg.left = self.robot.encoders[0]
        encoder_msg.right = self.robot.encoders[1]
        self.encoder_publisher.publish(encoder_msg)
        
        orientation_msg = Vector3()
        orientation_msg.x = self.robot.orientation[0]
        orientation_msg.y = self.robot.orientation[1]
        orientation_msg.z = self.robot.orientation[2]
        self.orientation_publisher.publish(orientation_msg)
        
        accel_msg = Vector3()
        accel_msg.x = self.robot.acceleration[0]
        accel_msg.y = self.robot.acceleration[1]
        accel_msg.z = self.robot.acceleration[2]
        self.acceleration_publisher.publish(accel_msg)
        
        bump_msg = BumpSensors()
        bump_msg.right = self.robot.bump_sensors[0]
        bump_msg.front = self.robot.bump_sensors[1]
        bump_msg.left = self.robot.bump_sensors[2]
        bump_msg.back = self.robot.bump_sensors[3]
        self.bump_publisher.publish(bump_msg)
        
        voltage_msg = Voltage()
        voltage_msg.volts = self.robot.battery_voltage
        self.voltage_publisher.publish(voltage_msg)
        
        temperature_msg = Temperature()
        temperature_msg.temperature = self.robot.temperature
        self.temperature_publisher.publish(temperature_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ODDNode()
    #node.get_logger().set_level(LoggingSeverity.DEBUG)
    try:
        while node.running:
            rclpy.spin_once(node, timeout_sec=0.01)
    except KeyboardInterrupt:
        pass
    finally:
        node.robot.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
