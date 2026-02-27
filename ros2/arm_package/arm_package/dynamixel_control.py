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

# DYNAMIXEL SDK
from arm_package.dynamixel_sdk.port_handler import PortHandler
from arm_package.dynamixel_sdk.packet_handler import PacketHandler


# ----- CLASS DEFINITION -----
class opencmCommandNode(Node):

    def __init__(self):
        super().__init__('opencm_command_node')

        # initial position and velocities (still degrees/RPM until you convert later)
        self.pos_deg = [154.0, 343.0, 165.0, 188.0, 157.0, 0.0]
        self.vel_rpm = [30.0, 30.0, 30.0, 30.0, 30.0, 30.0]

        # track whether a trajectory has ever been received
        self.have_received_trajectory = False

        # store last known valid readings
        self.last_positions = [0] * 6
        self.last_velocities = [0] * 6

        # ROS2 interfaces
        self.sub = self.create_subscription(
            JointTrajectory,
            '/joint_trajectory',
            self.listener_callback,
            100
        )

        self.pub = self.create_publisher(
            JointState,
            '/joint_state',
            100
        )

        # DYNAMIXEL control table addresses
        # MX (Protocol 1)
        self.ADDR_MX_TORQUE_ENABLE = 24
        self.ADDR_MX_GOAL_POSITION = 30
        self.ADDR_MX_MAX_VELOCITY = 32
        self.ADDR_MX_PRESENT_POSITION = 36
        self.ADDR_MX_PRESENT_VELOCITY = 38

        # PRO (Protocol 2)
        self.ADDR_PRO_TORQUE_ENABLE = 562
        self.ADDR_PRO_GOAL_POSITION = 596
        self.ADDR_PRO_MAX_VELOCITY = 112
        self.ADDR_PRO_PRESENT_POSITION = 611
        self.ADDR_PRO_PRESENT_VELOCITY = 615

        # protocol versions
        self.PROTOCOL_VERSION1 = 1.0
        self.PROTOCOL_VERSION2 = 2.0

        # servo IDs
        self.DXL1_ID = 1
        self.DXL2_ID = 2
        self.DXL3_ID = 3
        self.DXL4_ID = 4
        self.DXL5_ID = 5
        self.DXL6_ID = 6

        self.BAUDRATE = 1000000
        self.DEVICENAME = "/dev/ttyUSB0"

        self.TORQUE_ENABLE = 1
        self.TORQUE_DISABLE = 0

        # Initialize port + packet handlers
        self.portHandler = PortHandler(self.DEVICENAME)
        self.packetHandler1 = PacketHandler(self.PROTOCOL_VERSION1)
        self.packetHandler2 = PacketHandler(self.PROTOCOL_VERSION2)

        # Open port
        if self.portHandler.openPort():
            self.get_logger().info("Opened U2D2 Port")
        else:
            self.get_logger().error("Failed to open U2D2 Port")
            raise RuntimeError("Port open failed")

        # Set baudrate
        if self.portHandler.setBaudRate(self.BAUDRATE):
            self.get_logger().info(f"Set Baudrate {self.BAUDRATE}")
        else:
            self.get_logger().error("Failed to set Baudrate")
            raise RuntimeError("Baudrate set failed")

        # enable torque on PRO servos
        for dxl_id in [self.DXL1_ID, self.DXL2_ID, self.DXL3_ID]:
            self.enable_torque(dxl_id, self.packetHandler2, self.ADDR_PRO_TORQUE_ENABLE)

        # enable torque on MX servos
        for dxl_id in [self.DXL4_ID, self.DXL5_ID, self.DXL6_ID]:
            self.enable_torque(dxl_id, self.packetHandler1, self.ADDR_MX_TORQUE_ENABLE)

        # ----- INITIAL POSITION WRITE -----
        try:
            # PRO servos
            for i, dxl_id in enumerate([self.DXL1_ID, self.DXL2_ID, self.DXL3_ID]):
                self.packetHandler2.write4ByteTxRx(
                    self.portHandler, dxl_id, self.ADDR_PRO_GOAL_POSITION, int(self.pos_deg[i])
                )
                self.packetHandler2.write4ByteTxRx(
                    self.portHandler, dxl_id, self.ADDR_PRO_MAX_VELOCITY, int(self.vel_rpm[i])
                )

            # MX servos
            for i, dxl_id in enumerate([self.DXL4_ID, self.DXL5_ID, self.DXL6_ID], start=3):
                self.packetHandler1.write2ByteTxRx(
                    self.portHandler, dxl_id, self.ADDR_MX_GOAL_POSITION, int(self.pos_deg[i])
                )
                self.packetHandler1.write2ByteTxRx(
                    self.portHandler, dxl_id, self.ADDR_MX_MAX_VELOCITY, int(self.vel_rpm[i])
                )

            self.get_logger().info("Initial servo pose written.")
        except Exception as e:
            self.get_logger().error(f"Initial write failed: {e}")

        # continuous read timer (50 Hz)
        self.read_timer = self.create_timer(0.02, self.read_timer_callback)

    # ----- TORQUE ENABLE -----
    def enable_torque(self, dxl_id, handler, address):
    	dxl_comm_result, dxl_error = handler.write1ByteTxRx(
        	self.portHandler, dxl_id, address, 1
    	)

    	if dxl_comm_result != 0:  # COMM_SUCCESS
        	print(f"[ID:{dxl_id}] Communication error: {handler.getTxRxResult(dxl_comm_result)}")
        	return False

    	if dxl_error != 0:
        	print(f"[ID:{dxl_id}] Packet error: {handler.getRxPacketError(dxl_error)}")
        	return False

    	print(f"[ID:{dxl_id}] Torque enabled")
    	return True


    # ----- LISTENER -----
    def listener_callback(self, msg):
        if not msg.points:
            return

        point = msg.points[-1]

        # store goals
        self.pos_deg = list(point.positions) if point.positions else self. pos_deg # [int(math.degrees(p)) for p in point.positions]
        self.vel_rpm = list(point.velocities) if point.velocities else self.vel_rpm

        self.have_received_trajectory = True
        self.get_logger().info("Received new JointTrajectory point")

        try:
            # PRO servos (Protocol 2)
            for i, dxl_id in enumerate([self.DXL1_ID, self.DXL2_ID, self.DXL3_ID]):
                self.packetHandler2.write4ByteTxRx(
                    self.portHandler, dxl_id, self.ADDR_PRO_GOAL_POSITION, int(self.pos_deg[i])
                )
                self.packetHandler2.write4ByteTxRx(
                    self.portHandler, dxl_id, self.ADDR_PRO_MAX_VELOCITY, int(self.vel_rpm[i])
                )

            # MX servos (Protocol 1)
            for i, dxl_id in enumerate([self.DXL4_ID, self.DXL5_ID, self.DXL6_ID], start=3):
                self.packetHandler1.write2ByteTxRx(
                    self.portHandler, dxl_id, self.ADDR_MX_GOAL_POSITION, int(self.pos_deg[i])
                )
                self.packetHandler1.write2ByteTxRx(
                    self.portHandler, dxl_id, self.ADDR_MX_MAX_VELOCITY, int(self.vel_rpm[i])
                )

        except Exception as e:
            self.get_logger().error(f"Servo Write Error: {e}")

    # ----- CONTINUOUS READ LOOP (SAFE VERSION) -----
    def read_timer_callback(self):
        joint_states = JointState()
        joint_states.header.stamp = self.get_clock().now().to_msg()

        joint_positions = [0] * 6
        joint_velocities = [0] * 6

        try:
            # PRO servos
            for i, dxl_id in enumerate([self.DXL1_ID, self.DXL2_ID, self.DXL3_ID]):
                pos, _, _ = self.packetHandler2.read4ByteTxRx(
                    self.portHandler, dxl_id, self.ADDR_PRO_PRESENT_POSITION
                )
                vel, _, _ = self.packetHandler2.read4ByteTxRx(
                    self.portHandler, dxl_id, self.ADDR_PRO_PRESENT_VELOCITY
                )

                if pos is None:
                    pos = self.last_positions[i]
                if vel is None:
                    vel = self.last_velocities[i]

                joint_positions[i] = float(pos)
                joint_velocities[i] = float(vel)

            # MX servos
            for i, dxl_id in enumerate([self.DXL4_ID, self.DXL5_ID, self.DXL6_ID], start=3):
                pos, _, _ = self.packetHandler1.read2ByteTxRx(
                    self.portHandler, dxl_id, self.ADDR_MX_PRESENT_POSITION
                )
                vel, _, _ = self.packetHandler1.read2ByteTxRx(
                    self.portHandler, dxl_id, self.ADDR_MX_PRESENT_VELOCITY
                )

                if pos is None:
                    pos = self.last_positions[i]
                if vel is None:
                    vel = self.last_velocities[i]

                joint_positions[i] = float(pos)
                joint_velocities[i] = float(vel)

        except Exception as e:
            self.get_logger().error(f"Servo Read Error: {e}")
            return

        # Save last known valid values
        self.last_positions = joint_positions
        self.last_velocities = joint_velocities

        # Publish
        joint_states.position = joint_positions
        joint_states.velocity = joint_velocities
        self.pub.publish(joint_states)

    # ----- CLEAN SHUTDOWN -----
    def destroy_node(self):
        # disable torque
        for dxl_id in [self.DXL1_ID, self.DXL2_ID, self.DXL3_ID]:
            self.packetHandler2.write1ByteTxRx(
                self.portHandler, dxl_id, self.ADDR_PRO_TORQUE_ENABLE, self.TORQUE_DISABLE
            )

        for dxl_id in [self.DXL4_ID, self.DXL5_ID, self.DXL6_ID]:
            self.packetHandler1.write1ByteTxRx(
                self.portHandler, dxl_id, self.ADDR_MX_TORQUE_ENABLE, self.TORQUE_DISABLE
            )

        self.portHandler.closePort()
        super().destroy_node()


# ----- MAIN -----
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

