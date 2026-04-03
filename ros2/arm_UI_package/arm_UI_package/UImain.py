# Team: Articulated Removable Manipulator (ARM), 2025-2026

import rclpy
from rclpy.node import Node
from control_msgs.msg import JointTrajectoryControllerState
from sensor_msgs.msg import JointState

class armCommandNode(Node):
    def __init__(self):
        super().__init__('arm_command_node')

        # publish to the arm_controller/controller_state topic
        self.arm_publisher = self.create_publisher(
            JointTrajectoryControllerState,
            '/arm_controller/controller_state',
            10
        )

        # publish to the gripper_controller/controller_state topic
        self.gripper_publisher = self.create_publisher(
            JointTrajectoryControllerState,
            'gripper_controller/controller_state',
            10
        )

        # subscribe to the joint_state topic
        self.state_subscriber = self.create_subscriber(
            JointState,
            '/joint_state',
            self.arm_state_subscriber,
            10
        )

    # publish the noint values
    def arm_command_publisher(self, posCommand, velCommand):
        arm_command = JointTrajectoryControllerState()
        gripper_command = JointTrajectoryControllerState()

        arm_command.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
        gripper_command.name = 'joint6'

        arm_command.desired.positions = posCommand[:5]
        arm_command.desired.velocities = velCommand[:5]
        gripper_command.desired.positions = posCommand[5]
        gripper_command.desired.velocities = velCommand[5]   
        
    def arm_state_subscriber(self, msg):
        posActual = list(msg.position)
        velActual = list(msg.velocity)

        return posActual, velActual