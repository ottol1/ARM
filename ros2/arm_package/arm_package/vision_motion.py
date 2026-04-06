# Author: Leilani Alvarado
# Team: Articulated Removable Manipulator (ARM), 2025-2026
# This code is intended to be used to control the motion of the ARM based on what the camera sees.
# This code is NOT intended to be used at the same time as opencm_command3.
# To have something to start off with, I am copying the opencm_command3 code.
# Start NanoOwl before running this code.

import rclpy
from rclpy.node import Node
from control_msgs.msg import JointTrajectoryControllerState
from sensor_msgs.msg import JointState
from vision_msgs.msg import Detection2DArray
# from vision_msgs.msg import Detection2D
from vision_msgs.msg import BoundingBox2D
# from vision_msgs.msg import Pose2D
import serial
import pyrealsense2 as rs
import math
import subprocess

class visionMotionNode(Node):
    def __init__(self):
        super().__init__('vision_motion_node')

        # Silly code that finds where ARM is (feel free to move this somewhere nicer)
#         port = "/dev/ttyACM0"
#         grep = subprocess.run("udevadm info -q property /dev/ttyACM0 | grep 'ID_MODEL_ID='", shell=True,
#                               capture_output=True, text=True)
#         if grep.returncode != 0:
#             pass  # error out
#         # raise ODDException(f"No ODD Arduino found at {port}")
#
#         model_id_split = grep.stdout.strip().split("=")
#         if len(model_id_split) != 2:
#             pass  # error out
#         # raise ODDException(f"udevadm or grep did something weird: {grep.stdout}")
#
# #        if model_id_split[1] != "ff48":  # This means ODD is at /ttyACM0, probably
# #            port = "/dev/ttyACM1"  # So ARM should be here.
#
#         # start the serial connection
#         try:
#             self.ser = serial.Serial(port, 115200, timeout=1)  # this needs to be changed based on the device :(
#         # self.get_logger().info("Connected to serial /dev/ttyACM0")
#         except Exception as e:
#             self.get_logger().error(f"Serial Error: {e}")
#             raise e

        # subscribe to joint_trajectory topic
        # self.subscription = self.create_subscription(
        #     # FollowJointTrajectory.Goal,
        #     # JointTrajectory
        #     JointTrajectoryControllerState,
        #     # '/joint_trajectory', # possible alternatives: /arm_controller/controller_state
        #     '/arm_controller/controller_state',
        #     # will also need /gripper_controller/
        #     # '/arm_controller/joint_trajectory'
        #     # '/arm_controller/follow_joint_trajectory/_action/goal'
        #     self.listener_callback,
        #     10)
        # self.publisher = self.create_publisher(
        #     JointState,
        #     '/joint_state',
        #     # self.listener_callback,
        #     10)

        # subscribe to bounding boxes topic
        self.subscription = self.create_subscription(
            Detection2DArray,
            '/nanoowl/output_detections',
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(
            BoundingBox2D,
            '/bounding_boxes',
            # self.listener_callback,
            10)

        # self.prev_posCommand_int = [2048, 1024, 2048, 614, 614, 0]  # initial positions
        # self.joint_positions_int = [2048, 1024, 2048, 614, 614, 0]
        # self.posCommand_int = [2048, 1024, 2048, 614, 614, 0]
        # self.velCommand_int = [131, 131, 131, 273, 273, 273]
        # self.joint_tolerance = [11, 3,
        #                         102]  # joint tolerance of 1 degree (in steps respective to motor) and 0.1 N before processing next command

    # def vector_compare(self, a, b):
    #     # Check if the values are within 5 degrees each other
    #     # if it is not true for a single value, return false
    #     # if it is true for every value, return true
    #     for i in range(6):
    #         if i < 3:
    #             if (a[i] > (b[i] - self.joint_tolerance[0])) and (a[i] < (b[i] + self.joint_tolerance[0])):
    #                 return True
    #         elif i < 5:
    #             if (a[i] > (b[i] - self.joint_tolerance[1])) and (a[i] < (b[i] + self.joint_tolerance[1])):
    #                 return True
    #         elif i == 5:
    #             if (a[i] > (b[i] - self.joint_tolerance[2])) and (a[i] < (b[i] + self.joint_tolerance[2])):
    #                 return True
    #     return False

    def listener_callback(self, msg):

        # joint_states = JointState()
        # joint_positions_rad = [0.0] * 6
        # # joint_velocities = [0.0]*6
        # posCommand_rad = [0.0] * 6
        # prev_posCommand_rad = [0.0] * 6
        # velCommand_rad = [0.0] * 6
        # prev_velCommand_rad = [0.0] * 6
        # motor4_pos = 0
        # motor5_pos = 0
        # motor4_vel = 0
        # motor5_vel = 0

        # bounding_boxes = BoundingBox2D()
        # print(f"Bounding box data: {bounding_boxes}") # Not this one. It only returns 0s

        if msg.detections:
            print(f"Bounding box center x:\n{msg.detections[1].bbox.center.position.x}")
            print(f"Bounding box center y:\n{msg.detections[1].bbox.center.position.y}\n")

        else:
            print("No message detected\n")

        # get the most recent trajectory point
        # if msg.desired:  # if there is a new position and velocity, update it!
        #
        #     #print(f"Received: {msg.desired}")
        #     posCommand_rad = list(msg.desired.positions)
        #     velCommand_rad = list(msg.desired.velocities)  # leave velocity in rpm
        #
        #     # quick math for the wrist joint
        #     if posCommand_rad[3] > prev_posCommand_rad[3]:  # if wrist angle is increasing
        #         motor4_pos += posCommand_rad[3] - prev_posCommand_rad[3]
        #         motor5_pos -= posCommand_rad[3] - prev_posCommand_rad[3]
        #     if posCommand_rad[3] < prev_posCommand_rad[3]:  # if wrist angle is decreasing
        #         motor4_pos -= prev_posCommand_rad[3] - posCommand_rad[3]
        #         motor5_pos += prev_posCommand_rad[3] - posCommand_rad[3]
        #     if posCommand_rad[4] > prev_posCommand_rad[4]:  # if frame angle is increasing
        #         motor4_pos -= posCommand_rad[4] - prev_posCommand_rad[4]
        #         motor5_pos -= posCommand_rad[4] - prev_posCommand_rad[4]
        #     if posCommand_rad[4] < prev_posCommand_rad[4]:  # if frame angle is decreasing
        #         motor4_pos += prev_posCommand_rad[4] - posCommand_rad[4]
        #         motor5_pos += prev_posCommand_rad[4] - posCommand_rad[4]
        #
        #     if velCommand_rad[3] > prev_velCommand_rad[3]:  # if wrist angle is increasing
        #         motor4_vel += velCommand_rad[3] - prev_velCommand_rad[3]
        #         motor5_vel -= velCommand_rad[3] - prev_velCommand_rad[3]
        #     if velCommand_rad[3] < prev_velCommand_rad[3]:  # if wrist angle is decreasing
        #         motor4_vel -= prev_velCommand_rad[3] - velCommand_rad[3]
        #         motor5_vel += prev_velCommand_rad[3] - velCommand_rad[3]
        #     if velCommand_rad[4] > prev_velCommand_rad[4]:  # if frame angle is increasing
        #         motor4_vel -= velCommand_rad[4] - prev_velCommand_rad[4]
        #         motor5_vel -= velCommand_rad[4] - prev_velCommand_rad[4]
        #     if velCommand_rad[4] < prev_velCommand_rad[4]:  # if frame angle is decreasing
        #         motor4_vel += prev_velCommand_rad[4] - velCommand_rad[4]
        #         motor5_vel += prev_velCommand_rad[4] - velCommand_rad[4]
        #
        #     posCommand_rad[3] = motor4_pos
        #     posCommand_rad[4] = motor5_pos
        #
        #     velCommand_rad[3] = motor4_vel
        #     velCommand_rad[4] = motor5_vel
        #
        #     for i in range(6):
        #         if i < 3:
        #             # convert to integers based on dynamixel protocol 2
        #             self.posCommand_int[i] = int((posCommand_rad[i] + math.pi) * 4095 / (
        #                         2 * math.pi))  # 0.088 deg per pulse, in rad | 0 - 4095 pulses
        #             self.velCommand_int[i] = int((abs(velCommand_rad[i]) * 60 / (
        #                         2 * math.pi)) / 0.229) + 1  # 0.229 rev/min per pulse | 0 - 2047 pulses # CONVERT FROM RADIANS PER SECOND, NOT RPM
        #         elif i < 5:
        #             # convert to integers based on dynamixel protocol 1
        #             self.posCommand_int[i] = int((posCommand_rad[i] + math.pi) * 1023 / ((2 * math.pi) * (
        #                         300 / 360)))  # 0.293 deg per pulse, in rad (only to 300 deg) | 0 - 1023 pulses
        #             self.velCommand_int[i] = int((abs(velCommand_rad[i]) * 60 / (
        #                         2 * math.pi)) / 0.11) + 1  # 0.110 rev/min per pulse | 0 - 1023 pulses # CONVERT FROM RADIANS PER SECOND, NOT RPM
        #
        #     # NEED TO FIGURE OUT WHERE TO PULL THE DESIRED FORCE SENSOR VALUE FROM
        #     self.posCommand_int[5] = 0
        #     self.velCommand_int[5] = 273

        # format string for broadcast: P1,P2,P3,P4,P5,P6,V1,V2,V3,V4,V5,V6\n
        #command = f"{','.join(map(str, self.posCommand_int))},{','.join(map(str, self.velCommand_int))}\n"
        #print(f"Command: {command}")

        # try to send command
        #try:
            # # self.get_logger().info(f"Sending Command: {command}")
            #self.ser.write(command.encode('utf-8'))
            #self.ser.flush()  # wait for transmission to finish
        # self.get_logger().info("Command Sent!")

        #except Exception as e:
        #    self.get_logger().error(f"Write failed: {e}")

        # try:
        #     # self.get_logger().info("Waiting to receive joint data")
        #     joint_data = self.ser.readline().decode('utf-8').rstrip().split(
        #         ',')  # convert utf-8 status message into a vector
        #     # self.get_logger().info(f"Received Data: {jointData}")
        #     #print(f"Feedback: {joint_data}")

        # except Exception as e:
        #     self.get_logger().error("Failed to receive joint data: {e}")

        # joint_states.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

        # for i in range(6):
        #     self.joint_positions_int[i] = int(float(joint_data[i]))
        #     if i < 3:
        #         joint_positions_rad[i] = float(joint_data[i]) * math.pi / 4095.0
        #     # joint_velocities[i] = float(jointData[i+6]) # new system will only receive positions
        #     elif i < 5:
        #         joint_positions_rad[i] = float(joint_data[i]) * (math.pi - 0.5235987756) / 1023.0
        #     elif i == 5:
        #         joint_positions_rad[i] = float(joint_data[i]) / 1023.0
        # joint_states.position = joint_positions_rad
        # # joint_states.velocity = joint_velocities
        # self.publisher.publish(joint_states)
        # # print(self.joint_positions_int)
        #
        # prev_posCommand_rad = posCommand_rad

    def destroy_node(self):
        #self.ser.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = visionMotionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
    # rclpy.shutdown()


if __name__ == '__main__':
    main()
