# Author: Leilani Alvarado
# Team: Articulated Removable Manipulator (ARM), 2025-2026
# This code is intended to be used to control the motion of the ARM based on what the camera sees.
# This code is NOT intended to be used at the same time as opencm_command3.
# To have something to start off with, I am copying the opencm_command3 and opencm_command4 code.
# Start NanoOwl before running this code.

import rclpy
from rclpy.node import Node
from control_msgs.msg import JointTrajectoryControllerState
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Image as msg_Image
from vision_msgs.msg import Detection2DArray
from vision_msgs.msg import BoundingBox2D
import serial
import pyrealsense2 as rs
# import numpy as np
# import PIL
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

        # # subscribe to the controller topics
        # self.arm_subscription = self.create_subscription(
        #     JointTrajectoryControllerState,
        #     '/arm_controller/controller_state',
        #     self.arm_listener,
        #     10)
        # self.gripper_subscription = self.create_subscription(
        #     JointTrajectoryControllerState,
        #     'gripper_controller/controller_state',
        #     self.gripper_listener,
        #     10)
        #
        # # publish to joint_state topic
        # self.publisher = self.create_publisher(
        #     JointState,
        #     '/joint_state',
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
        # self.joint_tolerance = [11, 3, 102]  # joint tolerance of 1 degree (in steps respective to motor) and 0.1 N before processing next command
        # self.prev_posCommand_rad = [0.0] * 6
        # self.prev_velCommand_rad = [0.0] * 6
        # self.posCommand_rad = [0.0] * 6
        # self.velCommand_rad = [0.0] * 6

        # self.timer = self.create_timer(0.02, self.listener_callback)  # needed to spin the node

    # def gripper_listener(self, msg):
    #     # check for new gripper messages
    #     if msg.desired:
    #         print(f"Gripper msg recieved: {msg.desired}")
    #         self.posCommand_rad[5] = list(msg.desired.positions)
    #         self.velCommand_rad[5] = list(msg.desired.velocities)
    #
    # def arm_listener(self, msg):
    #     # check for new arm messages
    #     if msg.desired:
    #         print(f"Arm msg recieved: {msg.desired}")
    #         arm_posCommand_rad = list(msg.desired.positions)
    #         arm_velCommand_rad = list(msg.desired.velocities)
    #         for i in range(5):
    #             self.posCommand_rad[i] = arm_posCommand_rad[i]
    #             self.velCommand_rad[i] = arm_velCommand_rad[i]

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

    # convert wrist attitude and rotation to joint
    # def wrist_math(self):
    #     # quick math for the wrist joint
    #     wristAtt_pos = self.posCommand_rad[3]
    #     wristRot_pos = self.posCommand_rad[4]
    #     wristAtt_vel = self.velCommand_rad[3]
    #     wristRot_vel = self.velCommand_rad[4]
    #
    #     servo4_pos = 0.0
    #     servo5_pos = 0.0
    #     servo4_vel = 0.0
    #     servo5_vel = 0.0
    #
    #
    #     self.posCommand_rad[3] = servo4_pos
    #     self.posCommand_rad[4] = servo5_pos
    #     self.velCommand_rad[3] = servo4_vel
    #     self.velCommand_rad[4] = servo5_vel


    # def rad_to_int(self):
    #     # convert to integers for dynamixel to use
    #     for i in range(6):
    #         if i < 3:
    #             # convert to integers based on dynamixel protocol 2
    #             self.posCommand_int[i] = int((self.posCommand_rad[i]+math.pi)*4095/(2*math.pi)) # 0.088 deg per pulse, in rad | 0 - 4095 pulses
    #             self.velCommand_int[i] = int((abs(self.velCommand_rad[i])*60/(2*math.pi))/0.229) + 1 # 0.229 rev/min per pulse | 0 - 2047 pulses
    #         elif i < 5:
    #             # convert to integers based on dynamixel protocol 1
    #             self.posCommand_int[i] = int((self.posCommand_rad[i]+math.pi)*1023/((2*math.pi)*(300/360))) # 0.293 deg per pulse, in rad (only to 300 deg) | 0 - 1023 pulses
    #             self.velCommand_int[i] = int((abs(self.velCommand_rad[i])*60/(2*math.pi))/0.11) + 1 # 0.110 rev/min per pulse | 0 - 1023 pulses
    #         elif i == 5:
    #             # force sensor works a bit differently
    #             self.posCommand_int[5] = self.posCommand_rad[5]*1023 # up to 1 N
    #             self.velCommand_int[5] = int((abs(self.velCommand_rad[5])*60/(2*math.pi))/0.229) + 1025 # 0.229 rev/min per pulse | 1025 - 2047 pulses for reversed direction


    def listener_callback(self, msg):
        if msg.detections:
            print(f"Bounding box center x:\n{msg.detections[1].bbox.center.position.x}")
            print(f"Bounding box center y:\n{msg.detections[1].bbox.center.position.y}\n")

        else:
            print("No message detected\n")


        # W = 848
        # H = 480

        # config = rs.config()
        # config.enable_stream(rs.stream.color, W, H, rs.format.bgr8, 30)
        # config.enable_stream(rs.stream.depth, W, H, rs.format.z16, 30)
        #
        # pipeline = rs.pipeline()
        # profile = pipeline.start(config)
        #
        # align_to = rs.stream.color
        # align = rs.align(align_to)
        #
        # while True:
        #     frames = pipeline.wait_for_frames()
        #
        #     aligned_frames = align.process(frames)
        #     color_frame = aligned_frames.get_color_frame()
        #     depth_frame = aligned_frames.get_depth_frame()
        #     if not color_frame:
        #         continue

            # color_image = np.asanyarray(color_frame.get_data())
            # depth_image = np.asanyarray(depth_frame.get_data())
            # depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.08), cv2.COLORMAP_JET)

            # results = model(color_image)

        #
            # x, y = int(msg.detections[1].bbox.center.position.x), int(msg.detections[1].bbox.center.position.y)
            # dist = depth_frame.get_distance(x, y)
            # if (dist < 1.2) & (dist > 0.064):
            #     print(f"The camera is facing an object {dist:.3f} meters away", end="\r")

        # # From ODD: Following function was written by Gemini
        # def ros2_image_to_pil(ros_image_msg):
        #     """
        #     Converts a ROS2 sensor_msgs/Image to a PIL Image using only Pillow.
        #
        #     Args:
        #         ros_image_msg: The uncompressed ROS2 image message.
        #
        #     Returns:
        #         PIL.Image: The converted Pillow image.
        #     """
        #     # Map ROS2 encodings to PIL (Image Mode, Raw Mode)
        #     encoding_mapping = {
        #         'mono8': ('L', 'L'),
        #         '8UC1': ('L', 'L'),
        #         '16UC1': ('I;16', 'I;16'),
        #         'rgb8': ('RGB', 'RGB'),
        #         'rgba8': ('RGBA', 'RGBA'),
        #         'bgr8': ('RGB', 'BGR'),
        #         'bgra8': ('RGBA', 'BGRA')
        #     }
        #
        #     if ros_image_msg.encoding not in encoding_mapping:
        #         raise ValueError(f"Unsupported encoding: {ros_image_msg.encoding}")
        #
        #     pil_mode, raw_mode = encoding_mapping[ros_image_msg.encoding]
        #
        #     # Image dimensions
        #     size = (ros_image_msg.width, ros_image_msg.height)
        #
        #     # ROS2 message data is typically an array.array or sequence; convert to strict bytes
        #     byte_data = bytes(ros_image_msg.data)
        #
        #     # Construct and return the PIL Image
        #     return PIL.Image.frombytes(pil_mode, size, byte_data, 'raw', raw_mode)

        # joint_states = JointState()
        # joint_positions_rad = [0.0] * 6
        # # joint_velocities_rad = [0.0]*6
        #
        # # check if the current joint positions match the previous joint command
        # # if self.vector_compare(self.prev_posCommand_int, self.joint_positions_int): # - after we get the UI working, we need to double back to see if this is needed
        #
        # self.wrist_math()
        #
        # # convert to integers, and save to self variables
        # self.rad_to_int()
        # # format string for broadcast: P1,P2,P3,P4,P5,P6,V1,V2,V3,V4,V5,V6\n
        # command = f"{','.join(map(str, self.posCommand_int))},{','.join(map(str, self.velCommand_int))}\n"
        # print(f"Command: {command}")
        #
        # # try to send command
        # try:
        #     # self.get_logger().info(f"Sending Command: {command}")
        #     self.ser.write(command.encode('utf-8'))
        #     self.ser.flush()  # wait for transmission to finish
        # # self.get_logger().info("Command Sent!")
        #
        # except Exception as e:
        #     self.get_logger().error(f"Write failed: {e}")
        #
        # try:
        #     # self.get_logger().info("Waiting to recieve joint data")
        #     joint_data = self.ser.readline().decode('utf-8').rstrip().split(
        #         ',')  # convert utf-8 status message into a vector
        #     # self.get_logger().info(f"Recieved Data: {jointData}")
        #     print(f"Feedback: {joint_data}")
        #
        # except Exception as e:
        #     self.get_logger().error("Failed to recieve joint data: {e}")
        #
        # joint_states.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        #
        # for i in range(6):
        #     self.joint_positions_int[i] = int(float(joint_data[i]))
        #     if i < 3:
        #         joint_positions_rad[i] = float(joint_data[i]) * math.pi / 4095.0
        #     # joint_velocities[i] = float(jointData[i+6]) # new system will only recieve positions
        #     elif i < 5:
        #         joint_positions_rad[i] = float(joint_data[i]) * (math.pi - 0.5235987756) / 1023.0
        #     elif i == 5:
        #         joint_positions_rad[i] = float(joint_data[i]) / 1023.0
        # joint_states.position = joint_positions_rad
        # # joint_states.velocity = joint_velocities
        # self.publisher.publish(joint_states)
        # # print(self.joint_positions_int)
        #
        # self.prev_posCommand_rad = self.posCommand_rad
        # self.prev_velCommand_rad = self.velCommand_rad

    def destroy_node(self):
        #self.ser.close()
        super().destroy_node()

def main():
    print("Entered vision_motion main")
    rclpy.init()
    node = visionMotionNode()
    print("Assigned vision_motion node")
    try:
        print("Trying to spin vision_motion node")
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

if __name__ == '__main__':
    main()
