from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            # arguments=['--ros-args', '--log-level', 'DEBUG']
            arguments=['--ros-args', '-p', 'pointcloud.enable:=true', '-p',  'align_depth.enable:=true', '-r',  '__ns:=/arm']
            # arguments=['--ros-args', '-r', '__node:=D435',  '-r',  '__ns:=/odd', '-p', 'serial_no:=_207222073869', '-r', '/odd/D435/color/image_raw:=/odd/D435/color/image_rect_raw']
        )
    ])