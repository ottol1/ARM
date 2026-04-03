from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            # arguments=['--ros-args', '-r', '__node:=D405',  '-r',  '__ns:=/arm', '-p', 'device_type:=D405']
            arguments=['--ros-args', '-r', '__node:=D405',  '-r',  '__ns:=/arm', '-p', 'serial_no:=_335122272231', '-r', '/arm/D405/color/image_raw:=/arm/D405/color/image_rect_raw']
        ),
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            # arguments=['--ros-args', '-r', '__node:=D435',  '-r',  '__ns:=/odd', '-p', 'device_type:=D435']
            arguments=['--ros-args', '-r', '__node:=D435',  '-r',  '__ns:=/odd', '-p', 'serial_no:=_207222073869', '-r', '/odd/D435/color/image_raw:=/odd/D435/color/image_rect_raw']
        ),
        Node(
            package='topic_tools',
            executable='mux',
            name='input_image_mux',
            namespace='/nanoowl',
            arguments=['/nanoowl/input_image', '/odd/D435/color/image_rect_raw', '/arm/D405/color/image_rect_raw']
        ),
        #ExecuteProcess(
        #    cmd=['jetson-containers', 'run', '--ipc=host', '--pid=host', 'odd_arm/nanoowl:1.0.0 bash /opt/odd_arm_cv/nanoowl.sh'],
        #    output='screen'
        #)
    ])
