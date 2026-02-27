from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
	ld = LaunchDescription()
	
	arm_urdf_path = FindPackageShare('arm_package')
	default_model_path = PathJoinSubstitution(['urdf', 'test07.urdf'])
	default_rviz_config_path = PathJoinSubstitution([arm_urdf_path, 'rviz', 'urdf.rviz'])
	
	# leave for backward compatability
	gui_arg = DeclareLaunchArgument(name='gui', default_value='true', choices=['true', 'false'], description='Flag to enable joint_state_publisher_gui')
	ld.add_action(gui_arg)
	rviz_arg = DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path, description='Absolute path to rviz config file')
	ld.add_action(rviz_arg)
	
	# parameter has changed meaning from previous versions
	ld.add_action(DeclareLaunchArgument(name='model', default_value=default_model_path, description='Path to robot urdf file relative to arm_package package'))
	
	ld.add_action(IncludeLaunchDescription(PathJoinSubstitution([FindPackageShare('urdf_launch'), 'launch', 'display.launch.py']), launch_arguments={
		'urdf_package': 'arm_package',
		'urdf_package_path': LaunchConfiguration('model'),
		'rviz_config': LaunchConfiguration('rvizconfig'),
		'jsp_gui': LaunchConfiguration('gui')}.items()
		))
	
	return ld
