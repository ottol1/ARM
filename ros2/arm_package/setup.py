from setuptools import find_packages, setup
from glob import glob

package_name = 'arm_package'

setup(
    name=package_name,
    version='0.0.0',
    #packages=find_packages(exclude=['test']),
    packages = [
    	'arm_package',
    	'arm_package.dynamixel_sdk',
    	],
    #package_dir={'': 'arm_package'},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        #('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='moongoose',
    maintainer_email='moongoose@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
entry_points={
        'console_scripts': [
        	'opencm_command_example = arm_package.opencm_command_example:main',
        	'opencm_command = arm_package.opencm_command:main',
        	'teleop_joint_keyboard = arm_package.teleop_joint_keyboard:main',
        	'opencm_command2 = arm_package.opencm_command2:main',
            'opencm_command3 = arm_package.opencm_command3:main',
            'opencm_command4 = arm_package.opencm_command4:main',
        	'dynamixel_control = arm_package.dynamixel_control:main',
        ],
    },
)
