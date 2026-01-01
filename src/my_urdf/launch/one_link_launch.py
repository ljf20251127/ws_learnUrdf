from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument(name='model'))
    robot_description_content = ParameterValue(Command(['xacro ',LaunchConfiguration('model')]), value_type=str)
    robot_state_publisher=Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description':robot_description_content
            }]
        )
    rviz2=Node(
        package='rviz2',
        executable='rviz2',
        output='screen'
        )
    ld.add_action(robot_state_publisher)
    ld.add_action(rviz2)
    return ld
