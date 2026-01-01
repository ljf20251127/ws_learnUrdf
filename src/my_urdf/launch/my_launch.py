from launch.substitutions import Command, PathJoinSubstitution, FindExecutable
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch import LaunchDescription

def generate_launch_description():
    my_package = FindPackageShare("my_urdf")
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            ' ',
            PathJoinSubstitution(
                [
                    my_package, "urdf", "two_link.urdf"
                ]
            )
        ]
    )
    robot_description = {"robot_description": robot_description_content}
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description]
    )

    rviz_config_file = PathJoinSubstitution(
        [
            my_package, "rviz", "conf.rviz"
        ]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file]
    )

    cm_yaml = PathJoinSubstitution(
        [
            my_package,
            "config",
            "cm.yaml"
        ]
    )
    cm_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[cm_yaml],
        output="both"
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )
    
    controllers_yaml = PathJoinSubstitution(
        [
            my_package,
            "config",
            "controllers.yaml"
        ]
    )
    forward_command_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_command_controller", "--param-file", controllers_yaml],
    )

    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    # Delay start of joint_state_broadcaster after `robot_controller`
    # TODO(anyone): This is a workaround for flaky tests. Remove when fixed.
    delay_joint_state_broadcaster_after_robot_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=forward_command_controller_spawner,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    nodes = [
        robot_state_pub_node,
        cm_node,
        forward_command_controller_spawner,
        delay_joint_state_broadcaster_after_robot_controller_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner
    ]

    return LaunchDescription(nodes)
                                        