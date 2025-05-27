# Copyright 2024 NEURA Robotics Gmbh

import logging
import os
import sys

from colorama import Fore, Style
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, RegisterEventHandler,
                            SetLaunchConfiguration)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (Command, FindExecutable, LaunchConfiguration,
                                  PathJoinSubstitution)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


package_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(package_dir)
# TODO: This import is not correct
from scripts.get_robot_info import SSHRobotInfo

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("launch")


def generate_launch_description():
    """_summary_

    Returns
    -------
    _type_
        _description_
    """
    # Simulation params
    use_sim_interface = "true"
    robot_name = "maira7L"
    default_serial_number = "NR0226974"

    try:
        robot_ssh_client = SSHRobotInfo()
        DeclareLaunchArgument(name="log_level", default_value="info")
        SetLaunchConfiguration(name="log_level", value="info")
        robot_name = robot_ssh_client.get_robot_name()
        res, default_serial_number = robot_ssh_client.get_robot_serial_number()
        if res:
            use_sim_interface = "false"
    except BaseException:
        warning_msg = {Fore.YELLOW} + "Can not connect the robot, " + \
        "please check the robot connection, try to use use_sim_interface."+{Style.RESET_ALL}
        logger.warning(warning_msg)

    # Declare arguments
    declared_arguments = []
    declared_arguments = [
        DeclareLaunchArgument(
            "gui",
            default_value="false",
            description="Start RViz2 automatically with this launch file.",
        ),
        DeclareLaunchArgument(
            "robot_serial_number",
            default_value=default_serial_number,
            description="robot serial number",
        ),
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="use fake hardware to control robot",
        ),
        DeclareLaunchArgument(
            "sim_isaac",
            default_value="false",
            description="use isaac sim to control robot",
        ),
        DeclareLaunchArgument(
            "use_sim_interface",
            default_value=use_sim_interface,
            description="use our robot simulation interface",
        ),
    ]
    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("neura_ai_robot_description"),
                    "urdf",
                    f"{robot_name}.urdf.xacro",
                ]
            ),
            " ",
            "robot_serial_number:=",
            LaunchConfiguration(
                "robot_serial_number"
            ),  # Pass the robot_serial_number parameter
            " ",
            "use_fake_hardware:=",
            LaunchConfiguration("use_fake_hardware"),
            " ",
            "sim_isaac:=",
            LaunchConfiguration("sim_isaac"),
            " ",
            "use_sim_interface:=",
            LaunchConfiguration("use_sim_interface"),
        ]
    )
    logger.info("robot description loaded")
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("neura_ai_robot_bringup"),
            "config",
            f"{robot_name}_controllers.yaml",
        ]
    )
    rviz_config_file = PathJoinSubstitution(
        [
            FindPackageShare("neura_ai_robot_bringup"),
            "rviz",
            f"{robot_name}.rviz",
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
    )
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[
            {
                "robot_description": robot_description_content,
                "publish_frequency": 50.0,
            }
        ],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(LaunchConfiguration("gui")),
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_trajectory_position_controller",
            "--inactive",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    # Delay rviz start after `joint_state_broadcaster`
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
            target_action=robot_controller_spawner,
            on_exit=[joint_state_broadcaster_spawner],
        ))

    nodes = [
        control_node,
        robot_state_pub_node,
        robot_controller_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
        delay_joint_state_broadcaster_after_robot_controller_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes)
