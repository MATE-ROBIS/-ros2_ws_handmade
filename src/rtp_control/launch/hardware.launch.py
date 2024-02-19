import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
import rclpy

def generate_launch_description():

    # Get URDF via xacro
    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                os.path.join(
                    get_package_share_directory("urdf_description"),
                    "urdf",
                    "rtp_robot_description.urdf.xacro",
                ),
               
            ]
        ),
        # value_type=str,
    )
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
    )


    # test_controller = os.path.join(
    #     get_package_share_directory('rtp_control'),
    #     ' config',
    #     'diff_rtp_controller.yaml'
    #     )
    # control_node=Node(
    #     package='controller_manager',
    #     executable='ros2_control_node',
    #     parameters=[robot_description, test_controller],
    #     # output={
    #     #   'stdout': 'screen',
    #     #   'stderr': 'screen',
    #     #   },
    #     )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description,
             "use_sim_time": False},
            os.path.join(
                get_package_share_directory("rtp_control"),
                "config",
                "diff_rtp_controller.yaml",
            )
        ],
    )
    spawn_dd_controller = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["diff_controller","--controller-manager","/controller_manager"],
        output="screen",
    )

    spawn_jsb_controller = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_state_broadcaster","--controller-manager","/controller_manager"],
        output="screen",
    )
    delayed_controller_manager = TimerAction(period=3.0,actions=[controller_manager])
    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[spawn_dd_controller],
        )
    )
    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[spawn_jsb_controller],
        )
    )
    return LaunchDescription([
        
        controller_manager,
        robot_state_publisher_node,
        # spawn_jsb_controller,
        # delayed_controller_manager,
        delayed_diff_drive_spawner,
        delayed_joint_broad_spawner

        # spawn_dd_controller,
        # spawn_jsb_controller
        # control_node,
        # test_controller
 

    ])