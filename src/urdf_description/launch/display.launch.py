from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
import os
from ament_index_python import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import  Command,LaunchConfiguration,PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    model_args=DeclareLaunchArgument(
        name="rtp_model",
        default_value=os.path.join(get_package_share_directory("urdf_description"),"urdf","rtp_robot_description.urdf.xacro"),
        description="Path to the robot description"
    )

    robot_description=ParameterValue(Command(["xacro ",LaunchConfiguration("rtp_model")]))

    robot_state_publisher=Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )

    joint_state_publisher_gui=Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui"
    )
    # rviz_node=Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     name="rviz2",
    #     output="screen"
    # )
    # control_node = Node(
    #     package="controller_manager",
    #     executable="ros2_control_node",
    #     parameters=[robot_description, robot_controllers],
    #     output="both",
    # )


    return LaunchDescription([
        model_args,
        robot_state_publisher,
        joint_state_publisher_gui,
    
        # control_node
       
        # rviz_node
    ])