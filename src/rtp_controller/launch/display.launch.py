from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
import os
from ament_index_python import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import  Command,LaunchConfiguration,PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare






def generate_launch_description():

  
    joint_state_publisher=Node(
         package="controller_manager",
        executable="spawner",
       
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager"
        ]
    )
    diff_publisher=Node(
        package="controller_manager",
        executable="spawner",
        
        arguments=[
            "rtp_controller",
            "--controller-manager",
            "/controller_manager",
        ]
    )
    

    return LaunchDescription([
        joint_state_publisher,
        diff_publisher,
  

    ])