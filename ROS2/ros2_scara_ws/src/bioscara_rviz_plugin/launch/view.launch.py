# $LICENSE$

from launch import LaunchDescription
from launch.substitutions import (

    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
       
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("bioscara_rviz_plugin"), "display.rviz"]
    )

    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        # prefix=['gdbserver localhost:3000']
    )


    return LaunchDescription([rviz_node])