# $LICENSE$

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument
)
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed than also joint names in the controllers' configuration \
        have to be updated.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "arm_description_package",
            default_value="bioscara_description",
            description="Description package holding arm descriptions.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "arm_macro_path",
            default_value='"/urdf/bioscara_arm.xacro"',
            description="Path inside description package of the macro that holds the load_arm macro.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gripper_description_package",
            default_value="bioscara_gripper_descriptions",
            description="Description package holding gripper descriptions.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gripper_macro_path",
            default_value='"/urdf/bioscara_gripper_128.xacro"',
            description="Path inside description package of the macro that holds the load_gripper macro.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="false",
            description="Start robot with fake hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
        )
    )
    

    # Initialize Arguments
    arm_description_package = LaunchConfiguration("arm_description_package")
    arm_macro_path = LaunchConfiguration("arm_macro_path")
    gripper_description_package = LaunchConfiguration("gripper_description_package")
    gripper_macro_path = LaunchConfiguration("gripper_macro_path")
    prefix = LaunchConfiguration("prefix")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    gui = LaunchConfiguration("gui")
    xacro_file = "scene_descriptions"

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(xacro_file), "urdf/scene.xacro"]
            ),
            " ",
            "prefix:=",
            prefix,
            " ",
            "use_mock_hardware:=",
            use_mock_hardware,
            " ",
            "arm_description_package:=",
            arm_description_package,
            " ",
            "arm_macro_path:=",
            arm_macro_path,
            " ",
            "gripper_description_package:=",
            gripper_description_package,
            " ",
            "gripper_macro_path:=",
            gripper_macro_path,
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("scene_descriptions"), "rviz", "display.rviz"]
    )


    # start the robot state publisher node which gets the robot description file as paramter
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(gui),
    )

    joint_state_publisher_node = Node(
        condition=UnlessCondition(gui),
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    joint_state_publisher_gui_node = Node(
        condition=IfCondition(gui),
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    return LaunchDescription(
        declared_arguments
        + [
            joint_state_publisher_node,
            robot_state_pub_node,
            joint_state_publisher_gui_node,
            rviz_node,
        ]
    )