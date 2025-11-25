# $LICENSE$

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition

def generate_launch_description():
    # Declare arguments
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value="bioscara_gripper_controllers.yaml",
            description="YAML file with the controllers configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controller_manager_file",
            default_value="bioscara_gripper_controller_manager.yaml",
            description="YAML file with the controller manager configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gripper_description_package",
            default_value="bioscara_gripper_descriptions",
            description='Package that holds the robot gripper description.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gripper_macro",
            default_value="bioscara_gripper_128.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
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
            "use_mock_hardware",
            default_value="false",
            description="Start robot with fake hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_controller",
            default_value="gripper_controller",
            description="Robot controller to start.",
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
    runtime_config_package = "bioscara_gripper_bringup"
    controllers_file = LaunchConfiguration("controllers_file")
    controller_manager_file = LaunchConfiguration("controller_manager_file")
    gripper_description_package = LaunchConfiguration("gripper_description_package")
    gripper_macro = LaunchConfiguration("gripper_macro")
    prefix = LaunchConfiguration("prefix")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    robot_controller = LaunchConfiguration("robot_controller")
    gui = LaunchConfiguration("gui")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(gripper_description_package), "urdf", "scene.xacro"]
            ),
            " ",
            "prefix:=",
            prefix,
            " ",
            "use_mock_hardware:=",
            use_mock_hardware,
            " ",
            "gripper_macro:=",
            gripper_macro,
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "config", controllers_file]
    )
    controller_manager_file = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "config", controller_manager_file]
    )
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(gripper_description_package), "rviz", "display.rviz"]
    )

    # Start the ros2_control controller manager node that loads the controller(s)
    # First load the controller managers file, then the controllers file. The controller managers configuration
    # may be overridden by added controllers. 
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        parameters=[robot_description, controller_manager_file, robot_controllers],
        # prefix=['gdbserver localhost:3000']
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

    # uses the controller manager to spawn joint state broadcaster.
    # The joint_state_broadcaster is not actually a controller but is treated as such.
    # it publishes/broadcasts the joint states. 
    # Spawning simply starts the controller and puts it into active state.
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    # spawn the controller manager using the controller manager spawner.
    robot_controllers = [robot_controller]
    robot_controller_spawners = []
    for controller in robot_controllers:
        robot_controller_spawners += [
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller, "-c", "/controller_manager","--inactive"],
            )
        ]

    # Delay loading and activation of `joint_state_broadcaster` after start of ros2_control_node
    delay_joint_state_broadcaster_spawner_after_ros2_control_node = (
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=control_node,
                on_start=[
                    TimerAction(
                        period=1.0,
                        actions=[joint_state_broadcaster_spawner],
                    ),
                ],
            )
        )
    )

    # Delay rviz start after Joint State Broadcaster to avoid unnecessary warning output.
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
            rviz_node,
            ],
        )
    )

    # Delay loading and activation of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawners_after_joint_state_broadcaster_spawner = []
    for controller in robot_controller_spawners:
        delay_robot_controller_spawners_after_joint_state_broadcaster_spawner += [
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=joint_state_broadcaster_spawner,
                    on_exit=[
                        TimerAction(
                            period=3.0,
                            actions=[controller],
                        ),
                    ],
                )
            )
        ]

    return LaunchDescription(
        declared_arguments
        + [
            control_node,
            robot_state_pub_node,
            delay_rviz_after_joint_state_broadcaster_spawner,
            delay_joint_state_broadcaster_spawner_after_ros2_control_node,
        ]
        + delay_robot_controller_spawners_after_joint_state_broadcaster_spawner
    )