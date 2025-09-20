from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def launch_setup(context, *args, **kwargs):
    # Initialize Arguments
    runtime_config_package = LaunchConfiguration("runtime_config_package")
    controllers_file = LaunchConfiguration("controllers_file")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    movement_controller_spawner = LaunchConfiguration("movement_controller")
    first_movement_controller_spawner = LaunchConfiguration("first_movement_controller")
    second_movement_controller_spawner = LaunchConfiguration("second_movement_controller")
    initial_joint_controllers = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "config", controllers_file]
    )
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "urdf", description_file]
            ),
            " ",
            "sim_ignition:=true",
            " ",
            "simulation_controllers:=",
            initial_joint_controllers,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # Nodes
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"use_sim_time": True}, robot_description],
    )

    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, initial_joint_controllers],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager", "--inactive"],
    )

    movement_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["movement_controller", "--controller-manager", "/controller_manager"],
    )

    first_movement_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["first_movement_controller", "--controller-manager", "/controller_manager",  "--inactive"],
    )

    second_movement_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["second_movement_controller", "--controller-manager", "/controller_manager",  "--inactive"],
    )

    # GZ nodes
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-string",
            robot_description_content,
            "-name",
            "panda_arm",
            "-allow_renaming",
            "true",
        ],
    )

    gz_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments={"gz_args": " -r -v 1 empty.sdf"}.items(),
    )

    # Event handlers to ensure controller_manager is ready before spawners
    delay_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=controller_manager_node,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    delay_movement_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=controller_manager_node,
            on_exit=[movement_controller_spawner],
        )
    )

    delay_first_movement_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=controller_manager_node,
            on_exit=[first_movement_controller_spawner],
        )
    )

    delay_second_movement_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=controller_manager_node,
            on_exit=[second_movement_controller_spawner],
        )
    )

    nodes_to_start = [
        robot_state_publisher_node,
        controller_manager_node,
        delay_joint_state_broadcaster_spawner,
        delay_movement_controller_spawner,
        delay_first_movement_controller_spawner,
        delay_second_movement_controller_spawner,
        gz_spawn_entity,
        gz_launch_description,
    ]

    return nodes_to_start

def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "runtime_config_package",
            default_value="franka_gazebo",
            description='Package with the controller\'s configuration in "config" folder. \
        Usually the argument is not set, it enables use of a custom setup.',
        ),
        DeclareLaunchArgument(
            "controllers_file",
            default_value="franka_controllers.yaml",
            description="YAML file with the controllers configuration.",
        ),
        DeclareLaunchArgument(
            "description_package",
            default_value="franka_description",
            description="Description package with robot URDF/XACRO files. Usually the argument \
        is not set, it enables use of a custom description.",
        ),
        DeclareLaunchArgument(
            "description_file",
            default_value="panda_arm.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        ),
        DeclareLaunchArgument(
            "movement_controller",
            default_value="movement_controller",
            description="Main robot controller",
        ),
        DeclareLaunchArgument(
            "first_movement_controller",
            default_value="first_movement_controller",
            description="First custom robot controller",
        ),
        DeclareLaunchArgument(
            "second_movement_controller",
            default_value="second_movement_controller",
            description="Second custom robot controller",
        ),
        DeclareLaunchArgument(
            "sim_ignition",
            default_value="true",
            description="Sim ignition as physics engine",
        ),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
