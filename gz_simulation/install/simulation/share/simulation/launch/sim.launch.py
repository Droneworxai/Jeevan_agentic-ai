import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Paths
    pkg_project_gazebo = get_package_share_directory('simulation')
    world_file_path = os.path.join(pkg_project_gazebo, "worlds", "farm.sdf")

    # Gazebo Sim
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': f'-r {world_file_path}'}.items(),
    )

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='eco_parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',
            '/world/farm_world/model/ecoweeder/joint_state@sensor_msgs/msg/JointState@gz.msgs.Model',
        ],
        output='screen'
    )

    # Rosbridge Server
    rosbridge = IncludeLaunchDescription(
        AnyLaunchDescriptionSource([os.path.join(
            get_package_share_directory('rosbridge_server'), 'launch', 'rosbridge_websocket_launch.xml')]),
        launch_arguments={'delay_between_messages': '0.0'}.items()
    )

    # Mission Planner Node
    mission_planner = Node(
        package='simulation',
        executable='mission_planner',
        name='eco_mission_planner',
        output='screen'
    )

    # Boundary Manager
    boundary_manager = Node(
        package='simulation',
        executable='boundary_manager',
        name='eco_boundary_manager',
        output='screen'
    )

    # Weed Manager
    weed_manager = Node(
        package='simulation',
        executable='weed_manager',
        name='eco_weed_manager',
        output='screen'
    )

    # Mission Monitor
    mission_monitor = Node(
        package='simulation',
        executable='mission_monitor',
        name='eco_mission_monitor',
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        bridge,
        rosbridge,
        mission_planner,
        boundary_manager,
        weed_manager,
        mission_monitor,
    ])
