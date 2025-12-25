import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Paths
    pkg_project_gazebo = "/home/jeevan-koiri/Desktop/drive/dev-work/droneworx/Jeevan_agentic-ai/simulation"
    world_file_path = os.path.join(pkg_project_gazebo, "worlds", "farm.sdf")

    # Gazebo Sim
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': f'-r {world_file_path}'}.items(),
    )

    # Bridge
    # We bridge cmd_vel, odom, and tf for the robot named 'ecoweeder'
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            '/world/farm_world/model/ecoweeder/joint_state@sensor_msgs/msg/JointState@gz.msgs.Model',
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        bridge,
    ])
