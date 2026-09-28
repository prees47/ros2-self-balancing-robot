import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro

def generate_launch_description():
    pkg_share = FindPackageShare('self_balancing_robot').find('self_balancing_robot')
    
    # Path containing the 'self_balancing_robot' package folder
    gz_resource_path = os.path.dirname(pkg_share)
    
    xacro_file = os.path.join(pkg_share, 'urdf', 'self_balancing_robot.urdf.xacro')
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    return LaunchDescription([
        # Pass Gazebo resource paths directly to GUI and Sim processes
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', gz_resource_path),
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', gz_resource_path),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_raw, 'use_sim_time': True}]
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    FindPackageShare('ros_gz_sim').find('ros_gz_sim'),
                    'launch',
                    'gz_sim.launch.py'
                )
            ]),
            launch_arguments={'gz_args': '-r empty.sdf'}.items()
        ),
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-topic', 'robot_description', '-name', 'balance_bot', '-z', '0.1'],
            output='screen'
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                '/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
                '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry'
            ],
            output='screen'
        )
    ])