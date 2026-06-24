import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory('rover_bringup'),
        'config',
        'evofox.yaml'
    )

    ekf_config_path = os.path.join(
        get_package_share_directory('rover_bringup'),
        'config',
        'ekf.yaml'
    )

    return LaunchDescription([
        # 1. The micro-ROS Wi-Fi Bridge
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            arguments=['udp4', '--port', '8888'],
            output='screen'
        ),

        # 2. The Raw Joystick Reader
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen'
        ),

        # 3. The Kinematics Translator (Using your EvoFox layout)
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_node',
            parameters=[config_path],
            output='screen'
        ),

        # 4. The 20Hz Data Choke
        Node(
            package='topic_tools',
            executable='throttle',
            name='throttle_cmd_vel',
            arguments=['messages', '/cmd_vel', '20.0', '/cmd_vel_throttled'],
            output='screen'
        ),

        # 5. The Madgwick Filter (Raw IMU -> Quaternions)
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter',
            parameters=[{
                'use_mag': False,       # MPU6500 has no magnetometer
                'stateless': False,     # Maintain state between readings
                'publish_tf': False,     # Prevent TF tree conflicts with robot_localization
                'orientation_stddev': 0.05
            }],
            # Remappings aren't strictly required since these are the package defaults, 
            # but they explicitly show the data flow.
            remappings=[
                ('/imu/data_raw', '/imu/data_raw'),
                ('/imu/data', '/imu/data')
            ],
            output='screen'
        ),

	# 6. The Odometry Translator
        Node(
            package='rover_odom',
            executable='tick_to_odom',
            name='tick_to_odom',
            output='screen'
        ),

       # 7. The Extended Kalman Filter (Sensor Fusion)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config_path]
        ),

       # 8. Bridge the IMU to the Chassis
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_imu_tf',
            # Arguments: [X, Y, Z, Yaw, Pitch, Roll, Parent, Child]
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link']
        )

    ])
