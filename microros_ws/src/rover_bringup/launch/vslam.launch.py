import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    vslam_config = os.path.join(
        get_package_share_directory('rover_bringup'),
        'config',
        'vslam.yaml'
    )

    return LaunchDescription([
        # Decompress the edge stream from the ESP32 network
        Node(
            package='image_transport',
            executable='republish',
            name='republish_compressed',
            arguments=['compressed', 'raw'],
            parameters=[{
                'in_transport': 'compressed',
                'out_transport': 'raw'
            }],
            remappings=[
                ('in/compressed', '/camera/image_raw/compressed'),
                ('out', '/camera/image_raw_decompressed')
            ]
        ),

        # SLAM Node
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            parameters=[vslam_config],
            remappings=[
                ('rgb/image', '/camera/image_raw_decompressed'),
                ('depth/image', '/camera/depth/image_raw'),
                ('rgb/camera_info', '/camera/camera_info'),
                ('odom', '/odometry/filtered')
            ],
            arguments=['-d'], # Start with a clean map database
            output='screen'
        ),
        
        # Visualizer
        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            name='rtabmap_viz',
            parameters=[vslam_config],
            remappings=[
                ('rgb/image', '/camera/image_raw_decompressed'),
                ('depth/image', '/camera/depth/image_raw'),
                ('rgb/camera_info', '/camera/camera_info'),
                ('odom', '/odometry/filtered')
            ],
            output='screen'
        ),

        Node(
            package='rtabmap_odometry',
            executable='rgbd_odometry',
            name='rgbd_odometry',
            parameters=[
                vslam_config,
                {
                    'publish_tf': False,
                    'frame_id': 'base_link',
                    'odom_frame_id': 'vo'
                }
            ],
            remappings=[
                ('rgb/image', '/camera/image_raw_decompressed'),
                ('depth/image', '/camera/depth/image_raw'),
                ('rgb/camera_info', '/camera/camera_info'),
                ('imu', '/imu/data'),
                ('odom', '/vo')
            ],
            output='screen'
        ),
    ])