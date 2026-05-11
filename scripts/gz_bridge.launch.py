from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_file = LaunchConfiguration('config_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value='/home/hadi/amr_project/param/ros2_gz_bridge_topics.yaml',
            description='Path to ros_gz_bridge YAML config file'
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gz_parameter_bridge',
            output='screen',
            parameters=[{'config_file': config_file}],
        ),

        Node(
            package='ros_gz_image',
            executable='image_bridge',
            name='gz_image_bridge',
            output='screen',
            arguments=['/camera/image_raw', '/camera_left/image_raw', '/camera_right/image_raw', '/camera_back/image_raw'],
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_static_tf',
            output='screen',
            arguments=[
                '--x', '0.8', '--y', '0.0', '--z', '0.5',
                '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
                '--frame-id', 'chassis',
                '--child-frame-id', 'lidar_frame',
            ],
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_static_tf',
            output='screen',
            arguments=[
                '--x', '0.95', '--y', '0.0', '--z', '0.58',
                '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
                '--frame-id', 'chassis',
                '--child-frame-id', 'camera_frame',
            ],
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_left_static_tf',
            output='screen',
            arguments=[
                '--x', '0.95', '--y', '0.0', '--z', '0.58',
                '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.785398',
                '--frame-id', 'chassis',
                '--child-frame-id', 'camera_left_frame',
            ],
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_right_static_tf',
            output='screen',
            arguments=[
                '--x', '0.95', '--y', '0.0', '--z', '0.58',
                '--roll', '0.0', '--pitch', '0.0', '--yaw', '-0.785398',
                '--frame-id', 'chassis',
                '--child-frame-id', 'camera_right_frame',
            ],
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_back_static_tf',
            output='screen',
            arguments=[
                '--x', '-0.4', '--y', '0.0', '--z', '0.58',
                '--roll', '0.0', '--pitch', '0.0', '--yaw', '3.141593',
                '--frame-id', 'chassis',
                '--child-frame-id', 'camera_back_frame',
            ],
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='imu_static_tf',
            output='screen',
            arguments=[
                '--x', '0.0', '--y', '0.0', '--z', '0.15',
                '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
                '--frame-id', 'chassis',
                '--child-frame-id', 'imu_frame',
            ],
        ),
    ])
