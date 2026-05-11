from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    world_file = LaunchConfiguration("world")
    bridge_launch = LaunchConfiguration("bridge_launch")
    bridge_config = LaunchConfiguration("bridge_config")
    ekf_params = LaunchConfiguration("ekf_params")
    slam_params = LaunchConfiguration("slam_params")
    nav2_launch = LaunchConfiguration("nav2_launch")
    nav2_params = LaunchConfiguration("nav2_params")
    use_sim_time = LaunchConfiguration("use_sim_time")
    start_rviz = LaunchConfiguration("start_rviz")
    start_explore = LaunchConfiguration("start_explore")
    start_qr_localizer = LaunchConfiguration("start_qr_localizer")
    rviz_config = LaunchConfiguration("rviz_config")

    return LaunchDescription([

        DeclareLaunchArgument(
            "world",
            default_value="/home/hadi/amr_project/sdf/bigsmol.sdf",
        ),

        DeclareLaunchArgument(
            "bridge_launch",
            default_value="/home/hadi/amr_project/scripts/gz_bridge.launch.py",
        ),

        DeclareLaunchArgument(
            "bridge_config",
            default_value="/home/hadi/amr_project/param/ros2_gz_bridge_topics.yaml",
        ),

        DeclareLaunchArgument(
            "ekf_params",
            default_value="/home/hadi/amr_project/param/ekf.yaml",
        ),

        DeclareLaunchArgument(
            "slam_params",
            default_value="/home/hadi/amr_project/param/mapper_params_online_async.yaml",
        ),

        DeclareLaunchArgument(
            "nav2_launch",
            default_value="/home/hadi/amr_project/scripts/nav2_no_docking.launch.py",
        ),

        DeclareLaunchArgument(
            "nav2_params",
            default_value="/home/hadi/amr_project/param/nav2_chassis_params.yaml",
        ),

        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
        ),

        DeclareLaunchArgument(
            "start_rviz",
            default_value="true",
        ),

        DeclareLaunchArgument(
            "start_explore",
            default_value="true",
        ),

        DeclareLaunchArgument(
            "start_qr_localizer",
            default_value="true",
        ),

        DeclareLaunchArgument(
            "rviz_config",
            default_value="/home/hadi/amr_project/param/nav2_view.rviz",
        ),

        # ============================================================
        # 1. Gazebo
        # -r means run automatically, not paused
        # ============================================================
        ExecuteProcess(
            cmd=["gz", "sim", "-r", world_file],
            output="screen",
        ),

        # ============================================================
        # 2. Gazebo bridge
        # Bridges /clock, /cmd_vel, /odom, /scan, camera, IMU, bumpers, etc.
        # ============================================================
        TimerAction(
            period=4.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(bridge_launch),
                    launch_arguments={
                        "config_file": bridge_config,
                    }.items(),
                )
            ],
        ),

        # ============================================================
        # 3. EKF
        # Publishes filtered odometry, usually /odometry/filtered
        # ============================================================
        TimerAction(
            period=8.0,
            actions=[
                Node(
                    package="robot_localization",
                    executable="ekf_node",
                    name="ekf_filter_node",
                    output="screen",
                    parameters=[
                        ekf_params,
                        {"use_sim_time": use_sim_time},
                    ],
                )
            ],
        ),

        # ============================================================
        # 4. SLAM Toolbox
        # Creates map frame and publishes /map during exploration
        # ============================================================
        TimerAction(
            period=14.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        PathJoinSubstitution([
                            FindPackageShare("slam_toolbox"),
                            "launch",
                            "online_async_launch.py",
                        ])
                    ),
                    launch_arguments={
                        "slam_params_file": slam_params,
                        "use_sim_time": use_sim_time,
                    }.items(),
                )
            ],
        ),

        # ============================================================
        # 4b. QR localizer
        #
        # Detects QR landmarks while SLAM is building the map.
        # It should save landmarks to:
        #   /home/hadi/amr_project/landmarks.yaml
        #
        # IMPORTANT:
        # In qr_localizer.py, use:
        #   LANDMARKS_FILE = os.path.expanduser("~/amr_project/landmarks.yaml")
        #   PREFERRED_REF_FRAMES = ["map"]
        # ============================================================
        TimerAction(
            period=18.0,
            actions=[
                ExecuteProcess(
                    condition=IfCondition(start_qr_localizer),
                    cmd=[
                        "bash",
                        "-lc",
                        (
                            "cd /home/hadi/amr_project && "
                            "source /opt/ros/humble/setup.bash && "
                            "python3 -u /home/hadi/amr_project/scripts/qr_localizer.py"
                        ),
                    ],
                    output="screen",
                )
            ],
        ),

        # ============================================================
        # 5. Nav2
        # Give EKF, SLAM, and QR localizer time before Nav2 starts
        # ============================================================
        TimerAction(
            period=22.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(nav2_launch),
                    launch_arguments={
                        "use_sim_time": use_sim_time,
                        "params_file": nav2_params,
                    }.items(),
                )
            ],
        ),

        # ============================================================
        # 5b. Bumper escape node
        # Listens for bumper contacts, publishes /cmd_vel to escape
        # ============================================================
        TimerAction(
            period=24.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        "bash",
                        "-lc",
                        (
                            "cd /home/hadi/amr_project && "
                            "source /opt/ros/humble/setup.bash && "
                            "python3 -u /home/hadi/amr_project/scripts/bumper_escape_node.py"
                        ),
                    ],
                    output="screen",
                )
            ],
        ),

        # ============================================================
        # 6. RViz
        # ============================================================
        TimerAction(
            period=26.0,
            actions=[
                Node(
                    condition=IfCondition(start_rviz),
                    package="rviz2",
                    executable="rviz2",
                    name="rviz2",
                    output="screen",
                    arguments=["-d", rviz_config],
                    parameters=[
                        {"use_sim_time": use_sim_time},
                    ],
                )
            ],
        ),

        # ============================================================
        # 7. Explore Lite + automatic map saving
        #
        # This wrapper runs explore_lite and watches for:
        #   "All frontiers traversed/tried out, stopping."
        #   "Exploration stopped."
        #
        # Once detected, it saves the map to:
        #   /home/hadi/amr_project/maps/saved_map.yaml
        #   /home/hadi/amr_project/maps/saved_map.pgm
        # ============================================================
        TimerAction(
            period=32.0,
            actions=[
                ExecuteProcess(
                    condition=IfCondition(start_explore),
                    cmd=[
                        "bash",
                        "-lc",
                        (
                            "cd /home/hadi/amr_project/scripts && "
                            "source /opt/ros/humble/setup.bash && "
                            "source /home/hadi/ros2_ws/install/setup.bash && "
                            "python3 -u ./explore_and_save_map.py "
                            "--explore-params /home/hadi/amr_project/param/explore_params.yaml "
                            "--maps-dir ../maps "
                            "--map-name saved_map "
                            "--map-topic /map "
                            "--save-delay 2.0 "
                            "--save-timeout 10.0 "
                            "--use-sim-time true"
                        ),
                    ],
                    output="screen",
                )
            ],
        ),
    ])
