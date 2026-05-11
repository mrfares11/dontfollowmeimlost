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

    # -------------------- launch arguments --------------------
    world_file = LaunchConfiguration("world")
    bridge_launch = LaunchConfiguration("bridge_launch")
    bridge_config = LaunchConfiguration("bridge_config")
    ekf_params = LaunchConfiguration("ekf_params")
    map_file = LaunchConfiguration("map")
    localization_params = LaunchConfiguration("localization_params")
    nav2_launch = LaunchConfiguration("nav2_launch")
    nav2_params = LaunchConfiguration("nav2_params")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    start_rviz = LaunchConfiguration("start_rviz")
    start_bumper_escape = LaunchConfiguration("start_bumper_escape")
    start_mission_gui = LaunchConfiguration("start_mission_gui")
    rviz_config = LaunchConfiguration("rviz_config")

    return LaunchDescription([

        DeclareLaunchArgument(
            "world",
            default_value="/home/hadi/amr_project/sdf/bigsmol.sdf",
            description="Gazebo world file to load",
        ),

        DeclareLaunchArgument(
            "bridge_launch",
            default_value="/home/hadi/amr_project/scripts/gz_bridge.launch.py",
            description="Gazebo-to-ROS bridge launch file",
        ),

        DeclareLaunchArgument(
            "bridge_config",
            default_value="/home/hadi/amr_project/param/ros2_gz_bridge_topics.yaml",
            description="ros_gz_bridge YAML config file",
        ),

        DeclareLaunchArgument(
            "ekf_params",
            default_value="/home/hadi/amr_project/param/ekf.yaml",
            description="robot_localization EKF parameter file",
        ),

        DeclareLaunchArgument(
            "map",
            default_value="/home/hadi/amr_project/maps/saved_map.yaml",
            description="Saved map YAML file from phase 1",
        ),

        DeclareLaunchArgument(
            "localization_params",
            default_value="/home/hadi/amr_project/param/nav2_chassis_localization_params.yaml",
            description="Nav2 localization params containing AMCL + map_server params",
        ),

        DeclareLaunchArgument(
            "nav2_launch",
            default_value="/home/hadi/amr_project/scripts/nav2_no_docking.launch.py",
            description="Your custom Nav2 navigation launch file",
        ),

        DeclareLaunchArgument(
            "nav2_params",
            default_value="/home/hadi/amr_project/param/nav2_chassis_params.yaml",
            description="Your custom Nav2 navigation params file",
        ),

        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use Gazebo simulation clock",
        ),

        DeclareLaunchArgument(
            "autostart",
            default_value="true",
            description="Automatically activate Nav2 lifecycle nodes",
        ),

        DeclareLaunchArgument(
            "start_rviz",
            default_value="true",
            description="Start RViz",
        ),

        DeclareLaunchArgument(
            "start_bumper_escape",
            default_value="true",
            description="Start bumper escape node",
        ),

        DeclareLaunchArgument(
            "start_mission_gui",
            default_value="true",
            description="Start the direct destination + mission GUI",
        ),

        DeclareLaunchArgument(
            "rviz_config",
            default_value="/home/hadi/amr_project/param/nav2_view.rviz",
            description="RViz config file",
        ),

        # ============================================================
        # 1. Gazebo
        # ============================================================
        ExecuteProcess(
            cmd=["gz", "sim", "-r", world_file],
            output="screen",
        ),

        # ============================================================
        # 2. Gazebo bridge
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
        # Publishes odom -> chassis
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
        # 4. Saved map + AMCL localization
        #
        # Starts map_server + amcl.
        # AMCL publishes map -> odom after the initial pose is known.
        # ============================================================
        TimerAction(
            period=12.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        PathJoinSubstitution([
                            FindPackageShare("nav2_bringup"),
                            "launch",
                            "localization_launch.py",
                        ])
                    ),
                    launch_arguments={
                        "use_sim_time": use_sim_time,
                        "map": map_file,
                        "params_file": localization_params,
                        "autostart": autostart,
                    }.items(),
                )
            ],
        ),

        # ============================================================
        # 5. Nav2 navigation
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
        # 6. Bumper escape node
        # ============================================================
        TimerAction(
            period=24.0,
            actions=[
                ExecuteProcess(
                    condition=IfCondition(start_bumper_escape),
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
        # 7. RViz
        #
        # Use 2D Pose Estimate once if AMCL needs the robot start pose.
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
        # 7a. ROS->Gz bridge for patrol cylinders
        #
        # The gz-sim-velocity-control-system plugins on patrol_cyl_1 and
        # patrol_cyl_2 listen on gz-transport topics /patrol_N/cmd_vel.
        # The main YAML bridge does not include those, so we bridge them
        # inline here. Without these entries the controller publishes
        # Twists to ROS but Gazebo never sees them.
        # ============================================================
        TimerAction(
            period=20.0,
            actions=[
                Node(
                    package='ros_gz_bridge',
                    executable='parameter_bridge',
                    name='patrol_cmd_vel_bridge',
                    arguments=[
                        '/patrol_1/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                        '/patrol_2/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                    ],
                    output='screen',
                ),
            ],
        ),

        # ============================================================
        # 7b. Patrol controller (Phase 2 dynamic obstacles)
        # ============================================================
        TimerAction(
            period=28.0,
            actions=[
                ExecuteProcess(
                    cmd=['python3', '/home/hadi/amr_project/scripts/patrol_controller.py'],
                    output='screen',
                    name='patrol_controller',
                ),
            ],
        ),

        # ============================================================
        # 8. Direct destination + mission GUI
        #
        # Opens a GUI with:
        #   - Go to specific location
        #   - Assign mission
        #
        # It reads:
        #   /home/hadi/amr_project/landmarks.yaml
        #
        # Missing mission landmarks are shown as unavailable/gray.
        # ============================================================
        TimerAction(
            period=32.0,
            actions=[
                ExecuteProcess(
                    condition=IfCondition(start_mission_gui),
                    cmd=[
                        "bash",
                        "-lc",
                        (
                            "cd /home/hadi/amr_project && "
                            "source /opt/ros/humble/setup.bash && "
                            "python3 -u /home/hadi/amr_project/scripts/landmark_mission_gui.py "
                            "--ros-args "
                            "-p use_sim_time:=true "
                            "-p landmarks_file:=/home/hadi/amr_project/param/landmarks.yaml"
                        ),
                    ],
                    output="screen",
                )
            ],
        ),
    ])
