# Launch Guide

End-to-end commands for running the AMR project. Every command assumes you've sourced ROS 2 Humble:

```bash
source /opt/ros/humble/setup.bash
# If explore_lite was built locally in your colcon workspace:
source ~/ros2_ws/install/setup.bash
```

---

## Phase 1 — Map the world

```bash
ros2 launch /home/hadi/amr_project/scripts/bringup_all.launch.py
```

What happens, in order:

| Time | Action |
|------|--------|
| t = 0 s | Gazebo loads `sdf/bigsmol.sdf` with the robot + patrol bots |
| t = 4 s | `gz_bridge.launch.py` starts the Gazebo↔ROS bridge and TFs |
| t = 8 s | EKF (`robot_localization`) starts publishing `/odometry/filtered` |
| t = 14 s | `slam_toolbox` starts publishing `/map` |
| t = 18 s | `qr_localizer.py` begins detecting QR signs (4 OpenCV windows pop up) |
| t = 22 s | Nav2 navigation stack comes up so `explore_lite` can drive |
| t = 24 s | `bumper_escape_node.py` arms |
| t = 26 s | RViz opens |
| t = 32 s | `explore_and_save_map.py` starts `explore_lite` |

Then `explore_lite` autonomously drives the robot to frontiers. When the map is fully explored (~5–10 minutes), exploration stops and the map is auto-saved to `maps/saved_map.{yaml,pgm}`. `landmarks.yaml` is continuously updated by `qr_localizer.py` while this runs.

You can shut down with Ctrl+C once you see:
```
[explore_and_save_map] Map saved successfully:
[explore_and_save_map]   maps/saved_map.yaml
[explore_and_save_map]   maps/saved_map.pgm
```

### Useful options

```bash
# Disable RViz / GUI nodes if running headless
ros2 launch /home/hadi/amr_project/scripts/bringup_all.launch.py start_rviz:=false

# Keep explore_lite running after the map is saved
ros2 launch /home/hadi/amr_project/scripts/bringup_all.launch.py keep_explore_alive:=true
```

---

## Phase 2 — Run missions on the saved map

```bash
ros2 launch /home/hadi/amr_project/scripts/phase2.launch.py
```

What happens, in order:

| Time | Action |
|------|--------|
| t = 0 s | Gazebo loads the world (with patrol bots + parked test cylinder) |
| t = 4 s | Gazebo bridge |
| t = 8 s | EKF |
| t = 12 s | AMCL + map_server load `maps/saved_map.yaml` |
| t = 20 s | Inline ROS↔Gz bridge for `/patrol_*/cmd_vel` |
| t = 22 s | Nav2 navigation stack |
| t = 24 s | Bumper escape |
| t = 26 s | RViz |
| t = 28 s | Patrol controller starts driving the obstacle bots |
| t = 32 s | Mission control GUI opens |

**Required interaction after launch**:

1. In RViz, click **2D Pose Estimate** and click on the robot's actual location in the map (with an arrow indicating its heading). AMCL needs this initial guess to localize.
2. In the **AMR Phase 2 Control Center** GUI:
   - **Go to specific location** — pick a landmark, click *Confirm and Navigate*.
   - **Assign mission** — choose a mission type (Grocery / Food / Fire emergency / …) and a target house, then click *Start Mission*.

The robot will drive to each leg in turn, with patrol bots crossing its path. To test dynamic obstacle avoidance, you can:
- Drag the parked test cylinder (`moving_cyl_1`) in the Gazebo GUI in front of the robot, OR
- Right-click in the Gazebo GUI → *Spawn* → *Cylinder* — drop a new cylinder anywhere.

### Useful options

```bash
# Disable patrols (mission testing without dynamic obstacles)
ros2 launch /home/hadi/amr_project/scripts/phase2.launch.py start_dynamic_obstacles:=false

# Use a different saved map
ros2 launch /home/hadi/amr_project/scripts/phase2.launch.py map:=/path/to/other_map.yaml
```

---

## Direct goal sending (no GUI)

```bash
# Send a single goal by landmark name (no mission scheduling)
python3 /home/hadi/amr_project/scripts/landmark_goal_sender.py HOUSE_3
```

## Run coverage tour after exploration

`coverage_tour.py` is orphaned by default — run it manually after Phase 1 ends and Nav2 is up:

```bash
python3 /home/hadi/amr_project/scripts/coverage_tour.py \
    --grid-spacing 2.0 \
    --min-clearance 0.4 \
    --rotate-at-waypoints true \
    --rotation-degrees 180 \
    --use-sim-time true
```

---

## RViz setup tips

To see what each component is doing, add these displays in RViz:

| Display | Topic | What it shows |
|---------|-------|---------------|
| Map | `/map` | The saved/loaded static map |
| Map | `/global_costmap/costmap` | Global costmap (live, includes new obstacles) |
| Map | `/local_costmap/costmap` | Local rolling costmap |
| Path | `/plan` | Current planned path from `planner_server` |
| Path | `/transformed_global_plan` | Path as the controller sees it |
| LaserScan | `/scan` | Live lidar returns |
| TF | (default) | Frames including the per-camera frames |
| RobotModel | `/robot_description` | The chassis + cameras visually |

For the camera streams, add an Image display for each:
- `/camera/image_raw`, `/camera_left/image_raw`, `/camera_right/image_raw`, `/camera_back/image_raw`

---

## Debug cheatsheet

```bash
# Check that /scan is alive and being published
ros2 topic info /scan -v        # publishers + subscribers
ros2 topic hz /scan             # publish rate (~30 Hz)

# Verify the TF chain that lets the global costmap see /scan
ros2 run tf2_ros tf2_echo map lidar_frame
ros2 run tf2_ros tf2_echo odom chassis

# See the actual costmap values around the robot
ros2 topic echo --once /global_costmap/costmap | head -50

# Check Nav2 lifecycle
ros2 service call /lifecycle_manager_navigation/is_active std_srvs/srv/Trigger

# Examine the patrol bots' state
ros2 topic echo /patrol_1/cmd_vel
ros2 topic echo /patrol_1/odom

# Inspect landmarks discovered by qr_localizer
cat param/landmarks.yaml
ros2 topic list | grep camera   # confirm all 4 image + camera_info topics exist
```

---

## Backups

During development, snapshots of modified files are kept under `.backups/<YYYYMMDD_HHMM>_<label>/`. These are excluded from the git repo via `.gitignore` but kept locally for one-command rollbacks. To restore any file from a snapshot:

```bash
cp .backups/<snapshot_dir>/<filename> <original_path>
```

For example:
```bash
cp .backups/20260511_1521_global_obs_redo/nav2_chassis_params.yaml param/nav2_chassis_params.yaml
```
