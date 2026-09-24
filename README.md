# AMR_Fleet

A multi-robot simulation for four differential-drive AMRs on ROS 2 Humble. Each robot runs its own Nav2 stack inside one shared Gazebo maze. A fleet manager node watches all four robots and stops robots from driving into each other where their paths cross.

## How it works

1. Each robot is spawned in its own namespace (`/robot1` to `/robot4`) with its own AMCL, planner, controller and behaviour tree.
2. You give each robot a goal. The planner (a Dijkstra planner from `amr_planner_plugins`) produces a path.
3. The fleet manager reads every robot's path and position. If two paths pass within 2 m of each other at nearly the same time, it lets one robot go first and pauses the other until the crossing is clear.
4. The robot that goes first is chosen in this order:
   1. The one that reaches the crossing point first.
   2. The one that reaches the 2 m boundary around the crossing first.
   3. The one that is closer to its own goal.
   4. The one with the lower robot number.
5. Two helper nodes keep the run stable:
   - `amcl_watchdog_node` re-seeds a robot's localisation if AMCL stops publishing.
   - The fleet manager restarts a robot's `controller_server` if it stops updating (see Known problems) and sends the goal again.

## Packages

| Package | What it does |
|---|---|
| `amr_description` | Robot model (URDF/xacro, meshes) and the maze world |
| `amr_description_bringup` | Launch files, map, Nav2 and SLAM parameters |
| `amr_planner_plugins` | A*, Dijkstra and RRT global planners, plus a goal checker that ignores an invalid goal pose |
| `amr_fleet_msgs` | `FleetRobotState` message the fleet manager publishes |
| `amr_fleet_manager` | `fleet_manager_node` and `amcl_watchdog_node` |

## Requirements

- Ubuntu with ROS 2 Humble
- Nav2, `ros_gz` and Ignition Gazebo (Fortress)
- CycloneDDS: `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`

Four robots need larger network buffers than the Linux default. Run this once after every reboot:

```bash
sudo sysctl -w net.core.rmem_max=67108864 net.core.rmem_default=67108864 \
  net.core.wmem_max=67108864 net.core.wmem_default=67108864
```

## Build

```bash
cd ~/AMR_Fleet
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Run

Open a separate terminal for each step. In every terminal run these two lines first:

```bash
cd ~/AMR_Fleet && source /opt/ros/humble/setup.bash && source install/setup.bash
```

**1. Simulation.** Wait until "Managed nodes are active" has been printed four times (one to two minutes).

```bash
ros2 launch amr_description_bringup multi_robot_bringup.launch.py
```

Options: `headless:=true` (no Gazebo window), `use_rviz:=false` (no RViz).

**2. AMCL watchdog**

```bash
ros2 run amr_fleet_manager amcl_watchdog_node --ros-args -p use_sim_time:=true
```

**3. Fleet manager**

```bash
ros2 run amr_fleet_manager fleet_manager_node --ros-args -p use_sim_time:=true
```

Wait about two minutes before sending goals. If the fleet manager prints "appears frozen ... Restarting it", wait for that robot to come back first.

**4. Send goals.** Either use the goal buttons in RViz, or send one from a terminal:

```bash
ros2 action send_goal /robot1/navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 4.0, y: -0.5}, orientation: {w: 1.0}}}}"
```

Do not set a goal on top of another robot. The robots start at:

| Robot | x | y |
|---|---|---|
| robot1 | -12.2 | 9.95 |
| robot2 | 2.65 | -13.3 |
| robot3 | 3.90 | 5.25 |
| robot4 | 3.90 | -5.85 |

**Check a robot's real position.** A goal reported as SUCCEEDED should be compared with where the robot actually is:

```bash
ros2 run tf2_ros tf2_echo map robot1/base_link \
  --ros-args -r /tf:=/robot1/tf -r /tf_static:=/robot1/tf_static -p use_sim_time:=true
```

**Stop.** Press Ctrl+C in the fleet manager, watchdog and simulation terminals. If Gazebo keeps running, find it with `pgrep -af "ign gazebo"` and stop it with `kill -9 <pid>`.

## Single robot

```bash
ros2 launch amr_description_bringup amr_bringup.launch.py
```

## Known problems

- After startup, a robot's `controller_server` can stop receiving TF updates. Its local costmap then stops updating and its goals fail. AMCL and TF are fine when this happens. The cause is not known. The fleet manager detects it (the local costmap footprint timestamp falls more than 12 s behind the clock), restarts that controller, and sends the goal again. A restart takes about 30 to 60 seconds.
- Goals placed on another robot's position are rejected by the planner as "occupied".
- When a robot is paused for a crossing, it waits where it is. If that spot is another robot's goal, the two can block each other.
