# `offboard_ctrl`

ROS 2 nodes that publish PX4 offboard-control setpoints.  The active scouting
integration uses `offboard_follower`, which follows the `nav_msgs/msg/Path`
published by D* Lite.

This package does not start PX4, the Micro XRCE-DDS Agent, mapping, or D* Lite.
Those processes must already be running and connected before starting a node
from this package.

## Built executables

| Executable | Purpose | Scouting-stack status |
| --- | --- | --- |
| `offboard_follower` | Generates attitude/thrust offboard setpoints that track D* Lite path corners. | Active |
| `autonomy_controller` | Separate signal-driven takeoff/translate/land controller. | Not launched by the scouting stack |

The ACADOS-based `offboard_attitude` executable is optional and is built only
when `BUILD_ACADOS_CONTROLLER=ON`; it is not part of the scouting stack.

## Build

`px4_msgs` must be available in the same ROS overlay.  On a system where the
other workspace dependencies have already been installed:

```bash
source /opt/ros/jazzy/setup.bash
cd <workspace>
colcon build --packages-select offboard_ctrl
source install/setup.bash
```

For the Jetson, source `/opt/ros/humble/setup.bash` instead.  PX4, its DDS
bridge, and the matching `px4_msgs` definitions are external prerequisites.

## Run the D* Lite follower

Start the PX4/DDS connection and the mapping/D* Lite path publisher first,
then run:

```bash
ros2 run offboard_ctrl offboard_follower --ros-args \
  -p use_lio_visual_odometry:=false
```

Use hover mode to verify attitude control before path following:

```bash
ros2 run offboard_ctrl offboard_follower --ros-args \
  -p hover_test:=true \
  -p hover_altitude:=0.5 \
  -p use_lio_visual_odometry:=false
```

## `offboard_follower` interfaces

| Direction | Topic | Type | Notes |
| --- | --- | --- | --- |
| Subscribes | `/dstar_path` | `nav_msgs/msg/Path` | D* Lite path; pose 1 is normally the next corner target. |
| Subscribes | `/fmu/out/vehicle_odometry` | `px4_msgs/msg/VehicleOdometry` | PX4 state used for the controller. |
| Subscribes | `/fmu/out/vehicle_status_v4` | `px4_msgs/msg/VehicleStatus` | Used to detect arming state. |
| Subscribes, optional | `/odometry` | `nav_msgs/msg/Odometry` | Used only when `use_lio_visual_odometry:=true`. |
| Publishes | `/fmu/in/offboard_control_mode` | `px4_msgs/msg/OffboardControlMode` | Declares attitude-control offboard mode. |
| Publishes | `/fmu/in/vehicle_attitude_setpoint_v1` | `px4_msgs/msg/VehicleAttitudeSetpoint` | Attitude and thrust setpoint, published at 10 Hz. |
| Publishes, optional | `/fmu/in/vehicle_visual_odometry` | `px4_msgs/msg/VehicleOdometry` | Forwarded LIO odometry when enabled. |

The follower advertises `/fmu/in/vehicle_command`, but its current path-following
flow does not issue arm or mode commands.  Arm and enter offboard mode using
the established PX4 test procedure.

## Parameters

| Parameter | Default | Effect |
| --- | --- | --- |
| `hover_test` | `false` | Hold a hover target instead of accepting D* path corners. |
| `hover_altitude` | `0.5` | ENU hover/flight altitude in metres. |
| `waypoint_radius` | `1.0` | Distance at which the follower accepts a corner; it also advances once the vehicle projects past the corner segment. |
| `use_lio_visual_odometry` | `false` | Subscribe to `/odometry` and forward it to PX4 as visual odometry. |

## Current tuning and limitation

The current D* Lite tmux launcher uses `hard_clearance_radius:=0.70` m and
`wall_cost_radius:=1.0` m.  This gets the vehicle through most of the SITL
maze, but it still fails at certain corners.  Further tuning of clearance,
wall-cost weights, and follower corner behavior is required; the planner's
clearance cost is not a real-time safety filter.

There is currently no CBF or other independent collision-avoidance backstop in
the scouting stack.  Test changes in simulation before using hardware.
