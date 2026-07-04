# ET-ASTLF ROS2 Ackermann Controller

This ROS2 Humble package implements the event-triggered adaptive super-twisting-like fractional controller from:

`Event-Triggered Adaptive Super-Twisting-Like Fractional Control for Path-Tracking of Autonomous Agricultural Vehicles`

The package is intentionally split into a pure Python controller core and a ROS2 adapter node. You can migrate it to the Ubuntu 22.04 Humble VM first, then adjust topic names and message adapters after the real vehicle interfaces are known.

## Default ROS2 Interfaces

- Subscribes to `/odom`: `nav_msgs/msg/Odometry`
- Subscribes to `/path`: `nav_msgs/msg/Path`
- Publishes `/ackermann_cmd`: `ackermann_msgs/msg/AckermannDriveStamped`
- Publishes `/et_astlf/debug`: `std_msgs/msg/Float64MultiArray`

If your car uses different topics, change `config/et_astlf_params.yaml`. If it uses different message types, keep `src/et_astlf_control/controller.py` unchanged and adapt `src/et_astlf_control/node.py`.

## Build In ROS2 Humble

Copy this folder into a ROS2 workspace, for example:

```bash
mkdir -p ~/ros2_ws/src
cp -r et_astlf_control ~/ros2_ws/src/
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select et_astlf_control
source install/setup.bash
```

Run:

```bash
ros2 launch et_astlf_control et_astlf.launch.py
```

Run with a CSV reference path publisher while you are still wiring up the real planner:

```bash
ros2 launch et_astlf_control et_astlf_with_csv_path.launch.py \
  csv_file:=/absolute/path/to/sample_path.csv
```

The package includes `examples/sample_path.csv` for a first topic plumbing test. In a sourced workspace, you can find the installed copy with:

```bash
ros2 pkg prefix et_astlf_control
```

## Important Parameters

The defaults are based on the paper's experiment:

- `wheelbase_m`: vehicle wheelbase, default `2.5`
- `speed_mps`: commanded longitudinal speed, default `1.0`
- `max_steering_angle_rad`: front wheel steering limit, default `0.471238898` (27 degrees)
- `c1`, `c2`: sliding surface gains, default `0.3`, `1.0`
- `epsilon`: fractional exponent offset, default `-0.42`
- `sigma`, `gamma`: event-trigger threshold parameters, default `0.4`, `0.2`
- `super_twisting_c`, `mu1`, `mu0`, `beta1_min`, `beta1_initial`: adaptive ASTLF gains

## Debug Array Layout

`/et_astlf/debug` publishes:

```text
[lateral_error_m, heading_error_rad, sliding, held_sliding,
 steering_angle_rad, virtual_control, beta1, beta2,
 triggered, trigger_count, trigger_interval_s]
```

## Notes For Migration

The current adapter assumes a path made from global XY points and derives each reference heading from adjacent path points. When you send the real topic list later, the usual changes will be:

- replace `/odom` with your localization topic
- replace `/path` with your planner/reference line topic
- replace `/ackermann_cmd` with the steering command accepted by your chassis driver
- set `wheelbase_m`, steering limit, and speed to your actual vehicle values
