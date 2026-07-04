# ET-ASTLF ROS2 Package Design

## Goal

Build a ROS2 Humble Python package that reproduces the event-triggered adaptive super-twisting-like fractional path-tracking controller from the paper for an Ackermann vehicle, with default topic names that can be changed after the real vehicle interfaces are known.

## Architecture

The package is split into a pure Python control core and a ROS2 adapter node. The control core implements the paper equations, adaptive gain update, event-trigger condition, steering saturation, and path-error geometry. The ROS2 adapter subscribes to odometry and a planned path, computes lateral and heading error, calls the control core, and publishes an Ackermann steering command.

## Default Interfaces

- Subscribe: `/odom` as `nav_msgs/msg/Odometry`
- Subscribe: `/path` as `nav_msgs/msg/Path`
- Publish: `/ackermann_cmd` as `ackermann_msgs/msg/AckermannDriveStamped`
- Optional debug publish: `/et_astlf/debug` as `std_msgs/msg/Float64MultiArray`

These are defaults only. The node exposes topic names as ROS parameters so migration can begin before the final vehicle interface is known.

## Controller

The control state is:

- `x1 = lateral offset`
- `x2 = heading error`
- `s = c1 * x1 + c2 * v * sin(x2)`

The virtual input is `u = tan(delta_f)`, and the published steering angle is `delta_f = atan(u)` after saturating `u` by the configured steering limit. The event-triggered ASTLF controller uses the paper's signed power notation, adaptive `beta1`, derived `beta2`, and event threshold parameters `sigma` and `gamma`.

## Parameters

Default parameters come from the paper's experiment where possible:

- `wheelbase_m = 2.5`
- `speed_mps = 1.0`
- `max_steering_angle_rad = 0.471238898` (27 degrees)
- `c1 = 0.3`
- `c2 = 1.0`
- `epsilon = -0.42`
- `sigma = 0.4`
- `gamma = 0.2`
- `super_twisting_c = 0.18`
- `mu1 = 0.02`
- `mu0 = 0.02`
- `beta1_min = 4.0`
- `beta1_initial = 4.48`
- `reference_lookahead_m = 0.0`
- `control_rate_hz = 50.0`

## Testing

Local tests cover the pure Python parts that can run without ROS installed: signed fractional power, first controller update, event hold behavior, steering saturation, quaternion yaw extraction, heading normalization, and lateral-error geometry. ROS2 launch/package files are verified by syntax and file layout locally, then can be built with `colcon build` inside the Ubuntu 22.04 Humble VM.
