# et_astlf_path_tracking

ROS2 Humble Python package for Ackermann path tracking. This version implements the continuous, non-event-triggered ASTLF controller first, while keeping the `use_event_trigger` parameter for a later ET-ASTLF implementation.

## Model And Controller

Bicycle model:

```text
x_dot = v * cos(theta)
y_dot = v * sin(theta)
theta_dot = v / L * tan(delta_f)
```

Tracking errors:

```text
Los = -(x - xd) * sin(theta_d) + (y - yd) * cos(theta_d)
theta_os = theta - theta_d
s = c1 * Los + c2 * v * sin(theta_os)
```

Continuous ASTLF control:

```text
u = -beta1 * signed_power(s, 1 + eps) + u0
u0_dot = -beta2 * signed_power(s, 1 + 2 * eps)
beta2 = 2 * c_gain * (1 + eps) * beta1
delta_f = atan(u)
```

`delta_f` is saturated to `[-max_steer_angle, max_steer_angle]`.

## Topics

Subscriptions:

- `/odom`: `nav_msgs/msg/Odometry`
- `/reference_path`: `nav_msgs/msg/Path`

Publication:

- `/ackermann_cmd`: `ackermann_msgs/msg/AckermannDriveStamped`

## Parameters

Default values are in `config/et_astlf_params.yaml`.

| Parameter | Default | Meaning |
| --- | ---: | --- |
| `wheelbase` | `0.32` | Vehicle wheelbase in meters. Reserved for model documentation and later dynamic extensions. |
| `target_speed` | `0.5` | Published longitudinal speed in m/s. Also used in the sliding surface when odometry speed is too small. |
| `max_steer_angle` | `0.45` | Steering angle limit in radians. |
| `control_rate` | `50.0` | Timer frequency in Hz. |
| `c1` | `0.3` | Lateral error gain in sliding surface. |
| `c2` | `1.0` | Heading error gain in sliding surface. |
| `eps` | `-0.42` | Fractional exponent offset. Must be in `(-0.5, 0)`. |
| `sigma` | `0.4` | Adaptive threshold gain. |
| `gamma` | `0.2` | Adaptive threshold offset. |
| `c_gain` | `0.18` | Gain used in `beta2 = 2 * c_gain * (1 + eps) * beta1`. |
| `mu1` | `0.02` | Adaptive increase/decrease rate away from the lower bound. |
| `mu0` | `0.02` | Adaptive increase rate when `beta1 <= beta1_min`. |
| `beta1_min` | `4.0` | Lower bound for `beta1`. |
| `beta1_init` | `4.48` | Initial adaptive gain. |
| `use_event_trigger` | `false` | Reserved ET-ASTLF switch. Current code runs continuous ASTLF. |
| `lookahead_distance` | `0.6` | Distance forward from nearest path point to select the tracking target. |

## Build

Install the Ackermann message package if needed:

```bash
sudo apt update
sudo apt install ros-humble-ackermann-msgs
```

Build in a ROS2 Humble workspace:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/zl2626/et_astlf_control.git
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select et_astlf_path_tracking
source install/setup.bash
```

Run:

```bash
ros2 launch et_astlf_path_tracking et_astlf_controller.launch.py
```

Run with a custom parameter file:

```bash
ros2 launch et_astlf_path_tracking et_astlf_controller.launch.py \
  params_file:=/absolute/path/to/et_astlf_params.yaml
```

## Path Handling

The node finds the nearest point in `/reference_path`, then walks forward along the path by `lookahead_distance`. The reference heading `theta_d` is computed from the selected target point to the next path point with `atan2`. At the end of the path, it uses the previous point.

## Safety Guards

The node includes protections for:

- missing `/odom`
- empty `/reference_path`
- abnormal or non-positive `dt`
- odometry speed close to zero
- invalid steering commands through saturation

## Tuning Suggestions

Start with the defaults and tune in this order:

1. Set `target_speed`, `max_steer_angle`, and `lookahead_distance` for your vehicle and test site.
2. Increase `lookahead_distance` if steering oscillates on straight paths; decrease it if turns are cut too much.
3. Tune `c1` for lateral convergence and `c2` for heading response.
4. Keep `eps` in `(-0.5, 0)`. Values near `-0.5` behave closer to super-twisting sliding mode and may be more aggressive.
5. Adjust `beta1_init`, `beta1_min`, `mu1`, and `mu0` after basic tracking is stable.

The node prints throttled debug information once per second:

```text
Los, theta_os, s, beta1, beta2, u, delta_f
```
