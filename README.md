# Wheelbase Verifier

A ROS 2 node that verifies a mobile robot wheelbase by commanding it through a **360-degree rotation test**, a **1-metre linear movement test**, or both in sequence — then comparing the commanded motion against the odometry reported by the platform.

## Requirements

| Dependency | Notes |
|---|---|
| ROS 2 (Humble / Iron / Jazzy) | Any distribution with `rclpy` |
| `geometry_msgs` | `Twist` on `/cmd_vel` |
| `nav_msgs` | `Odometry` on `/odom` |

The node subscribes to `/odom` and publishes to `/cmd_vel`. Make sure your robot or simulator exposes these topics before running.

## Running the node

```bash
# Run both tests in sequence (default)
ros2 run <your_package> wheelbase_verifier

# Rotation test only (360-degree spin)
ros2 run <your_package> wheelbase_verifier --ros-args -p test_mode:=rotation

# Linear movement test only (1 m forward)
ros2 run <your_package> wheelbase_verifier --ros-args -p test_mode:=linear
```

If you are running the script directly (outside a package):

```bash
python3 wheelbase_verifier.py
```

## Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `test_mode` | `string` | `both` | Which test to run: `rotation`, `linear`, or `both` |

### Tuning test values

Edit the constants near the top of `__init__` to match your platform:

| Variable | Default | Description |
|---|---|---|
| `target_radians` | `2π` (360°) | Total rotation target |
| `test_angular_velocity` | `0.3 rad/s` | Angular speed during rotation test |
| `target_distance` | `1.0 m` | Forward distance target |
| `test_linear_velocity` | `0.2 m/s` | Linear speed during movement test |

## Test sequence

### `both` (default)
```
WAITING_FOR_ODOM → ROTATING → ROTATION_DONE (1 s settle) → LINEAR → DONE
```

### `rotation`
```
WAITING_FOR_ODOM → ROTATING → DONE
```

### `linear`
```
WAITING_FOR_ODOM → LINEAR → DONE
```

## What to check after each test

**Rotation test** — Place a reference mark on the floor under the robot before starting. After the test the node logs the odometry-measured angle. Check that the robot physically returned to the same heading as the mark.

**Linear test** — Mark the robot's starting position on the floor. After the test the node logs the odometry-measured displacement. Measure the actual distance travelled with a tape measure and compare the two values.

A significant discrepancy between the odometry reading and the physical measurement indicates wheel slip, incorrect wheel radius calibration, or encoder misconfiguration.

## Stopping the node

Press `Ctrl+C` at any time. The node will publish a zero-velocity `Twist` before shutting down to ensure the robot stops.
