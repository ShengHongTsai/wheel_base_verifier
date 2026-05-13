# Wheelbase Verifier

A Python script (using ROS 2 / `rclpy`) that verifies a mobile robot's wheelbase by commanding it through a **360-degree rotation test**, a **1-metre linear movement test**, or both in sequence — then comparing the commanded motion against the odometry reported by the platform.

## Requirements

| Dependency | Notes |
|---|---|
| ROS 2 (Humble / Iron / Jazzy) | Any distribution with `rclpy` |
| `geometry_msgs` | `Twist` on `/cmd_vel` |
| `nav_msgs` | `Odometry` on `/odom` |

The script subscribes to `/odom` and publishes to `/cmd_vel`. Make sure your robot or simulator exposes these topics before running.

## Running the script

```bash
# Run all tests in sequence (default)
python3 wheelbase_verifier.py

# Rotation test only (360-degree spin)
python3 wheelbase_verifier.py -test_mode rotation

# Linear movement test only (1 m forward)
python3 wheelbase_verifier.py -test_mode linear

# Explicitly run all tests
python3 wheelbase_verifier.py -test_mode all
```

## CLI argument

| Argument | Choices | Default | Description |
|---|---|---|---|
| `-test_mode` | `rotation`, `linear`, `all` | `all` | Which test(s) to run |

### Tuning test values

Edit the constants near the top of `__init__` to match your platform:

| Variable | Default | Description |
|---|---|---|
| `target_radians` | `2π` (360°) | Total rotation target |
| `test_angular_velocity` | `0.3 rad/s` | Angular speed during rotation test |
| `target_distance` | `1.0 m` | Forward distance target |
| `test_linear_velocity` | `0.2 m/s` | Linear speed during movement test |

## Test sequence

### `all` (default)
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

**Rotation test** — Place a reference mark on the floor under the robot before starting. After the test the script logs the odometry-measured angle. Check that the robot physically returned to the same heading as the mark.

**Linear test** — Mark the robot's starting position on the floor. After the test the script logs the odometry-measured displacement. Measure the actual distance travelled with a tape measure and compare the two values.

A significant discrepancy between the odometry reading and the physical measurement indicates wheel slip, incorrect wheel radius calibration, or encoder misconfiguration.

## Stopping

Press `Ctrl+C` at any time. The script will publish a zero-velocity `Twist` before shutting down to ensure the robot stops.
