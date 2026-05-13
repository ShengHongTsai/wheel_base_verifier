import argparse
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

def euler_from_quaternion(quaternion):
    """
    Converts quaternion (w in last place) to euler roll, pitch, yaw.
    quaternion = [x, y, z, w]
    """
    x, y, z, w = quaternion
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return yaw

VALID_MODES = ('rotation', 'linear', 'all')

class WheelbaseVerifier(Node):
    def __init__(self, test_mode='all'):
        super().__init__('wheelbase_verifier')

        # --- User-selectable test mode (set via -test_mode CLI arg) ---
        if test_mode not in VALID_MODES:
            self.get_logger().error(
                f"Invalid test_mode '{test_mode}'. "
                f"Valid options: {VALID_MODES}. Defaulting to 'all'."
            )
            test_mode = 'all'
        self.test_mode = test_mode

        # Publisher for velocity commands
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber to odometry
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Control loop timer (runs at 20 Hz)
        self.timer = self.create_timer(0.05, self.control_loop)

        # --- Rotation test parameters ---
        self.target_radians = 2.0 * math.pi  # 360 degrees
        self.test_angular_velocity = 0.3      # rad/s (slow to prevent slip)

        # --- Linear test parameters ---
        self.target_distance = 1.0   # metres
        self.test_linear_velocity = 0.2  # m/s

        # --- Rotation odometry state ---
        self.accumulated_yaw = 0.0
        self.previous_yaw = None

        # --- Linear odometry state ---
        self.start_x = None
        self.start_y = None
        self.accumulated_distance = 0.0

        # --- Current pose (kept up-to-date by odom_callback) ---
        self.current_x = None
        self.current_y = None
        self.current_yaw = None

        # State machine:
        #   all:      WAITING_FOR_ODOM → ROTATING → ROTATION_DONE → LINEAR → DONE
        #   rotation: WAITING_FOR_ODOM → ROTATING → DONE
        #   linear:   WAITING_FOR_ODOM → LINEAR → DONE
        self.state = 'WAITING_FOR_ODOM'
        self._settle_cycles = 0  # pause between the two tests (both mode only)

        self.get_logger().info(
            f"Wheelbase Verifier Node Initialized. Test mode: '{self.test_mode}'. "
            "Waiting for /odom data..."
        )

    # ------------------------------------------------------------------
    # Odometry callback
    # ------------------------------------------------------------------

    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        self.current_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        if self.state == 'WAITING_FOR_ODOM':
            if self.test_mode in ('rotation', 'all'):
                self.previous_yaw = self.current_yaw
                self.state = 'ROTATING'
                self.get_logger().info("Odom received. Starting 360-degree rotation test!")
            else:  # linear only
                self._start_linear_test()
            return

        if self.state == 'ROTATING':
            self._update_rotation_odometry()

        if self.state == 'LINEAR':
            self._update_linear_odometry()

    # ------------------------------------------------------------------
    # Rotation odometry helpers
    # ------------------------------------------------------------------

    def _update_rotation_odometry(self):
        """Accumulate absolute yaw change, handling the ±π wrap-around."""
        delta_yaw = self.current_yaw - self.previous_yaw
        if delta_yaw > math.pi:
            delta_yaw -= 2.0 * math.pi
        elif delta_yaw < -math.pi:
            delta_yaw += 2.0 * math.pi
        self.accumulated_yaw += abs(delta_yaw)
        self.previous_yaw = self.current_yaw

    # ------------------------------------------------------------------
    # Linear odometry helpers
    # ------------------------------------------------------------------

    def _start_linear_test(self):
        """Initialise state for the linear movement test."""
        self.start_x = self.current_x
        self.start_y = self.current_y
        self.accumulated_distance = 0.0
        self.state = 'LINEAR'
        self.get_logger().info(
            f"Starting linear movement test: target {self.target_distance:.2f} m forward."
        )

    def _update_linear_odometry(self):
        """Compute straight-line displacement from the starting position."""
        if self.start_x is None:
            return
        dx = self.current_x - self.start_x
        dy = self.current_y - self.start_y
        self.accumulated_distance = math.sqrt(dx * dx + dy * dy)

    # ------------------------------------------------------------------
    # Control loop (20 Hz state machine)
    # ------------------------------------------------------------------

    def control_loop(self):
        msg = Twist()

        if self.state == 'ROTATING':
            if self.accumulated_yaw < self.target_radians:
                msg.angular.z = self.test_angular_velocity
                self.cmd_pub.publish(msg)
                if self.accumulated_yaw % 0.5 < 0.05:
                    self.get_logger().info(
                        f"Rotation progress: {math.degrees(self.accumulated_yaw):.1f} / 360.0 degrees"
                    )
            else:
                self.cmd_pub.publish(msg)  # zero-velocity stop
                self.get_logger().info("=== ROTATION TEST COMPLETE ===")
                self.get_logger().info(
                    f"Odometry registered {math.degrees(self.accumulated_yaw):.2f} degrees "
                    f"(expected 360.0). Check physical robot alignment."
                )
                if self.test_mode == 'all':
                    self.state = 'ROTATION_DONE'
                else:
                    self.state = 'DONE'
                    self.get_logger().info("=== ALL TESTS COMPLETE ===")

        elif self.state == 'ROTATION_DONE':
            self.cmd_pub.publish(msg)  # keep stopped
            self._settle_cycles += 1
            if self._settle_cycles >= 20:  # ~1 s settle time
                self._start_linear_test()

        elif self.state == 'LINEAR':
            if self.accumulated_distance < self.target_distance:
                msg.linear.x = self.test_linear_velocity
                self.cmd_pub.publish(msg)
                if self.accumulated_distance % 0.1 < 0.01:
                    self.get_logger().info(
                        f"Linear progress: {self.accumulated_distance:.3f} / "
                        f"{self.target_distance:.2f} m"
                    )
            else:
                self.cmd_pub.publish(msg)  # zero-velocity stop
                self.state = 'DONE'
                self.get_logger().info("=== LINEAR TEST COMPLETE ===")
                self.get_logger().info(
                    f"Odometry registered {self.accumulated_distance:.4f} m "
                    f"(expected {self.target_distance:.2f} m). Check physical displacement."
                )
                self.get_logger().info("=== ALL TESTS COMPLETE ===")

        elif self.state == 'DONE':
            self.cmd_pub.publish(msg)  # ensure robot stays stopped


def main():
    parser = argparse.ArgumentParser(description='Wheelbase Verifier')
    parser.add_argument(
        '-test_mode',
        choices=VALID_MODES,
        default='all',
        help='Test to run: linear, rotation, or all (default: all)',
    )
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args if ros_args else None)
    verifier = WheelbaseVerifier(test_mode=args.test_mode)

    try:
        rclpy.spin(verifier)
    except KeyboardInterrupt:
        verifier.get_logger().info('Manual interrupt received. Stopping robot...')
        stop_msg = Twist()
        verifier.cmd_pub.publish(stop_msg)
    finally:
        verifier.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
