import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

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

class WheelbaseVerifier(Node):
    def __init__(self):
        super().__init__('wheelbase_verifier')
        
        # Publisher for velocity commands
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscriber to vendor's odometry
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        # Control loop timer (runs at 20Hz)
        self.timer = self.create_timer(0.05, self.control_loop)

        # State variables
        self.target_radians = 2.0 * math.pi  # Target: exactly 360 degrees
        self.accumulated_yaw = 0.0
        self.previous_yaw = None
        self.state = 'WAITING_FOR_ODOM' # States: WAITING_FOR_ODOM, ROTATING, DONE
        
        # Test parameters
        self.test_angular_velocity = 0.3  # rad/s (slow and steady to prevent slip)

        self.get_logger().info("Wheelbase Verifier Node Initialized. Waiting for /odom data...")

    def odom_callback(self, msg):
        # Extract quaternion and calculate current yaw
        q = msg.pose.pose.orientation
        current_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

        if self.previous_yaw is None:
            self.previous_yaw = current_yaw
            self.state = 'ROTATING'
            self.get_logger().info("Odom received. Starting 360-degree rotation test!")
            return

        # Calculate delta yaw and handle the -pi to pi wrap-around singularity
        delta_yaw = current_yaw - self.previous_yaw
        if delta_yaw > math.pi:
            delta_yaw -= 2.0 * math.pi
        elif delta_yaw < -math.pi:
            delta_yaw += 2.0 * math.pi

        # Accumulate the absolute rotation
        self.accumulated_yaw += abs(delta_yaw)
        self.previous_yaw = current_yaw

    def control_loop(self):
        msg = Twist()

        if self.state == 'ROTATING':
            if self.accumulated_yaw < self.target_radians:
                # Keep rotating
                msg.angular.z = self.test_angular_velocity
                self.cmd_pub.publish(msg)
                
                # Log progress every ~0.5 radians to avoid terminal spam
                if self.accumulated_yaw % 0.5 < 0.05:
                    degrees_done = math.degrees(self.accumulated_yaw)
                    self.get_logger().info(f"Progress: {degrees_done:.1f} / 360.0 degrees")
            else:
                # Target reached! Send hard stop.
                msg.angular.z = 0.0
                self.cmd_pub.publish(msg)
                self.state = 'DONE'
                self.get_logger().info("=== TEST COMPLETE ===")
                self.get_logger().info(f"Odometry registered full {math.degrees(self.accumulated_yaw):.2f} degree turn.")
                self.get_logger().info("Please check the physical robot's alignment against your floor marks.")
                
        elif self.state == 'DONE':
            # Ensure the robot stays stopped
            msg.angular.z = 0.0
            self.cmd_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    verifier = WheelbaseVerifier()
    
    try:
        rclpy.spin(verifier)
    except KeyboardInterrupt:
        verifier.get_logger().info("Manual interrupt received. Stopping robot...")
        # Send zero velocity before shutting down
        stop_msg = Twist()
        verifier.cmd_pub.publish(stop_msg)
    finally:
        verifier.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()