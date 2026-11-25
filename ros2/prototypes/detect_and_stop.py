import math
import rclpy
import cv2
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, CompressedImage
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge

# ─── Configuration ────────────────────────────────────────────────────────────
NAMESPACE      = '/T29'   # Change to your robot namespace
FORWARD_SPEED  = 0.15    # m/s
TURN_SPEED     = 0.5     # rad/s
AVOID_DISTANCE = 0.55    # metres
FRONT_ARC_DEG  = 60      # degrees either side of forward

RED_LOW1  = np.array([0,   150, 100])
RED_HIGH1 = np.array([8,  255, 255])
RED_LOW2  = np.array([172, 150, 100])
RED_HIGH2 = np.array([180, 255, 255])
MIN_PIXELS = 2000          # ← use your calibrated value


# ─── Node ─────────────────────────────────────────────────────────────────────
class DetectAndStop(Node):

    def __init__(self):
        super().__init__('detect_and_stop')

        self.bridge   = CvBridge()
        self.last_overlay = None   # stores latest frame for display

        self.pub = self.create_publisher(
            Twist, f'{NAMESPACE}/cmd_vel', 10
        )
        self.create_subscription(
            LaserScan, f'{NAMESPACE}/scan', self.scan_callback, 10
        )
        self.create_subscription(
            CompressedImage,
            f'{NAMESPACE}/oakd/rgb/image_raw/compressed',
            self.image_callback,
            10
        )
        self.create_subscription(
            Odometry,
            f'{NAMESPACE}/odom',
            self.odom_callback,
            10
        )

        # Shared state
        self.nearest_front = float('inf')
        self.nearest_left  = float('inf')
        self.nearest_right = float('inf')

        self.current_x     = 0.0
        self.current_y     = 0.0

        self.cube_detected = False
        self.state         = 'SEARCHING'

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info('Detect-and-stop node started — SEARCHING')

    # ── Laser scan callback ───────────────────────────────────────────────────
    def scan_callback(self, msg: LaserScan):
        inc    = msg.angle_increment
        arc_r  = math.radians(FRONT_ARC_DEG)
        side_r = math.radians(90)

        # index corresponding to forward (angle 0)
        front_r = math.radians(90)
        front_i = int(round(front_r / inc))

        half_a = int(round(arc_r  / inc))
        side_a = int(round(side_r / inc))
        n      = len(msg.ranges)

        def arc_min(lo: int, hi: int) -> float:
            lo = max(0, lo)
            hi = min(n - 1, hi)
            vals = [
                r for r in msg.ranges[lo:hi + 1]
                if msg.range_min < r < msg.range_max
            ]
            return min(vals) if vals else float('inf')

        self.nearest_front = arc_min(front_i - half_a, front_i + half_a)
        self.nearest_left  = arc_min(front_i,          front_i + side_a)
        self.nearest_right = arc_min(front_i - side_a, front_i)

    # ── Odometry callback ─────────────────────────────────────────────────────
    def odom_callback(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

    # ── Camera callback ───────────────────────────────────────────────────────
    def image_callback(self, msg: CompressedImage):
        img  = self.bridge.compressed_imgmsg_to_cv2(msg, 'bgr8')
        hsv  = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.bitwise_or(
            cv2.inRange(hsv, RED_LOW1, RED_HIGH1),
            cv2.inRange(hsv, RED_LOW2, RED_HIGH2)
        )
        pixels = cv2.countNonZero(mask)

        # Build overlay
        overlay = img.copy()
        overlay[mask > 0] = [0, 0, 255]

        state_colour = (0, 255, 0) if self.state == 'SEARCHING' else (0, 0, 255)
        cv2.putText(overlay, f'State: {self.state}', (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, state_colour, 2)
        cv2.putText(overlay, f'Red pixels: {pixels}', (10, 65),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.putText(overlay,
                    f'Pos: ({self.current_x:.2f}, {self.current_y:.2f}) m',
                    (10, 100),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        if pixels >= MIN_PIXELS:
            cv2.putText(overlay, 'DETECTED', (10, 140),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 3)

        # Save frame — displayed in control_loop to keep waitKey on main timer
        self.last_overlay = overlay

        # Update detection state
        if self.state == 'SEARCHING' and pixels >= MIN_PIXELS:
            self.cube_detected = True

    # ── Stop helper ───────────────────────────────────────────────────────────
    def stop(self):
        self.pub.publish(Twist())

    # ── Control loop (10 Hz) ──────────────────────────────────────────────────
    def control_loop(self):
        # Always refresh the display window here
        if self.last_overlay is not None:
            cv2.imshow('Detection', self.last_overlay)
            cv2.waitKey(1)

        if self.state == 'DONE':
            return

        if self.state == 'SEARCHING':
            if self.cube_detected:
                self.state = 'DETECTED'
                self.stop()
                self.get_logger().info('RED CUBE DETECTED — stopping')
                self.get_logger().info(
                    f'Detected position: x={self.current_x:.3f} m  '
                    f'y={self.current_y:.3f} m'
                )
                return

            cmd = Twist()
            if self.nearest_front > AVOID_DISTANCE:
                cmd.linear.x  = FORWARD_SPEED
                cmd.angular.z = 0.0
                self.get_logger().info(
                    f'Fwd | front={self.nearest_front:.2f} m '
                    f'| pos=({self.current_x:.2f}, {self.current_y:.2f})'
                )
            else:
                cmd.linear.x = 0.0
                if self.nearest_left >= self.nearest_right:
                    cmd.angular.z = TURN_SPEED
                    self.get_logger().warn('Obstacle — turning LEFT')
                else:
                    cmd.angular.z = -TURN_SPEED
                    self.get_logger().warn('Obstacle — turning RIGHT')
            self.pub.publish(cmd)

        elif self.cube_detected:
            self.stop()
            self.state = 'DONE'


# ─── Entry point ──────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = DetectAndStop()
    try:
        rclpy.spin(node)
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
