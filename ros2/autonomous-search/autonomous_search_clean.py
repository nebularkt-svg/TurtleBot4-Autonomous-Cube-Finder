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
NAMESPACE      = 'T29'   # Change to your robot namespace
FORWARD_SPEED  = 0.14    # m/s
TURN_SPEED     = 0.5     # rad/s
AVOID_DISTANCE = 0.30    # metres — obstacle avoidance threshold

WALL_DISTANCE  = 0.30    # metres — target distance from right wall
WALL_KP        = 1.2     # proportional gain for wall following
WALL_KD        = 0.3     # derivative gain — damps oscillation
WALL_LOOKAHEAD = 0.6     # weight for front-right lookahead term
WALL_DT        = 0.1     # matches control loop timer interval

FRONT_ARC_DEG  = 60      # degrees either side of forward (total 120°)

APPROACH_SPEED      = 0.08    # m/s — slow creep toward cube
APPROACH_KP_ANG     = 0.005   # proportional gain: pixel offset → angular.z
APPROACH_STOP_DIST  = 0.30    # metres — distance threshold to stop approaching
APPROACH_STOP_PIX   = 10000   # pixels — pixel count threshold to stop approaching

STUCK_TIME          = 30.0    # seconds without movement before escape
STUCK_DIST_THRESH   = 0.05    # metres — less than this = considered not moving
ESCAPE_ANGLE        = 135     # degrees to turn when stuck

# Circling detection
CIRCLE_CHECK_INTERVAL = 60.0  # seconds between net-displacement checks
CIRCLE_DISP_THRESH    = 0.30  # metres — less than this in interval = circling

RETURN_KP_ANG  = 0.5     # proportional gain for return heading
RETURN_KP_LIN  = 0.6     # proportional gain for return speed
RETURN_MAX_SPD = 0.18    # m/s — max speed while returning
RETURN_THRESH  = 0.12    # metres — close enough to (0,0)
RETURN_AVOID_TURN  = TURN_SPEED * 1.5  # angular speed during return obstacle avoidance
RETURN_WALL_THRESH = 1.0               # metres — switch from wall-follow to P-controller

CORNER_FRONT_DIST  = 0.25   # metres — front threshold for corner detection
CORNER_SIDE_DIST   = 0.25   # metres — side threshold for corner detection
BACKUP_SPEED       = -0.00  # m/s — reverse speed when cornered
BACKUP_TIME        = 0.0    # seconds to back up before turning

RED_LOW1  = np.array([0,   150, 100])
RED_HIGH1 = np.array([8,   255, 255])
RED_LOW2  = np.array([172, 150, 100])
RED_HIGH2 = np.array([180, 255, 255])
MIN_PIXELS = 1500
CAMERA_HFOV_DEG = 69.0   # OAK-D RGB horizontal field of view (degrees)


# ─── Node ─────────────────────────────────────────────────────────────────────
class DetectAndStop(Node):

    def __init__(self):
        super().__init__('detect_and_stop')

        self.bridge       = CvBridge()
        self.last_overlay = None
        self.img_width    = 640

        # Approach tracking
        self.cube_cx     = None
        self.cube_pixels = 0

        self.pub = self.create_publisher(
            Twist, f'{NAMESPACE}/cmd_vel', 10
        )
        self.create_subscription(
            LaserScan, f'{NAMESPACE}/scan', self.scan_callback, 10
        )
        self.create_subscription(
            CompressedImage,
            f'{NAMESPACE}/oakd/rgb/image_raw/compressed',
            self.image_callback, 10
        )
        self.create_subscription(
            Odometry,
            f'{NAMESPACE}/odom',
            self.odom_callback, 10
        )

        # Laser distances
        self.nearest_front       = float('inf')
        self.nearest_left        = float('inf')
        self.nearest_right       = float('inf')
        self.nearest_front_right = float('inf')   # lookahead arc

        # Wall PD state
        self.wall_prev_error = 0.0

        # Odometry
        self.current_x   = 0.0
        self.current_y   = 0.0
        self.current_yaw = 0.0

        # State machine
        self.cube_detected   = False
        self.state           = 'SEARCHING'
        self.turn_target_yaw = None

        # Stuck detection
        self.last_moved_x      = 0.0
        self.last_moved_y      = 0.0
        self.last_moved_time   = self.get_clock().now()
        self.prev_state        = None
        self.escape_target_yaw = None

        # Circling detection (net displacement check)
        self.circle_check_pos  = (0.0, 0.0)
        self.circle_check_time = self.get_clock().now()

        # Corner escape
        self.backup_start_time  = None
        self.corner_turn_target = None

        # Reacquire (lost cube during approach)
        self.reacquire_start_time = None
        self.reacquire_timeout    = 10.0   # seconds to sweep before giving up
        self.reacquire_start_yaw  = None   # yaw when reacquire started
        self.reacquire_direction  = 1      # +1 = left, -1 = right

        self.start_time = self.get_clock().now()

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info('Detect-and-stop node started — SEARCHING')

    # ── Laser scan callback ───────────────────────────────────────────────────
    def scan_callback(self, msg: LaserScan):
        inc    = msg.angle_increment
        arc_r  = math.radians(FRONT_ARC_DEG)
        side_r = math.radians(90)

        front_r = math.radians(90)
        front_i = int(round(front_r / inc))
        half_a  = int(round(arc_r  / inc))
        side_a  = int(round(side_r / inc))
        n       = len(msg.ranges)

        def arc_min(lo, hi):
            lo = max(0, lo); hi = min(n - 1, hi)
            vals = [r for r in msg.ranges[lo:hi + 1]
                    if msg.range_min < r < msg.range_max]
            return min(vals) if vals else float('inf')

        self.nearest_front       = arc_min(front_i - half_a, front_i + half_a)
        self.nearest_left        = arc_min(front_i,          front_i + side_a)
        self.nearest_right       = arc_min(front_i - side_a, front_i)
        # Lookahead: front-right quadrant (between forward and right)
        self.nearest_front_right = arc_min(front_i - side_a, front_i - half_a)

    # ── Odometry callback ─────────────────────────────────────────────────────
    def odom_callback(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny, cosy)

    # ── Camera callback ───────────────────────────────────────────────────────
    def image_callback(self, msg: CompressedImage):
        img  = self.bridge.compressed_imgmsg_to_cv2(msg, 'bgr8')
        h, w = img.shape[:2]
        self.img_width = w

        hsv  = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.bitwise_or(
            cv2.inRange(hsv, RED_LOW1, RED_HIGH1),
            cv2.inRange(hsv, RED_LOW2, RED_HIGH2)
        )
        pixels = cv2.countNonZero(mask)
        self.cube_pixels = pixels

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        self.cube_cx = None
        overlay = img.copy()
        overlay[mask > 0] = [0, 0, 255]

        if contours:
            largest = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest) >= MIN_PIXELS:
                M = cv2.moments(largest)
                if M['m00'] > 0:
                    self.cube_cx = int(M['m10'] / M['m00'])
                x, y, bw, bh = cv2.boundingRect(largest)
                cv2.rectangle(overlay, (x, y), (x+bw, y+bh), (0, 255, 0), 2)
                if self.cube_cx is not None:
                    cv2.circle(overlay, (self.cube_cx, y + bh // 2), 6,
                               (0, 255, 0), -1)

        state_colour = (0, 255, 0) if self.state == 'SEARCHING' else (0, 0, 255)
        cv2.putText(overlay, f'State: {self.state}', (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, state_colour, 2)
        cv2.putText(overlay, f'Red pixels: {pixels}', (10, 65),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.putText(overlay,
                    f'Pos: ({self.current_x:.2f}, {self.current_y:.2f}) m  '
                    f'yaw: {math.degrees(self.current_yaw):.1f}deg  '
                    f'front: {self.nearest_front:.2f} m',
                    (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        if pixels >= MIN_PIXELS:
            cv2.putText(overlay, 'DETECTED', (10, 135),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 3)

        self.last_overlay = overlay

        if self.state == 'SEARCHING' and pixels >= MIN_PIXELS:
            self.cube_detected = True

    # ── Stop helper ───────────────────────────────────────────────────────────
    def stop(self):
        self.pub.publish(Twist())

    # ── Wall-following command (right wall only) ───────────────────────────────
    def wall_follow_cmd(self) -> Twist:
        """
        PD controller with front-right lookahead and adaptive speed.
          P term  : current right-side error
          D term  : rate of change of error (damps oscillation)
          Lookahead: front-right distance anticipates upcoming corners
          Speed   : reduce when error is large (cornering)
        """
        cmd = Twist()

        # ── PD on current right-side error ────────────────────────────────────
        error   = self.nearest_right - WALL_DISTANCE
        d_error = (error - self.wall_prev_error) / WALL_DT
        self.wall_prev_error = error

        # ── Lookahead term: front-right quadrant ──────────────────────────────
        lookahead_error = self.nearest_front_right - WALL_DISTANCE

        # ── Combined angular output ───────────────────────────────────────────
        angular = -(WALL_KP * error
                    + WALL_KD * d_error
                    + WALL_KP * WALL_LOOKAHEAD * lookahead_error)
        cmd.angular.z = max(-TURN_SPEED, min(TURN_SPEED, angular))

        # ── Adaptive speed: slow down when far from wall or cornering ─────────
        speed_factor  = max(0.5, 1.0 - abs(error) * 1.5)
        cmd.linear.x  = FORWARD_SPEED * speed_factor

        return cmd

    # ── Approach command ───────────────────────────────────────────────────────
    def approach_cmd(self) -> Twist:
        cmd = Twist()
        if self.cube_cx is None:
            cmd.linear.x  = 0.0
            cmd.angular.z = TURN_SPEED * 0.4
            self.get_logger().warn('Approach: cube lost — spinning to re-acquire')
            return cmd
        offset = self.cube_cx - (self.img_width / 2)
        cmd.angular.z = max(-TURN_SPEED * 0.6,
                            min(TURN_SPEED * 0.6, -APPROACH_KP_ANG * offset))
        cmd.linear.x  = APPROACH_SPEED
        return cmd

    # ── Approach obstacle avoidance (gentle turn to keep cube in view) ────────
    def approach_avoid_cmd(self) -> Twist:
        APPROACH_AVOID_TURN = TURN_SPEED * 0.3
        cmd = Twist()
        cmd.linear.x = 0.0
        if self.nearest_left >= self.nearest_right:
            cmd.angular.z = APPROACH_AVOID_TURN
            self.get_logger().warn(
                f'Approach avoid: obstacle — gentle LEFT '
                f'(az={cmd.angular.z:.2f})')
        else:
            cmd.angular.z = -APPROACH_AVOID_TURN
            self.get_logger().warn(
                f'Approach avoid: obstacle — gentle RIGHT '
                f'(az={cmd.angular.z:.2f})')
        return cmd

    # ── Obstacle-aware return command ─────────────────────────────────────────
    def return_cmd(self) -> Twist:
        cmd = Twist()
        dx   = 0.0 - self.current_x
        dy   = 0.0 - self.current_y
        dist = math.sqrt(dx ** 2 + dy ** 2)

        # ── Phase A: far from origin — follow right wall home ─────────────────
        if dist > RETURN_WALL_THRESH:
            if self.nearest_front <= AVOID_DISTANCE:
                cmd.linear.x = 0.05
                if self.nearest_left >= self.nearest_right:
                    cmd.angular.z = RETURN_AVOID_TURN
                    self.get_logger().warn(
                        f'Return wall: obstacle — LEFT (az={RETURN_AVOID_TURN:.2f})')
                else:
                    cmd.angular.z = -RETURN_AVOID_TURN
                    self.get_logger().warn(
                        f'Return wall: obstacle — RIGHT (az={RETURN_AVOID_TURN:.2f})')
            else:
                # PD + lookahead (same as wall_follow_cmd)
                error          = self.nearest_right - WALL_DISTANCE
                d_error        = (error - self.wall_prev_error) / WALL_DT
                self.wall_prev_error = error
                lookahead_err  = self.nearest_front_right - WALL_DISTANCE
                angular = -(WALL_KP * error
                            + WALL_KD * d_error
                            + WALL_KP * WALL_LOOKAHEAD * lookahead_err)
                cmd.angular.z  = max(-TURN_SPEED, min(TURN_SPEED, angular))
                speed_factor   = max(0.5, 1.0 - abs(error) * 1.5)
                cmd.linear.x   = FORWARD_SPEED * speed_factor
                self.get_logger().info(
                    f'Return wall-follow PD | right={self.nearest_right:.2f} m '
                    f'err={error:.3f} derr={d_error:.3f} '
                    f'dist_to_origin={dist:.2f} m'
                )
            return cmd

        # ── Phase B: close to origin — P controller direct navigation ─────────
        target_yaw  = math.atan2(dy, dx)
        heading_err = target_yaw - self.current_yaw
        while heading_err >  math.pi: heading_err -= 2 * math.pi
        while heading_err < -math.pi: heading_err += 2 * math.pi

        if self.nearest_front <= AVOID_DISTANCE:
            cmd.linear.x = 0.05
            if self.nearest_left >= self.nearest_right:
                cmd.angular.z = RETURN_AVOID_TURN
                self.get_logger().warn(
                    f'Return P: obstacle — LEFT (az={RETURN_AVOID_TURN:.2f})')
            else:
                cmd.angular.z = -RETURN_AVOID_TURN
                self.get_logger().warn(
                    f'Return P: obstacle — RIGHT (az={RETURN_AVOID_TURN:.2f})')
        elif abs(heading_err) > math.radians(30):
            cmd.linear.x  = 0.0
            cmd.angular.z = max(-TURN_SPEED,
                                min(TURN_SPEED, RETURN_KP_ANG * heading_err))
        else:
            cmd.linear.x  = min(RETURN_MAX_SPD, RETURN_KP_LIN * dist)
            cmd.angular.z = max(-TURN_SPEED * 0.5,
                                min(TURN_SPEED * 0.5,
                                    RETURN_KP_ANG * heading_err))
        return cmd

    # ── Stuck detection ───────────────────────────────────────────────────────
    def check_stuck(self):
        dist_moved = math.sqrt(
            (self.current_x - self.last_moved_x) ** 2 +
            (self.current_y - self.last_moved_y) ** 2
        )
        now = self.get_clock().now()

        if dist_moved >= STUCK_DIST_THRESH:
            self.last_moved_x    = self.current_x
            self.last_moved_y    = self.current_y
            self.last_moved_time = now
        else:
            elapsed = (now - self.last_moved_time).nanoseconds / 1e9
            if elapsed >= STUCK_TIME:
                self.get_logger().warn(
                    f'STUCK detected ({elapsed:.1f}s) — escaping'
                )
                self.prev_state = self.state
                target = self.current_yaw + math.radians(ESCAPE_ANGLE)
                if target > math.pi:
                    target -= 2 * math.pi
                self.escape_target_yaw = target
                self.state = 'ESCAPING'
                self.last_moved_time = now
                self.last_moved_x    = self.current_x
                self.last_moved_y    = self.current_y

    # ── Circling detection (net displacement) ─────────────────────────────────
    def check_circling(self):
        now     = self.get_clock().now()
        elapsed = (now - self.circle_check_time).nanoseconds / 1e9

        if elapsed >= CIRCLE_CHECK_INTERVAL:
            dx = self.current_x - self.circle_check_pos[0]
            dy = self.current_y - self.circle_check_pos[1]
            net_disp = math.sqrt(dx ** 2 + dy ** 2)

            if net_disp < CIRCLE_DISP_THRESH:
                self.get_logger().warn(
                    f'CIRCLING detected (moved {net_disp:.2f} m in '
                    f'{elapsed:.1f}s) — escaping'
                )
                self.prev_state = self.state
                target = self.current_yaw + math.radians(ESCAPE_ANGLE)
                if target > math.pi:
                    target -= 2 * math.pi
                self.escape_target_yaw = target
                self.state = 'ESCAPING'

            self.circle_check_pos  = (self.current_x, self.current_y)
            self.circle_check_time = now

    # ── Control loop (10 Hz) ──────────────────────────────────────────────────
    def control_loop(self):
        if self.last_overlay is not None:
            cv2.imshow('Detection', self.last_overlay)
            cv2.waitKey(1)

        if self.state == 'DONE':
            return

        # ── SEARCHING ─────────────────────────────────────────────────────────
        if self.state == 'SEARCHING':
            self.check_stuck()
            self.check_circling()
            if self.state == 'ESCAPING':
                return

            if self.cube_detected:
                self.state = 'APPROACHING'
                self.get_logger().info('Cube detected — switching to APPROACHING')
                return

            if self.nearest_front <= AVOID_DISTANCE:
                cmd = Twist()
                cmd.linear.x = 0.0
                if self.nearest_left >= self.nearest_right:
                    cmd.angular.z = TURN_SPEED
                    self.get_logger().warn('Obstacle — turning LEFT')
                else:
                    cmd.angular.z = -TURN_SPEED
                    self.get_logger().warn('Obstacle — turning RIGHT')
            else:
                cmd = self.wall_follow_cmd()
                self.get_logger().info(
                    f'Wall-follow | right={self.nearest_right:.2f} m '
                    f'front={self.nearest_front:.2f} m '
                    f'pos=({self.current_x:.2f}, {self.current_y:.2f})'
                )
            self.pub.publish(cmd)

        # ── APPROACHING ───────────────────────────────────────────────────────
        elif self.state == 'APPROACHING':
            if (self.cube_pixels >= APPROACH_STOP_PIX and
                    self.nearest_front <= APPROACH_STOP_DIST):
                self.stop()
                self.state = 'DETECTED'
                self.get_logger().info(
                    f'Arrived at cube — front={self.nearest_front:.2f} m')
                self.get_logger().info(
                    f'Detected position: x={self.current_x:.3f} m  '
                    f'y={self.current_y:.3f} m  '
                    f'yaw={math.degrees(self.current_yaw):.1f}°'
                )
                if self.cube_cx is not None:
                    _focal_px  = self.img_width / (2.0 * math.tan(
                                     math.radians(CAMERA_HFOV_DEG / 2.0)))
                    _offset_px = self.cube_cx - self.img_width / 2.0
                    _bear_deg  = math.degrees(math.atan2(_offset_px, _focal_px))
                    self.get_logger().info(
                        f'Cube bearing: offset={_offset_px:.1f} px  '
                        f'bear={_bear_deg:.2f}deg'
                    )
                ts = self.get_clock().now().to_msg().sec
                if self.last_overlay is not None:
                    # ── Annotate detection image with position and angle ──────
                    save_img = self.last_overlay.copy()
                    yaw_deg  = math.degrees(self.current_yaw)

                    # Cube bearing
                    focal_px   = self.img_width / (2.0 * math.tan(
                                     math.radians(CAMERA_HFOV_DEG / 2.0)))
                    offset_px  = (self.cube_cx - self.img_width / 2.0) if self.cube_cx is not None else 0.0
                    bear_deg   = math.degrees(math.atan2(offset_px, focal_px))

                    # Single info line at top-left (below State label)
                    cv2.putText(save_img,
                                f'x={self.current_x:.3f} m  '
                                f'y={self.current_y:.3f} m  '
                                f'yaw={yaw_deg:.1f}deg  '
                                f'bear={bear_deg:.1f}deg  '
                                f'front={self.nearest_front:.2f} m  '
                                f'px={self.cube_pixels}',
                                (10, 145),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                                (0, 255, 255), 2)

                    # Compass arrow showing yaw direction (bottom-right)
                    ih, iw = save_img.shape[:2]
                    arrow_cx = iw - 55
                    arrow_cy = ih - 55
                    arrow_len = 35
                    ax = int(arrow_cx + arrow_len * math.cos(self.current_yaw))
                    ay = int(arrow_cy - arrow_len * math.sin(self.current_yaw))
                    cv2.circle(save_img, (arrow_cx, arrow_cy), 40,
                               (50, 50, 50), -1)
                    cv2.arrowedLine(save_img,
                                   (arrow_cx, arrow_cy), (ax, ay),
                                   (0, 200, 255), 2, tipLength=0.3)
                    cv2.putText(save_img, 'N',
                                (arrow_cx - 5, arrow_cy - 42),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                                (150, 150, 150), 1)

                    cv2.imwrite(f'detection_{ts}.jpg', save_img)
                    self.get_logger().info(
                        f'Image saved: detection_{ts}.jpg  '
                        f'yaw={yaw_deg:.1f}°')
                return

            if self.cube_pixels < MIN_PIXELS // 2:
                # Lost cube during approach — check if we were avoiding an obstacle
                if self.nearest_front <= AVOID_DISTANCE:
                    # Lost while avoiding — spin in place to re-acquire before giving up
                    self.cube_detected        = False
                    self.reacquire_start_time = self.get_clock().now()
                    self.reacquire_start_yaw  = self.current_yaw
                    self.reacquire_direction  = 1   # start by turning left
                    self.state                = 'REACQUIRING'
                    self.get_logger().warn(
                        'Cube lost during avoid — REACQUIRING')
                else:
                    # Lost without obstacle — go straight back to searching
                    self.cube_detected = False
                    self.state         = 'SEARCHING'
                    self.get_logger().warn(
                        'Cube lost during approach — back to SEARCHING')
                return

            if (self.nearest_front <= AVOID_DISTANCE and
                    self.cube_pixels < APPROACH_STOP_PIX):
                cmd = self.approach_avoid_cmd()
                self.pub.publish(cmd)
                return

            cmd = self.approach_cmd()
            self.get_logger().info(
                f'Approaching | front={self.nearest_front:.2f} m '
                f'pixels={self.cube_pixels} '
                f'cx={self.cube_cx} '
                f'pos=({self.current_x:.2f}, {self.current_y:.2f})'
            )
            self.pub.publish(cmd)

        # ── REACQUIRING: sweep ±90° to find cube again ───────────────────────
        elif self.state == 'REACQUIRING':
            elapsed = (self.get_clock().now() -
                       self.reacquire_start_time).nanoseconds / 1e9

            # Cube found — go back to approaching
            if self.cube_pixels >= MIN_PIXELS:
                self.cube_detected = True
                self.state = 'APPROACHING'
                self.get_logger().info(
                    f'Cube re-acquired ({self.cube_pixels} px) — APPROACHING')
                return

            # Timeout — give up and go back to full search
            if elapsed > self.reacquire_timeout:
                self.cube_detected = False
                self.state = 'SEARCHING'
                self.get_logger().warn(
                    f'Re-acquire timeout ({elapsed:.1f}s) — back to SEARCHING')
                return

            # Compute angle swept from start yaw
            swept = self.current_yaw - self.reacquire_start_yaw
            while swept >  math.pi: swept -= 2 * math.pi
            while swept < -math.pi: swept += 2 * math.pi

            # If reached ±90° limit, reverse direction
            if self.reacquire_direction == 1 and swept >= math.radians(90):
                self.reacquire_direction = -1
                self.get_logger().info('Reacquire: reached +90° — sweeping RIGHT')
            elif self.reacquire_direction == -1 and swept <= math.radians(-90):
                self.reacquire_direction = 1
                self.get_logger().info('Reacquire: reached -90° — sweeping LEFT')

            cmd = Twist()
            cmd.linear.x  = 0.0
            cmd.angular.z = TURN_SPEED * 0.35 * self.reacquire_direction
            self.pub.publish(cmd)
            self.get_logger().info(
                f'Reacquiring | swept={math.degrees(swept):.1f}° '
                f'dir={"LEFT" if self.reacquire_direction > 0 else "RIGHT"} '
                f'elapsed={elapsed:.1f}s'
            )
            self.get_logger().info(
                f'Reacquiring | elapsed={elapsed:.1f}/{self.reacquire_timeout}s '
                f'pixels={self.cube_pixels}')

        # ── DETECTED: record then start 180° turn ─────────────────────────────
        elif self.state == 'DETECTED':
            self.stop()
            target = self.current_yaw + math.pi
            if target > math.pi:
                target -= 2 * math.pi
            self.turn_target_yaw = target
            self.state = 'TURNING'
            self.get_logger().info(
                f'Starting 180° turn — '
                f'from yaw={math.degrees(self.current_yaw):.1f}° '
                f'to {math.degrees(target):.1f}°'
            )

        # ── TURNING: spin 180° in place ───────────────────────────────────────
        elif self.state == 'TURNING':
            err = self.turn_target_yaw - self.current_yaw
            while err >  math.pi: err -= 2 * math.pi
            while err < -math.pi: err += 2 * math.pi

            if abs(err) < math.radians(8):
                self.stop()
                self.state = 'RETURNING'
                self.get_logger().info('180° turn complete — switching to RETURNING')
            else:
                cmd = Twist()
                cmd.linear.x  = 0.0
                cmd.angular.z = TURN_SPEED if err > 0 else -TURN_SPEED
                self.pub.publish(cmd)
                self.get_logger().info(f'Turning | err={math.degrees(err):.1f}°')

        # ── RETURNING ─────────────────────────────────────────────────────────
        elif self.state == 'RETURNING':
            # Corner check — highest priority, all three directions blocked
            if (self.nearest_front <= CORNER_FRONT_DIST and
                    self.nearest_left  <= CORNER_SIDE_DIST  and
                    self.nearest_right <= CORNER_SIDE_DIST):
                self.backup_start_time  = self.get_clock().now()
                self.corner_turn_target = None
                self.state = 'CORNERED'
                self.get_logger().warn(
                    f'CORNERED — front={self.nearest_front:.2f} m '
                    f'left={self.nearest_left:.2f} m '
                    f'right={self.nearest_right:.2f} m'
                )
                return

            self.check_stuck()
            if self.state == 'ESCAPING':
                return

            dx   = 0.0 - self.current_x
            dy   = 0.0 - self.current_y
            dist = math.sqrt(dx ** 2 + dy ** 2)

            if dist < RETURN_THRESH:
                self.stop()
                self.state = 'DONE'
                elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
                self.get_logger().info(
                    f'Arrived at origin — final pos '
                    f'({self.current_x:.3f}, {self.current_y:.3f}), '
                    f'error={dist:.3f} m  total_time={elapsed:.1f}s'
                )
                return

            cmd = self.return_cmd()
            self.get_logger().info(
                f'Returning | dist={dist:.2f} m '
                f'front={self.nearest_front:.2f} m '
                f'pos=({self.current_x:.2f}, {self.current_y:.2f})'
            )
            self.pub.publish(cmd)

        # ── CORNERED: back up then turn toward open space ─────────────────────
        elif self.state == 'CORNERED':
            now     = self.get_clock().now()
            elapsed = (now - self.backup_start_time).nanoseconds / 1e9

            if elapsed < BACKUP_TIME:
                cmd = Twist()
                cmd.linear.x  = BACKUP_SPEED
                cmd.angular.z = 0.0
                self.pub.publish(cmd)
                self.get_logger().warn(
                    f'Cornered: backing up {elapsed:.1f}/{BACKUP_TIME}s')

            elif self.corner_turn_target is None:
                if self.nearest_left >= self.nearest_right:
                    turn = math.radians(180)
                    self.get_logger().warn('Cornered: turning LEFT (more space)')
                else:
                    turn = math.radians(-180)
                    self.get_logger().warn('Cornered: turning RIGHT (more space)')
                target = self.current_yaw + turn
                if target >  math.pi: target -= 2 * math.pi
                if target < -math.pi: target += 2 * math.pi
                self.corner_turn_target = target

            else:
                err = self.corner_turn_target - self.current_yaw
                while err >  math.pi: err -= 2 * math.pi
                while err < -math.pi: err += 2 * math.pi

                if abs(err) < math.radians(8):
                    self.stop()
                    self.corner_turn_target = None
                    self.state = 'RETURNING'
                    self.get_logger().info(
                        'Corner escape complete — resuming RETURNING')
                else:
                    cmd = Twist()
                    cmd.linear.x  = 0.0
                    cmd.angular.z = TURN_SPEED if err > 0 else -TURN_SPEED
                    self.pub.publish(cmd)
                    self.get_logger().info(
                        f'Cornered: turning | err={math.degrees(err):.1f}°')

        # ── ESCAPING: turn 135° then resume previous state ────────────────────
        elif self.state == 'ESCAPING':
            err = self.escape_target_yaw - self.current_yaw
            while err >  math.pi: err -= 2 * math.pi
            while err < -math.pi: err += 2 * math.pi

            if abs(err) < math.radians(8):
                self.stop()
                self.state = self.prev_state
                self.circle_check_pos  = (self.current_x, self.current_y)
                self.circle_check_time = self.get_clock().now()
                self.get_logger().info(
                    f'Escape complete — resuming {self.prev_state}'
                )
            else:
                cmd = Twist()
                cmd.linear.x  = 0.0
                cmd.angular.z = TURN_SPEED if err > 0 else -TURN_SPEED
                self.pub.publish(cmd)
                self.get_logger().info(f'Escaping | err={math.degrees(err):.1f}°')


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
