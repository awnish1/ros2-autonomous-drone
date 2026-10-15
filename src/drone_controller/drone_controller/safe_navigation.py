import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


class SafeNavigation(Node):

    def __init__(self):
        super().__init__('safe_navigation')

        self.declare_parameter('target_x', 5.0)
        self.declare_parameter('target_y', 5.0)
        self.declare_parameter('target_z', 2.0)

        self.tx = float(self.get_parameter('target_x').value)
        self.ty = float(self.get_parameter('target_y').value)
        self.tz = float(self.get_parameter('target_z').value)

        if not all(math.isfinite(v) for v in
                   (self.tx, self.ty, self.tz)):
            raise ValueError('Target coordinates must be finite')

        if (abs(self.tx) > 30 or abs(self.ty) > 30
                or not 0.5 <= self.tz <= 10):
            raise ValueError('Target outside test limits')

        self.safe_distance = 1.8
        self.state = 'TAKEOFF'

        self.position = None
        self.yaw = 0.0
        self.vz = 0.0

        self.scan = None
        self.odom_time = None
        self.scan_time = None
        self.stable_since = None
        self.last_log = 0.0

        self.cmd_pub = self.create_publisher(
            Twist, '/quadcopter/cmd_vel', 10
        )

        self.enable_pub = self.create_publisher(
            Bool, '/quadcopter/enable', 10
        )

        self.create_subscription(
            Odometry,
            '/model/quadcopter/odometry',
            self.odom_callback,
            qos_profile_sensor_data
        )

        self.create_subscription(
            LaserScan,
            '/quadcopter/scan',
            self.scan_callback,
            qos_profile_sensor_data
        )

        self.timer = self.create_timer(0.1, self.control)

        self.get_logger().info(
            f'Safe navigation started. '
            f'Target=({self.tx}, {self.ty}, {self.tz})'
        )

    def odom_callback(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        self.position = (p.x, p.y, p.z)
        self.vz = msg.twist.twist.linear.z

        self.yaw = math.atan2(
            2 * (q.w * q.z + q.x * q.y),
            1 - 2 * (q.y * q.y + q.z * q.z)
        )

        self.odom_time = time.monotonic()

    def scan_callback(self, msg):
        self.scan = msg
        self.scan_time = time.monotonic()

    def clearance(self, direction):

        minimum = float('inf')
        valid = False

        for i, distance in enumerate(self.scan.ranges):

            angle = (
                self.scan.angle_min
                + i * self.scan.angle_increment
            )

            diff = math.atan2(
                math.sin(angle - direction),
                math.cos(angle - direction)
            )

            if abs(diff) > math.radians(30):
                continue

            if math.isinf(distance) and distance > 0:
                valid = True

            elif (
                math.isfinite(distance)
                and self.scan.range_min <= distance
                <= self.scan.range_max
            ):
                valid = True
                minimum = min(minimum, distance)

        return minimum if valid else None

    def control(self):

        now = time.monotonic()

        if (self.cmd_pub.get_subscription_count() == 0 or
                self.enable_pub.get_subscription_count() == 0):
            if now - self.last_log > 2:
                self.get_logger().warn(
                    'Waiting for flight-command bridge'
                )
                self.last_log = now
            return

        odom_ok = (
            self.odom_time is not None
            and now - self.odom_time < 1.5
        )

        scan_ok = (
            self.scan_time is not None
            and now - self.scan_time < 1.5
        )

        if not odom_ok or not scan_ok:

            self.cmd_pub.publish(Twist())

            if odom_ok:
                self.enable_pub.publish(Bool(data=True))

            if self.state == 'NAVIGATE':
                self.state = 'BLOCKED'
                self.get_logger().warn(
                    'SAFETY STOP: Sensor data unavailable'
                )

            if now - self.last_log > 2:
                self.get_logger().warn(
                    'Waiting for fresh odometry and LiDAR'
                )
                self.last_log = now

            return

        self.enable_pub.publish(Bool(data=True))

        x, y, z = self.position

        vx = 0.0
        vy = 0.0

        vz = max(
            -0.3,
            min(0.3, 0.8 * (self.tz - z))
        )

        if self.state == 'TAKEOFF':

            if (
                abs(z - self.tz) < 0.07
                and abs(self.vz) < 0.1
            ):
                if self.stable_since is None:
                    self.stable_since = now

                elif now - self.stable_since > 1.0:
                    self.state = 'NAVIGATE'
                    self.get_logger().info(
                        'Takeoff complete. Navigating.'
                    )
            else:
                self.stable_since = None

        elif self.state == 'NAVIGATE':

            dx = self.tx - x
            dy = self.ty - y

            distance = math.hypot(dx, dy)

            if distance < 0.12:

                self.state = 'ARRIVED'
                self.get_logger().info(
                    'Destination reached. Hovering.'
                )

            else:

                vx = 0.6 * dx
                vy = 0.6 * dy

                speed = math.hypot(vx, vy)

                if speed > 0.25:
                    vx *= 0.25 / speed
                    vy *= 0.25 / speed

                # World velocity -> body velocity
                c = math.cos(self.yaw)
                s = math.sin(self.yaw)

                bx = c * vx + s * vy
                by = -s * vx + c * vy

                direction = math.atan2(by, bx)

                obstacle_distance = self.clearance(direction)

                if (
                    obstacle_distance is None
                    or obstacle_distance < self.safe_distance
                ):

                    self.state = 'BLOCKED'
                    vx = 0.0
                    vy = 0.0

                    self.get_logger().warn(
                        'SAFETY STOP! '
                        f'Obstacle distance: {obstacle_distance}'
                    )

        # BLOCKED and ARRIVED always command zero XY velocity

        cmd = Twist()

        c = math.cos(self.yaw)
        s = math.sin(self.yaw)

        cmd.linear.x = c * vx + s * vy
        cmd.linear.y = -s * vx + c * vy
        cmd.linear.z = vz

        self.cmd_pub.publish(cmd)

        if now - self.last_log > 1.0:

            self.get_logger().info(
                f'{self.state} | '
                f'XYZ=({x:.2f}, {y:.2f}, {z:.2f})'
            )

            self.last_log = now


def main(args=None):
    rclpy.init(args=args)
    node = SafeNavigation()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.cmd_pub.publish(Twist())

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
