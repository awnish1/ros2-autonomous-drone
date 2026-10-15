import json
import math
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


class AStarFollower(Node):

    def __init__(self):
        super().__init__('follow_astar')

        route_file = (
            Path.home()
            / 'drone_ws/src/drone_description/worlds/astar_route.json'
        )

        route = json.loads(route_file.read_text())

        self.waypoints = [
            (float(w['x']), float(w['y']), float(w['z']))
            for w in route['waypoints']
        ]

        if len(self.waypoints) < 2:
            raise ValueError('Route must contain at least two waypoints')

        if not all(
            math.isfinite(v)
            for point in self.waypoints
            for v in point
        ):
            raise ValueError('Invalid route coordinates')

        self.index = 1
        self.state = 'TAKEOFF'

        self.position = None
        self.yaw = 0.0
        self.vz = 0.0

        self.scan = None
        self.odom_time = None
        self.scan_time = None

        self.start_checked = False
        self.stable_since = None
        self.hover_since = None
        self.touchdown_since = None
        self.disarm_ticks = 0
        self.last_log = 0.0

        self.max_speed = 0.25
        self.safe_distance = 1.6

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
            f'A* navigation ready: {len(self.waypoints)} waypoints'
        )

    def odom_callback(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        self.position = (p.x, p.y, p.z)

        self.vz = msg.twist.twist.linear.z

        self.yaw = math.atan2(
            2 * (q.w * q.z + q.x * q.y),
            1 - 2 * (q.y*q.y + q.z*q.z)
        )

        self.odom_time = time.monotonic()

    def scan_callback(self, msg):
        self.scan = msg
        self.scan_time = time.monotonic()

    def send_velocity(self, vx=0.0, vy=0.0, vz=0.0):
        c = math.cos(self.yaw)
        s = math.sin(self.yaw)

        cmd = Twist()

        cmd.linear.x = c*vx + s*vy
        cmd.linear.y = -s*vx + c*vy
        cmd.linear.z = float(vz)

        self.cmd_pub.publish(cmd)

    def position_control(self, target):
        x, y, z = self.position
        tx, ty, tz = target

        vx = 0.6 * (tx - x)
        vy = 0.6 * (ty - y)

        speed = math.hypot(vx, vy)

        if speed > self.max_speed:
            scale = self.max_speed / speed
            vx *= scale
            vy *= scale

        vz = max(
            -0.3,
            min(0.3, 0.8 * (tz - z))
        )

        return vx, vy, vz

    def obstacle_distance(self, vx, vy):
        if math.hypot(vx, vy) < 0.01:
            return float('inf')

        c = math.cos(self.yaw)
        s = math.sin(self.yaw)

        bx = c*vx + s*vy
        by = -s*vx + c*vy

        heading = math.atan2(by, bx)

        minimum = float('inf')
        valid = False

        for i, distance in enumerate(self.scan.ranges):

            angle = (
                self.scan.angle_min
                + i*self.scan.angle_increment
            )

            difference = math.atan2(
                math.sin(angle - heading),
                math.cos(angle - heading)
            )

            if abs(difference) > math.radians(40):
                continue

            if (
                math.isfinite(distance)
                and self.scan.range_min <= distance <= self.scan.range_max
            ):
                valid = True
                minimum = min(minimum, distance)

            elif math.isinf(distance) and distance > 0:
                valid = True

        if not valid:
            return None

        return minimum

    def control(self):
        now = time.monotonic()

        if self.state == 'DONE':
            return

        if self.state == 'DISARM':
            self.send_velocity()
            self.enable_pub.publish(Bool(data=False))

            self.disarm_ticks += 1

            if self.disarm_ticks >= 15:
                self.state = 'DONE'
                self.timer.cancel()
                self.get_logger().info(
                    'MISSION COMPLETE: A* navigation and landing successful'
                )
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
            self.send_velocity()

            if self.state == 'NAVIGATE':
                self.state = 'BLOCKED'

            if now - self.last_log > 2.0:
                self.get_logger().warn(
                    'Waiting for fresh odometry and LiDAR'
                )
                self.last_log = now
            return

        x, y, z = self.position

        if not self.start_checked:
            sx, sy, _ = self.waypoints[0]

            if math.hypot(x - sx, y - sy) > 0.5:
                self.state = 'ABORT'
                self.get_logger().error(
                    'Drone is not at route start. Restart Gazebo.'
                )

            self.start_checked = True

        if self.state == 'ABORT':
            self.send_velocity()
            return

        self.enable_pub.publish(Bool(data=True))

        if self.state == 'BLOCKED':
            vz = self.position_control(
                (x, y, self.waypoints[0][2])
            )[2]

            self.send_velocity(vz=vz)

            if now - self.last_log >= 2.0:
                self.get_logger().warn(
                    'BLOCKED: Holding position. Navigation stopped.'
                )
                self.last_log = now
            return

        if self.state == 'TAKEOFF':
            vx, vy, vz = self.position_control(self.waypoints[0])

            if (
                abs(z - self.waypoints[0][2]) < 0.07
                and abs(self.vz) < 0.1
            ):
                if self.stable_since is None:
                    self.stable_since = now

                elif now - self.stable_since >= 1.0:
                    self.state = 'NAVIGATE'
                    self.stable_since = None
                    self.get_logger().info(
                        'Takeoff complete. Following A* route.'
                    )
            else:
                self.stable_since = None

        elif self.state == 'NAVIGATE':
            target = self.waypoints[self.index]
            vx, vy, vz = self.position_control(target)

            distance = math.hypot(
                target[0] - x,
                target[1] - y
            )

            if distance < 0.15 and abs(z - target[2]) < 0.1:
                vx = vy = 0.0

                if self.stable_since is None:
                    self.stable_since = now

                elif now - self.stable_since >= 0.4:
                    self.get_logger().info(
                        f'Waypoint {self.index} reached!'
                    )

                    self.index += 1
                    self.stable_since = None

                    if self.index >= len(self.waypoints):
                        self.state = 'HOVER'
                        self.hover_since = now
                        self.get_logger().info(
                            'Destination reached. Hovering.'
                        )
            else:
                self.stable_since = None

            if self.state == 'NAVIGATE':
                clearance = self.obstacle_distance(vx, vy)

                if (
                    clearance is None
                    or clearance < self.safe_distance
                ):
                    self.state = 'BLOCKED'
                    vx = vy = 0.0

                    self.get_logger().warn(
                        f'SAFETY STOP! Clearance: {clearance}'
                    )

        elif self.state == 'HOVER':
            vx, vy, vz = self.position_control(
                self.waypoints[-1]
            )

            if now - self.hover_since >= 5.0:
                self.state = 'LAND'
                self.get_logger().info(
                    'Hover complete. Landing.'
                )

        else:  # LAND
            tx, ty, _ = self.waypoints[-1]
            vx, vy, _ = self.position_control((tx, ty, z))

            distance = max(0.0, z - 0.05)

            vz = -min(
                0.22,
                max(0.02, 0.8 * distance)
            )

            if (
                z <= 0.085
                and abs(self.vz) < 0.1
                and math.hypot(tx-x, ty-y) < 0.25
            ):
                if self.touchdown_since is None:
                    self.touchdown_since = now

                elif now - self.touchdown_since >= 1.0:
                    self.state = 'DISARM'
                    self.send_velocity()
                    self.get_logger().info(
                        'Touchdown confirmed. Disarming.'
                    )
                    return
            else:
                self.touchdown_since = None

        self.send_velocity(vx, vy, vz)

        if now - self.last_log >= 1.0:
            self.get_logger().info(
                f'{self.state} | '
                f'Waypoint {self.index}/{len(self.waypoints)-1} | '
                f'XYZ=({x:.2f}, {y:.2f}, {z:.2f})'
            )
            self.last_log = now


def main(args=None):
    rclpy.init(args=args)
    node = AStarFollower()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        if rclpy.ok():
            node.send_velocity()

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
