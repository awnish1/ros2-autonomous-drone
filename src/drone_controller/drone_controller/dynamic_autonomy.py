
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

from drone_controller.dynamic_navigation import AStarPlanner


class DynamicAutonomy(Node):

    def __init__(self):
        super().__init__('dynamic_autonomy')

        self.declare_parameter('dry_run', True)
        self.dry_run = self.get_parameter('dry_run').value

        self.declare_parameter('takeoff_only', False)
        self.takeoff_only = self.get_parameter('takeoff_only').value

        self.declare_parameter('waypoint_test', False)
        self.waypoint_test = self.get_parameter('waypoint_test').value
        self.declare_parameter('max_test_waypoints', 1)
        self.max_test_waypoints = self.get_parameter('max_test_waypoints').value

        self.declare_parameter('recovery_test', False)
        self.recovery_test = self.get_parameter('recovery_test').value

        if self.recovery_test and (self.takeoff_only or self.waypoint_test):
            raise ValueError('Recovery test must run alone')

        if not isinstance(self.max_test_waypoints, int) or self.max_test_waypoints < 1:
            raise ValueError('max_test_waypoints must be a positive integer')

        if self.takeoff_only and self.waypoint_test:
            raise ValueError(
                'takeoff_only and waypoint_test cannot both be true'
            )

        self.goal = (5.0, 5.0)
        self.target_z = 2.0
        self.state = 'TAKEOFF'

        self.position = None
        self.yaw = 0.0
        self.vz = 0.0
        self.odom_time = 0.0
        self.scan = None
        self.scan_time = 0.0

        data = json.loads(
            (Path.home() /
             'drone_ws/maps/lidar_obstacle_regions.json').read_text()
        )

        self.known = [
            (w['name'], w['x'], w['y'], w['width'], w['height'])
            for w in data['known_walls']
        ]

        self.unknown = set()
        self.route = []
        self.index = 1
        self.stable_since = None
        self.hover_since = None
        self.touchdown_since = None
        self.blocked_since = None
        self.recovery_start = None
        self.recovery_direction = None
        self.disarm_ticks = 0
        self.last_log = 0.0

        self.cmd_pub = self.create_publisher(
            Twist, '/quadcopter/cmd_vel', 10
        )

        self.enable_pub = self.create_publisher(
            Bool, '/quadcopter/enable', 10
        )

        self.create_subscription(
            Odometry, '/model/quadcopter/odometry',
            self.on_odom, qos_profile_sensor_data
        )

        self.create_subscription(
            LaserScan, '/quadcopter/scan',
            self.on_scan, qos_profile_sensor_data
        )

        self.timer = self.create_timer(0.1, self.update)

        self.get_logger().info(
            f'Dynamic autonomy started | dry_run={self.dry_run}'
        )

        if self.dry_run:
            self.get_logger().warn(
                'FLIGHT DISABLED: No motor commands will be published'
            )

    def on_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        self.position = (p.x, p.y, p.z)
        self.vz = msg.twist.twist.linear.z

        self.yaw = math.atan2(
            2*(q.w*q.z + q.x*q.y),
            1 - 2*(q.y*q.y + q.z*q.z)
        )

        self.odom_time = time.monotonic()

    def on_scan(self, msg):
        self.scan = msg
        self.scan_time = time.monotonic()

        if (
            self.position is None
            or time.monotonic() - self.odom_time > 0.5
        ):
            return

        x, y, z = self.position

        if not 1.5 <= z <= 2.5:
            return

        for i, d in enumerate(msg.ranges):
            if (
                not math.isfinite(d)
                or not msg.range_min <= d <= msg.range_max
            ):
                continue

            angle = self.yaw + msg.angle_min + i*msg.angle_increment
            hx = x + d*math.cos(angle)
            hy = y + d*math.sin(angle)

            near_known = any(
                math.hypot(
                    max(abs(hx-ox)-sx/2, 0),
                    max(abs(hy-oy)-sy/2, 0)
                ) <= 0.45
                for _, ox, oy, sx, sy in self.known
            )

            if not near_known:
                self.unknown.add(
                    (round(hx/0.25), round(hy/0.25))
                )

    def estimated_obstacles(self):
        remaining = set(self.unknown)
        boxes = []

        while remaining:
            seed = remaining.pop()
            group = {seed}
            stack = [seed]

            while stack:
                a, b = stack.pop()

                for dx in (-1, 0, 1):
                    for dy in (-1, 0, 1):
                        candidate = (a+dx, b+dy)

                        if candidate in remaining:
                            remaining.remove(candidate)
                            group.add(candidate)
                            stack.append(candidate)

            if len(group) < 3:
                continue

            xs = [p[0]*0.25 for p in group]
            ys = [p[1]*0.25 for p in group]

            boxes.append((
                'observed',
                (min(xs)+max(xs))/2,
                (min(ys)+max(ys))/2,
                max(xs)-min(xs)+0.7,
                max(ys)-min(ys)+0.7
            ))

        return self.known + boxes

    def clearance(self, vx, vy):
        if self.scan is None:
            return None

        direction = math.atan2(vy, vx) - self.yaw
        nearest = math.inf
        valid = False

        for i, d in enumerate(self.scan.ranges):
            angle = (
                self.scan.angle_min
                + i*self.scan.angle_increment
            )

            diff = math.atan2(
                math.sin(angle-direction),
                math.cos(angle-direction)
            )

            if abs(diff) > math.radians(35):
                continue

            if (
                math.isfinite(d)
                and self.scan.range_min <= d <= self.scan.range_max
            ):
                valid = True
                nearest = min(nearest, d)

            elif math.isinf(d) and d > 0:
                valid = True

        return nearest if valid else None

    @staticmethod
    def limit(value, bound):
        return max(-bound, min(bound, value))

    def hold_z(self):
        return self.limit(
            0.8*(self.target_z-self.position[2]), 0.3
        )

    def send_velocity(self, vx=0.0, vy=0.0, vz=0.0):
        if self.dry_run:
            return

        # Takeoff-only mode never permits horizontal commands.
        if self.takeoff_only:
            vx = 0.0
            vy = 0.0

        c = math.cos(self.yaw)
        s = math.sin(self.yaw)

        cmd = Twist()
        cmd.linear.x = c*vx + s*vy
        cmd.linear.y = -s*vx + c*vy
        cmd.linear.z = float(vz)

        self.cmd_pub.publish(cmd)

    def enable(self, active):
        if not self.dry_run:
            self.enable_pub.publish(Bool(data=active))

    def choose_recovery(self, boxes):
        planner = AStarPlanner(boxes)
        x, y, _ = self.position
        initial = planner.clearance((x, y))
        best = None

        for i in range(16):
            angle = 2*math.pi*i/16
            dx = math.cos(angle)
            dy = math.sin(angle)

            sensed = self.clearance(dx, dy)
            if sensed is None or sensed < 1.8:
                continue

            previous = initial
            valid = True

            for step in range(1, 11):
                point = (
                    x + 0.1*step*dx,
                    y + 0.1*step*dy
                )

                value = planner.clearance(point)

                if value < previous-0.02:
                    valid = False
                    break

                previous = value

            gain = previous-initial

            if valid and gain > 0.12:
                if best is None or gain > best[0]:
                    best = (gain, dx, dy)

        return (best[1], best[2]) if best else None

    def update(self):
        now = time.monotonic()

        if self.state == 'DONE':
            return

        odom_ok = (
            self.position is not None
            and now-self.odom_time < 1.0
        )

        scan_ok = (
            self.scan is not None
            and now-self.scan_time < 1.0
        )

        if not odom_ok or not scan_ok:
            self.send_velocity()

            # Wait normally until both sensors first connect.
            # After initial connection, sensor loss latches HOLD.
            if (
                self.odom_time > 0.0
                and self.scan_time > 0.0
                and self.state not in ('HOLD', 'DONE', 'DISARM')
            ):
                self.state = 'HOLD'
                self.get_logger().error(
                    'SENSOR LOSS: HOLD latched. '
                    'Restart controller only after checking sensors.'
                )

            if now-self.last_log >= 2.0:
                self.get_logger().warn(
                    'WAIT: Fresh odometry and LiDAR required'
                )
                self.last_log = now
            return

        x, y, z = self.position

        if not self.route:
            if math.hypot(x, y) > 0.5:
                self.state = 'HOLD'
                self.get_logger().error(
                    'Drone is not near (0,0). Reset Gazebo.'
                )
            else:
                try:
                    self.route = AStarPlanner(
                        self.known
                    ).plan((x, y), self.goal)

                    self.get_logger().info(
                        f'Initial A* route: {len(self.route)} points'
                    )

                except ValueError as error:
                    self.state = 'HOLD'
                    self.get_logger().error(str(error))

        if self.state == 'HOLD':
            # HOLD must never initiate takeoff.
            if z <= 0.15:
                self.send_velocity()
                self.enable(False)
            else:
                # Preserve altitude when entering HOLD.
                if not hasattr(self, '_hold_altitude'):
                    self._hold_altitude = z

                vz = self.limit(
                    0.8 * (self._hold_altitude - z),
                    0.20
                )

                self.enable(True)
                self.send_velocity(vz=vz)

            return

        if self.state in ('HOLD', 'BLOCKED'):
            self.enable(True)
            self.send_velocity(vz=self.hold_z())

            if (
                self.state == 'BLOCKED'
                and now-self.blocked_since > 1.0
            ):
                if self.recovery_direction is None:
                    self.recovery_direction = self.choose_recovery(
                        self.estimated_obstacles()
                    )

                if self.recovery_direction is None:
                    self.state = 'HOLD'
                    self.get_logger().error(
                        'No verified recovery direction. Holding.'
                    )
                else:
                    self.state = 'RECOVER'
                    self.recovery_start = (x, y)
                    self.get_logger().warn(
                        'Recovery proposed using live LiDAR'
                    )
            return

        if self.state == 'DISARM':
            self.send_velocity()
            self.enable(False)
            self.disarm_ticks += 1

            if self.disarm_ticks >= 15:
                self.state = 'DONE'
                self.get_logger().info(
                    'MISSION COMPLETE: Landed and disarmed'
                )
            return

        self.enable(True)

        vx = 0.0
        vy = 0.0
        vz = self.hold_z()

        if self.state == 'TAKEOFF':
            if (
                abs(z-self.target_z) < 0.07
                and abs(self.vz) < 0.08
            ):
                if self.stable_since is None:
                    self.stable_since = now

                elif now-self.stable_since >= 1.0:
                    self.stable_since = None

                    if self.takeoff_only:
                        self.state = 'HOLD'
                        self._hold_altitude = z
                        self.get_logger().info(
                            'TAKEOFF TEST PASSED: Holding altitude.'
                        )
                    else:
                        self.state = 'NAVIGATE'
                        self.get_logger().info(
                            'Takeoff complete. Starting A* navigation.'
                        )
            else:
                self.stable_since = None

        elif self.state == 'NAVIGATE':
            target = self.route[self.index]

            dx = target[0]-x
            dy = target[1]-y
            distance = math.hypot(dx, dy)

            if distance < 0.17:
                if self.stable_since is None:
                    self.stable_since = now

                elif now-self.stable_since >= 0.4:
                    self.index += 1
                    self.stable_since = None

                    if self.waypoint_test and (
                        self.index >= self.max_test_waypoints + 1
                        or self.index >= len(self.route)
                    ):
                        self.state = 'HOLD'
                        self._hold_altitude = z
                        self.get_logger().info(
                            'WAYPOINT TEST PASSED: Holding position.'
                        )

                    elif self.index >= len(self.route):
                        self.state = 'HOVER'
                        self.hover_since = now
                        self.get_logger().info(
                            'Destination reached. Hovering.'
                        )
                    else:
                        self.get_logger().info(
                            f'Waypoint {self.index-1} reached'
                        )
            else:
                self.stable_since = None

                vx = 0.6*dx
                vy = 0.6*dy

                speed = math.hypot(vx, vy)

                if speed > 0.22:
                    vx *= 0.22/speed
                    vy *= 0.22/speed

                gap = self.clearance(vx, vy)

                if gap is None or gap < 1.8:
                    vx = vy = 0.0

                    if self.waypoint_test:
                        self.state = 'HOLD'
                        self._hold_altitude = z
                        self.get_logger().error(
                            f'WAYPOINT TEST STOPPED: Clearance {gap}'
                        )
                    else:
                        self.state = 'BLOCKED'
                        self.blocked_since = now
                        self.recovery_direction = None
                        self.get_logger().warn(
                            f'SAFETY STOP! Clearance: {gap}'
                        )

        elif self.state == 'RECOVER':
            if self.recovery_test:
                if self.recovery_start is None:
                    self.state = 'HOLD'
                    self._hold_altitude = z
                    self.send_velocity()
                    self.get_logger().error(
                        'Recovery start missing. HOLD.'
                    )
                    return

                if not hasattr(self, '_recovery_test_since'):
                    self._recovery_test_since = now

                traveled = math.hypot(
                    x - self.recovery_start[0],
                    y - self.recovery_start[1]
                )

                if traveled >= 0.35:
                    self.state = 'HOLD'
                    self._hold_altitude = z
                    self.send_velocity(vz=self.hold_z())
                    self.get_logger().info(
                        f'RECOVERY TEST COMPLETE: '
                        f'Moved {traveled:.2f} m. HOLD.'
                    )
                    return

                if now - self._recovery_test_since > 15.0:
                    self.state = 'HOLD'
                    self._hold_altitude = z
                    self.send_velocity()
                    self.get_logger().error(
                        'Recovery test timeout. HOLD.'
                    )
                    return
            planner = AStarPlanner(
                self.estimated_obstacles()
            )

            current_clearance = planner.clearance((x, y))

            if current_clearance >= 2.30 and not self.recovery_test:
                try:
                    updated = planner.plan(
                        (x, y), self.goal
                    )
                except ValueError:
                    updated = None

                if updated:
                    self.route = updated
                    self.index = 1
                    self.state = 'NAVIGATE'
                    self.stable_since = None

                    self.get_logger().info(
                        f'NEW A* ROUTE: {len(updated)} waypoints'
                    )

            if self.state == 'RECOVER':
                dx, dy = self.recovery_direction

                gap = self.clearance(dx, dy)

                traveled = math.hypot(
                    x-self.recovery_start[0],
                    y-self.recovery_start[1]
                )

                if (
                    gap is None
                    or gap < 1.8
                    or traveled >= 2.0
                ):
                    self.state = 'HOLD'
                    self.get_logger().error(
                        'Recovery stopped: Safety limit reached.'
                    )
                else:
                    vx = 0.10*dx
                    vy = 0.10*dy

        elif self.state == 'HOVER':
            dx = self.goal[0]-x
            dy = self.goal[1]-y

            vx = self.limit(0.5*dx, 0.12)
            vy = self.limit(0.5*dy, 0.12)

            if now-self.hover_since >= 5.0:
                self.state = 'LAND'
                self.get_logger().info(
                    'Hover complete. Landing.'
                )

        elif self.state == 'LAND':
            dx = self.goal[0]-x
            dy = self.goal[1]-y

            vx = self.limit(0.5*dx, 0.12)
            vy = self.limit(0.5*dy, 0.12)

            vz = -min(
                0.22,
                max(0.02, 0.8*(z-0.05))
            )

            if (
                z <= 0.085
                and abs(self.vz) < 0.1
                and math.hypot(dx, dy) < 0.25
            ):
                if self.touchdown_since is None:
                    self.touchdown_since = now

                elif now-self.touchdown_since >= 1.0:
                    self.state = 'DISARM'
                    vx = vy = vz = 0.0
                    self.get_logger().info(
                        'Touchdown confirmed. Disarming.'
                    )
            else:
                self.touchdown_since = None

        self.send_velocity(vx, vy, vz)

        if now-self.last_log >= 2.0:
            self.get_logger().info(
                f'{self.state} | '
                f'XYZ=({x:.2f}, {y:.2f}, {z:.2f}) | '
                f'Waypoint={self.index}/'
                f'{max(len(self.route)-1, 0)} | '
                f'Unknown cells={len(self.unknown)}'
            )
            self.last_log = now


def main(args=None):
    rclpy.init(args=args)
    node = DynamicAutonomy()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        if rclpy.ok() and not node.dry_run:
            node.send_velocity()

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
