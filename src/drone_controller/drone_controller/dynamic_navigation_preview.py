
import json
import math
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


class DynamicNavigationPreview(Node):

    def __init__(self):
        super().__init__('dynamic_navigation_preview')

        route_file = (
            Path.home()
            / 'drone_ws/src/drone_description/worlds/astar_route.json'
        )

        route = json.loads(route_file.read_text())

        self.waypoints = route['waypoints']
        self.position = None
        self.yaw = 0.0
        self.scan = None
        self.odom_time = None
        self.scan_time = None
        self.safe_distance = 1.8

        self.create_subscription(
            Odometry,
            '/model/quadcopter/odometry',
            self.on_odom,
            qos_profile_sensor_data
        )

        self.create_subscription(
            LaserScan,
            '/quadcopter/scan',
            self.on_scan,
            qos_profile_sensor_data
        )

        self.create_timer(1.0, self.check)

        self.get_logger().info(
            'Dynamic navigation preview started'
        )
        self.get_logger().info(
            'FLIGHT DISABLED: No motor commands will be sent'
        )

    def on_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        self.position = (p.x, p.y, p.z)

        self.yaw = math.atan2(
            2 * (q.w*q.z + q.x*q.y),
            1 - 2 * (q.y*q.y + q.z*q.z)
        )

        self.odom_time = time.monotonic()

    def on_scan(self, msg):
        self.scan = msg
        self.scan_time = time.monotonic()

    def check(self):
        now = time.monotonic()

        if (
            self.position is None
            or self.scan is None
            or self.odom_time is None
            or self.scan_time is None
            or now - self.odom_time > 1.5
            or now - self.scan_time > 1.5
        ):
            self.get_logger().warn(
                'WAIT: Fresh odometry and LiDAR required'
            )
            return

        x, y, z = self.position

        # Preview direction toward first A* waypoint.
        target = self.waypoints[1]

        dx = target['x'] - x
        dy = target['y'] - y

        heading = math.atan2(dy, dx) - self.yaw

        minimum = math.inf
        valid = False

        for i, distance in enumerate(self.scan.ranges):

            angle = (
                self.scan.angle_min
                + i * self.scan.angle_increment
            )

            difference = math.atan2(
                math.sin(angle - heading),
                math.cos(angle - heading)
            )

            if abs(difference) > math.radians(40):
                continue

            if (
                math.isfinite(distance)
                and self.scan.range_min <= distance
                <= self.scan.range_max
            ):
                minimum = min(minimum, distance)
                valid = True

            elif math.isinf(distance) and distance > 0:
                valid = True

        if not valid:
            status = 'UNKNOWN - STOP REQUIRED'
        elif minimum < self.safe_distance:
            status = 'BLOCKED - STOP REQUIRED'
        else:
            status = 'CLEAR'

        distance_text = (
            f'{minimum:.2f} m'
            if math.isfinite(minimum)
            else 'NO FINITE RETURN'
        )

        self.get_logger().info(
            f'PREVIEW | '
            f'XYZ=({x:.2f}, {y:.2f}, {z:.2f}) | '
            f'Clearance={distance_text} | '
            f'Status={status}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = DynamicNavigationPreview()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
