import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ObstacleDetector(Node):

    def __init__(self):
        super().__init__('obstacle_detector')

        self.threshold = 1.5
        self.latest_scan = None
        self.last_scan_time = None

        self.subscription = self.create_subscription(
            LaserScan,
            '/quadcopter/scan',
            self.scan_callback,
            qos_profile_sensor_data
        )

        self.timer = self.create_timer(
            1.0,
            self.check_obstacles
        )

        self.get_logger().info(
            '360-degree LiDAR obstacle detector started!'
        )

    def scan_callback(self, msg):
        self.latest_scan = msg
        self.last_scan_time = time.monotonic()

    def check_obstacles(self):

        if (
            self.latest_scan is None
            or self.last_scan_time is None
            or time.monotonic() - self.last_scan_time > 2.0
        ):
            self.get_logger().warn(
                'Waiting for fresh LiDAR data...'
            )
            return

        scan = self.latest_scan

        # Direction angles relative to the drone
        directions = {
            'Front': 0.0,
            'Left': math.pi / 2,
            'Right': -math.pi / 2,
            'Rear': math.pi
        }

        distances = {}
        half_width = math.radians(30)

        for name, center in directions.items():

            valid = []
            has_clear_return = False

            for i, distance in enumerate(scan.ranges):

                angle = (
                    scan.angle_min
                    + i * scan.angle_increment
                )

                difference = math.atan2(
                    math.sin(angle - center),
                    math.cos(angle - center)
                )

                if abs(difference) > half_width:
                    continue

                if (
                    math.isfinite(distance)
                    and scan.range_min <= distance <= scan.range_max
                ):
                    valid.append(distance)

                elif math.isinf(distance) and distance > 0:
                    has_clear_return = True

            if valid:
                distances[name] = min(valid)
            elif has_clear_return:
                distances[name] = None
            else:
                distances[name] = 'UNKNOWN'

        def format_distance(value):

            if value == 'UNKNOWN':
                return 'UNKNOWN'

            if value is None:
                return f'CLEAR (>{scan.range_max:.1f} m)'

            return f'{value:.2f} m'

        front = format_distance(distances['Front'])
        left = format_distance(distances['Left'])
        right = format_distance(distances['Right'])
        rear = format_distance(distances['Rear'])

        danger = any(
            isinstance(value, (int, float))
            and value < self.threshold
            for value in distances.values()
        )

        unknown = any(
            value == 'UNKNOWN'
            for value in distances.values()
        )

        if danger:
            status = 'WARNING: OBSTACLE TOO CLOSE!'
        elif unknown:
            status = 'UNKNOWN: CHECK SENSOR'
        else:
            status = 'SAFE'

        message = (
            f'Front: {front} | '
            f'Left: {left} | '
            f'Right: {right} | '
            f'Rear: {rear} | '
            f'Status: {status}'
        )

        if danger:
            self.get_logger().warn(message)
        else:
            self.get_logger().info(message)


def main(args=None):

    rclpy.init(args=args)

    node = ObstacleDetector()

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
