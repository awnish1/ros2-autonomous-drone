import json
import math
import time
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


class MovingMapper(Node):

    def __init__(self):
        super().__init__('moving_mapper')

        self.resolution = 0.25
        self.pose = None
        self.pose_time = 0.0

        self.occupied = set()
        self.free = set()
        self.scans = 0

        folder = Path.home() / 'drone_ws/maps'
        folder.mkdir(parents=True, exist_ok=True)

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')

        self.output = folder / f'moving_map_{timestamp}.json'

        self.create_subscription(
            Odometry,
            '/model/quadcopter/odometry',
            self.on_odometry,
            qos_profile_sensor_data
        )

        self.create_subscription(
            LaserScan,
            '/quadcopter/scan',
            self.on_scan,
            qos_profile_sensor_data
        )

        self.timer = self.create_timer(
            5.0,
            self.report
        )

        self.get_logger().info(
            'Continuous LiDAR mapping started!'
        )

    def on_odometry(self, msg):

        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        yaw = math.atan2(
            2 * (q.w*q.z + q.x*q.y),
            1 - 2 * (q.y*q.y + q.z*q.z)
        )

        self.pose = (p.x, p.y, p.z, yaw)
        self.pose_time = time.monotonic()

    def on_scan(self, msg):

        if self.pose is None:
            return

        if time.monotonic() - self.pose_time > 0.5:
            return

        x, y, z, yaw = self.pose

        if not 1.5 <= z <= 2.5:
            return

        resolution = self.resolution

        for i, reading in enumerate(msg.ranges):

            if (
                math.isfinite(reading)
                and msg.range_min <= reading <= msg.range_max
            ):
                distance = reading
                obstacle = True

            elif math.isinf(reading) and reading > 0:
                distance = msg.range_max
                obstacle = False

            else:
                continue

            angle = yaw + msg.angle_min + i * msg.angle_increment

            c = math.cos(angle)
            s = math.sin(angle)

            steps = max(
                0,
                int((distance - resolution / 2) / resolution)
            )

            for j in range(1, steps + 1):

                d = j * resolution

                cell = (
                    round((x + d*c) / resolution),
                    round((y + d*s) / resolution)
                )

                self.free.add(cell)

            if obstacle:

                hit = (
                    round((x + distance*c) / resolution),
                    round((y + distance*s) / resolution)
                )

                self.occupied.add(hit)

        self.scans += 1

    def save(self):

        if self.scans == 0:
            return

        observed_free = self.free - self.occupied

        result = {
            'type': 'partial_lidar_map',
            'resolution_m': self.resolution,
            'scans': self.scans,
            'occupied_cells_xy': [
                list(p) for p in sorted(self.occupied)
            ],
            'free_cells_xy': [
                list(p) for p in sorted(observed_free)
            ]
        }

        temporary = self.output.with_suffix('.json.tmp')

        temporary.write_text(
            json.dumps(result, indent=2) + '\n'
        )

        temporary.replace(self.output)

    def report(self):

        self.save()

        self.get_logger().info(
            f'Scans: {self.scans} | '
            f'Occupied: {len(self.occupied)} | '
            f'Observed-free: {len(self.free - self.occupied)}'
        )


def main():

    rclpy.init()
    node = MovingMapper()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.save()

        print(f'\nMap saved: {node.output}')

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
