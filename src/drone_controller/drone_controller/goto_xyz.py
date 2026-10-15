import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


class GoToXYZ(Node):

    def __init__(self):
        super().__init__('goto_xyz')

        self.declare_parameter('target_x', 2.0)
        self.declare_parameter('target_y', 0.0)
        self.declare_parameter('target_z', 2.0)

        self.tx = float(self.get_parameter('target_x').value)
        self.ty = float(self.get_parameter('target_y').value)
        self.tz = float(self.get_parameter('target_z').value)

        if not (0.5 <= self.tz <= 10.0 and
                abs(self.tx) <= 30.0 and abs(self.ty) <= 30.0):
            raise ValueError('Target outside safe test range')

        self.state = 'TAKEOFF'

        self.pose = None
        self.yaw = 0.0
        self.vz = 0.0
        self.last_odom = None
        self.home_xy = None

        self.stable_since = None
        self.hover_since = None
        self.touchdown_since = None
        self.disarm_ticks = 0
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
            self.on_odom,
            qos_profile_sensor_data
        )

        self.timer = self.create_timer(0.1, self.update)

        self.get_logger().info(
            f'Navigation target: '
            f'({self.tx}, {self.ty}, {self.tz})'
        )

    def on_odom(self, msg):

        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        self.pose = (p.x, p.y, p.z)

        self.yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

        self.vz = msg.twist.twist.linear.z
        self.last_odom = time.monotonic()

        if self.home_xy is None:
            self.home_xy = (p.x, p.y)

    def publish_velocity(self, vx=0.0, vy=0.0, vz=0.0):

        # Convert world-frame velocity to drone body frame.
        c = math.cos(self.yaw)
        s = math.sin(self.yaw)

        cmd = Twist()

        cmd.linear.x = c * vx + s * vy
        cmd.linear.y = -s * vx + c * vy
        cmd.linear.z = float(vz)

        self.cmd_pub.publish(cmd)

    def position_command(self, tx, ty, tz):

        x, y, z = self.pose

        vx = 0.6 * (tx - x)
        vy = 0.6 * (ty - y)

        speed = math.hypot(vx, vy)

        if speed > 0.35:
            scale = 0.35 / speed
            vx *= scale
            vy *= scale

        vz = max(
            -0.35,
            min(0.35, 0.8 * (tz - z))
        )

        return vx, vy, vz

    def update(self):

        now = time.monotonic()

        if self.state == 'DONE':
            return

        if self.state == 'DISARM':

            self.publish_velocity()
            self.enable_pub.publish(Bool(data=False))
            self.disarm_ticks += 1

            if self.disarm_ticks >= 15:
                self.state = 'DONE'
                self.timer.cancel()
                self.get_logger().info(
                    'MISSION COMPLETE: Destination reached and landed'
                )
            return

        if self.last_odom is None or now - self.last_odom > 1.0:

            self.publish_velocity()

            if now - self.last_log >= 2.0:
                self.get_logger().warn(
                    'Waiting for fresh odometry'
                )
                self.last_log = now
            return

        self.enable_pub.publish(Bool(data=True))

        x, y, z = self.pose

        # PHASE 1: TAKEOFF
        if self.state == 'TAKEOFF':

            vx, vy, vz = self.position_command(
                *self.home_xy, self.tz
            )

            if abs(z - self.tz) < 0.07 and abs(self.vz) < 0.1:

                if self.stable_since is None:
                    self.stable_since = now

                elif now - self.stable_since >= 1.0:
                    self.state = 'NAVIGATE'
                    self.stable_since = None
                    self.get_logger().info(
                        'Reached flight altitude. Navigating.'
                    )
            else:
                self.stable_since = None

        # PHASE 2: NAVIGATE
        elif self.state == 'NAVIGATE':

            vx, vy, vz = self.position_command(
                self.tx, self.ty, self.tz
            )

            distance = math.hypot(
                self.tx - x,
                self.ty - y
            )

            if (distance < 0.12
                    and abs(self.tz - z) < 0.07
                    and abs(self.vz) < 0.1):

                if self.stable_since is None:
                    self.stable_since = now

                elif now - self.stable_since >= 1.0:
                    self.state = 'HOVER'
                    self.hover_since = now
                    self.get_logger().info(
                        'Destination reached. Hovering for 5 seconds.'
                    )
            else:
                self.stable_since = None

        # PHASE 3: HOVER
        elif self.state == 'HOVER':

            vx, vy, vz = self.position_command(
                self.tx, self.ty, self.tz
            )

            if now - self.hover_since >= 5.0:
                self.state = 'LAND'
                self.get_logger().info(
                    'Hover complete. Landing at destination.'
                )

        # PHASE 4: LAND
        else:

            vx, vy, _ = self.position_command(
                self.tx, self.ty, self.tz
            )

            distance = max(0.0, z - 0.05)

            vz = -min(
                0.25,
                max(0.025, 0.8 * distance)
            )

            if z <= 0.085 and abs(self.vz) < 0.1:

                if self.touchdown_since is None:
                    self.touchdown_since = now

                elif now - self.touchdown_since >= 1.0:
                    self.state = 'DISARM'
                    self.publish_velocity()
                    self.get_logger().info(
                        'Touchdown confirmed. Disarming.'
                    )
                    return
            else:
                self.touchdown_since = None

        self.publish_velocity(vx, vy, vz)

        if now - self.last_log >= 1.0:

            self.get_logger().info(
                f'{self.state} | '
                f'XYZ=({x:.2f}, {y:.2f}, {z:.2f}) m | '
                f'Target=({self.tx:.2f}, {self.ty:.2f}, {self.tz:.2f})'
            )

            self.last_log = now

    def stop(self):

        for _ in range(5):
            self.publish_velocity()
            time.sleep(0.05)


def main(args=None):

    rclpy.init(args=args)
    node = GoToXYZ()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        if rclpy.ok() and node.state not in ('DONE', 'DISARM'):
            node.stop()

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
