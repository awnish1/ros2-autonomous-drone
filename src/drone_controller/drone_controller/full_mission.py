import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


class FullMission(Node):

    def __init__(self):
        super().__init__('full_mission')

        self.target = 2.0
        self.phase = 'TAKEOFF'

        self.z = None
        self.vz = 0.0
        self.last_odom = None

        self.stable_since = None
        self.hover_since = None
        self.touchdown_since = None
        self.disarm_ticks = 0
        self.last_log = 0.0

        self.velocity_pub = self.create_publisher(
            Twist, '/quadcopter/cmd_vel', 10
        )

        self.enable_pub = self.create_publisher(
            Bool, '/quadcopter/enable', 10
        )

        self.create_subscription(
            Odometry,
            '/model/quadcopter/odometry',
            self.on_odometry,
            qos_profile_sensor_data
        )

        self.timer = self.create_timer(
            0.1, self.update
        )

        self.get_logger().info(
            'Mission ready: takeoff -> hover -> land'
        )

    def on_odometry(self, msg):
        self.z = msg.pose.pose.position.z
        self.vz = msg.twist.twist.linear.z
        self.last_odom = time.monotonic()

    def send_velocity(self, z_speed):
        command = Twist()
        command.linear.z = float(z_speed)
        self.velocity_pub.publish(command)

    def log_status(self, now, command):
        if now - self.last_log >= 1.0:
            self.get_logger().info(
                f'{self.phase} | '
                f'Altitude: {self.z:.3f} m | '
                f'Vz: {self.vz:+.3f} m/s | '
                f'Command: {command:+.3f} m/s'
            )
            self.last_log = now

    def update(self):
        now = time.monotonic()

        if self.phase == 'DONE':
            return

        # Never fly without fresh position feedback
        if self.last_odom is None or now - self.last_odom > 1.0:
            self.send_velocity(0.0)

            if now - self.last_log >= 2.0:
                self.get_logger().warn(
                    'Waiting for fresh odometry'
                )
                self.last_log = now
            return

        if self.phase == 'DISARM':
            self.send_velocity(0.0)
            self.enable_pub.publish(Bool(data=False))
            self.disarm_ticks += 1

            if self.disarm_ticks >= 15:
                self.phase = 'DONE'
                self.get_logger().info(
                    'MISSION COMPLETE: Landed and disarmed'
                )
                self.timer.cancel()
            return

        self.enable_pub.publish(Bool(data=True))

        if self.phase == 'TAKEOFF':

            error = self.target - self.z

            speed = max(
                -0.35,
                min(0.35, 0.8 * error)
            )

            if abs(error) <= 0.05 and abs(self.vz) < 0.08:

                if self.stable_since is None:
                    self.stable_since = now

                elif now - self.stable_since >= 1.5:
                    self.phase = 'HOVER'
                    self.hover_since = now
                    self.get_logger().info(
                        'Reached 2 metres. Hovering.'
                    )
            else:
                self.stable_since = None

        elif self.phase == 'HOVER':

            speed = max(
                -0.35,
                min(0.35, 0.8 * (self.target - self.z))
            )

            if now - self.hover_since >= 5.0:
                self.phase = 'LAND'
                self.get_logger().info(
                    'Hover completed. Starting landing.'
                )

        else:  # LAND

            distance = max(0.0, self.z - 0.05)

            speed = -min(
                0.25,
                max(0.025, 0.8 * distance)
            )

            if self.z <= 0.085 and abs(self.vz) < 0.10:

                if self.touchdown_since is None:
                    self.touchdown_since = now

                elif now - self.touchdown_since >= 1.0:
                    self.phase = 'DISARM'
                    self.send_velocity(0.0)
                    self.get_logger().info(
                        'Touchdown confirmed. Disarming.'
                    )
                    return
            else:
                self.touchdown_since = None

        self.send_velocity(speed)
        self.log_status(now, speed)

    def stop(self):
        for _ in range(5):
            self.send_velocity(0.0)
            time.sleep(0.05)


def main(args=None):
    rclpy.init(args=args)
    node = FullMission()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        if rclpy.ok():
            node.stop()

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
