import time
import rclpy

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import ExternalShutdownException

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


class AutoTakeoff(Node):

    def __init__(self):
        super().__init__('auto_takeoff')

        self.target_altitude = 2.0
        self.kp = 0.8
        self.max_speed = 0.35

        self.altitude = None
        self.last_odom = None
        self.last_log = 0.0

        self.publisher = self.create_publisher(
            Twist,
            '/quadcopter/cmd_vel',
            10
        )

        self.enabler = self.create_publisher(
            Bool,
            '/quadcopter/enable',
            10
        )

        self.subscription = self.create_subscription(
            Odometry,
            '/model/quadcopter/odometry',
            self.odometry_callback,
            qos_profile_sensor_data
        )

        self.timer = self.create_timer(
            0.1,
            self.control_loop
        )

        self.get_logger().info(
            'Waiting for odometry. Target altitude: 2.0 m'
        )

    def odometry_callback(self, msg):
        self.altitude = msg.pose.pose.position.z
        self.last_odom = time.monotonic()

    def control_loop(self):

        now = time.monotonic()

        if self.last_odom is None or now - self.last_odom > 1.5:
            self.publisher.publish(Twist())

            if now - self.last_log >= 2.0:
                self.get_logger().warn(
                    'Waiting for fresh odometry'
                )
                self.last_log = now
            return

        self.enabler.publish(Bool(data=True))

        error = self.target_altitude - self.altitude

        z_speed = self.kp * error

        z_speed = max(
            -self.max_speed,
            min(self.max_speed, z_speed)
        )

        command = Twist()
        command.linear.z = z_speed

        self.publisher.publish(command)

        if now - self.last_log >= 1.0:

            state = (
                'HOVER'
                if abs(error) < 0.05
                else 'CORRECTING'
            )

            self.get_logger().info(
                f'{state} | '
                f'Altitude: {self.altitude:.2f} m | '
                f'Target: 2.00 m | '
                f'Velocity command: {z_speed:+.2f} m/s'
            )

            self.last_log = now

    def stop(self):
        for _ in range(5):
            self.publisher.publish(Twist())
            time.sleep(0.05)


def main(args=None):

    rclpy.init(args=args)
    node = AutoTakeoff()

    try:
        rclpy.spin(node)

    except (KeyboardInterrupt, ExternalShutdownException):
        pass

    finally:
        if rclpy.ok():
            node.stop()

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
