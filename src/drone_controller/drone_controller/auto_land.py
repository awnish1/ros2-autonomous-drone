import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


class AutoLand(Node):

    def __init__(self):
        super().__init__('auto_land')

        self.ground_z = 0.05
        self.altitude = None
        self.vertical_velocity = 0.0
        self.last_odom = None
        self.touchdown_since = None
        self.disarm_count = 0
        self.landed = False
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

        self.timer = self.create_timer(
            0.1, self.control_loop
        )

        self.get_logger().info(
            'Automatic landing controller ready'
        )

    def odom_callback(self, msg):
        self.altitude = msg.pose.pose.position.z
        self.vertical_velocity = msg.twist.twist.linear.z
        self.last_odom = time.monotonic()

    def control_loop(self):

        now = time.monotonic()

        if self.landed:
            if self.disarm_count < 15:
                self.enable_pub.publish(Bool(data=False))
                self.disarm_count += 1
            else:
                self.timer.cancel()
                self.get_logger().info(
                    'LANDING COMPLETE. Motors disarmed.'
                )
            return

        if (
            self.last_odom is None
            or now - self.last_odom > 1.0
        ):
            self.cmd_pub.publish(Twist())

            if now - self.last_log > 2.0:
                self.get_logger().warn(
                    'Waiting for fresh odometry'
                )
                self.last_log = now
            return

        self.enable_pub.publish(Bool(data=True))

        distance = self.altitude - self.ground_z

        # Slow down automatically near the ground
        speed = max(
            -0.25,
            min(0.0, -0.8 * distance)
        )

        command = Twist()
        command.linear.z = speed
        self.cmd_pub.publish(command)

        # Confirm the drone is close to the ground
        if (
            self.altitude <= 0.08
            and abs(self.vertical_velocity) < 0.10
        ):
            if self.touchdown_since is None:
                self.touchdown_since = now

            if now - self.touchdown_since >= 1.0:
                self.cmd_pub.publish(Twist())
                self.landed = True
                self.get_logger().info(
                    'Touchdown threshold reached!'
                )
                return
        else:
            self.touchdown_since = None

        if now - self.last_log >= 1.0:
            self.get_logger().info(
                f'LANDING | Altitude: '
                f'{self.altitude:.3f} m | '
                f'Descent command: {speed:.3f} m/s'
            )
            self.last_log = now


def main(args=None):

    rclpy.init(args=args)
    node = AutoLand()

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
