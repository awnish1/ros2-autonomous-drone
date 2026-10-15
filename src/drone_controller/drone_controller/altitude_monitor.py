import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry


class AltitudeMonitor(Node):

    def __init__(self):
        super().__init__('altitude_monitor')

        self.position = None

        self.subscription = self.create_subscription(
            Odometry,
            '/model/quadcopter/odometry',
            self.odometry_callback,
            qos_profile_sensor_data
        )

        self.timer = self.create_timer(
            1.0,
            self.display_position
        )

        self.get_logger().info(
            'Drone altitude monitor started!'
        )

    def odometry_callback(self, msg):
        self.position = msg.pose.pose.position

    def display_position(self):

        if self.position is None:
            self.get_logger().info(
                'Waiting for drone odometry...'
            )
            return

        self.get_logger().info(
            f'X: {self.position.x:.2f} | '
            f'Y: {self.position.y:.2f} | '
            f'Altitude: {self.position.z:.2f} m'
        )


def main(args=None):

    rclpy.init(args=args)

    node = AltitudeMonitor()

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
