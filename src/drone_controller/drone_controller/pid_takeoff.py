import time
import rclpy

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


class PIDTakeoff(Node):

    def __init__(self):
        super().__init__('pid_takeoff')

        self.target = 2.0

        # PID gains
        self.kp = 0.8
        self.ki = 0.04
        self.kd = 0.15

        self.max_speed = 0.35

        self.altitude = None
        self.vertical_velocity = 0.0
        self.integral = 0.0

        self.last_odom = None
        self.last_update = time.monotonic()
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
            'PID controller ready. Target: 2.0 metres'
        )

    def odom_callback(self, msg):

        self.altitude = msg.pose.pose.position.z

        self.vertical_velocity = (
            msg.twist.twist.linear.z
        )

        self.last_odom = time.monotonic()

    def control_loop(self):

        now = time.monotonic()

        dt = min(now - self.last_update, 0.2)
        self.last_update = now

        if (
            self.last_odom is None
            or now - self.last_odom > 1.0
        ):
            self.cmd_pub.publish(Twist())
            self.integral = 0.0

            if now - self.last_log >= 2.0:
                self.get_logger().warn(
                    'Waiting for fresh odometry'
                )
                self.last_log = now

            return

        self.enable_pub.publish(Bool(data=True))

        error = self.target - self.altitude

        # Integral candidate with anti-windup limit
        candidate = max(
            -0.5,
            min(0.5, self.integral + error * dt)
        )

        # Derivative of altitude error is approximately -vz
        derivative = -self.vertical_velocity

        raw = (
            self.kp * error
            + self.ki * candidate
            + self.kd * derivative
        )

        # Avoid accumulating more error while saturated
        if not (
            abs(raw) > self.max_speed
            and raw * error > 0
        ):
            self.integral = candidate

        speed = (
            self.kp * error
            + self.ki * self.integral
            + self.kd * derivative
        )

        speed = max(
            -self.max_speed,
            min(self.max_speed, speed)
        )

        command = Twist()
        command.linear.z = speed

        self.cmd_pub.publish(command)

        if now - self.last_log >= 1.0:

            self.get_logger().info(
                f'Altitude: {self.altitude:.3f} m | '
                f'Error: {error:+.3f} m | '
                f'Command: {speed:+.3f} m/s'
            )

            self.last_log = now

    def stop(self):

        for _ in range(5):
            self.cmd_pub.publish(Twist())
            time.sleep(0.05)


def main(args=None):

    rclpy.init(args=args)
    node = PIDTakeoff()

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
