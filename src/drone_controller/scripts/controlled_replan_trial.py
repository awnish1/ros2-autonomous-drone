
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

from drone_controller.dynamic_autonomy import DynamicAutonomy


class ControlledReplanTrial(DynamicAutonomy):

    def __init__(self):
        super().__init__()

        self.declare_parameter("allow_sim_flight", False)
        self.allow_sim_flight = self.get_parameter(
            "allow_sim_flight"
        ).value

        if not self.dry_run and not self.allow_sim_flight:
            raise RuntimeError("Simulation flight not authorized")

        if self.takeoff_only or self.waypoint_test or self.recovery_test:
            raise RuntimeError("Other test modes must be disabled")

        self.test_replans = 0
        self.test_started = time.monotonic()

        self.get_logger().info(
            "CONTROLLED REPLAN TEST | "
            "Landing disabled | "
            "Maximum one replan | "
            "210-second timeout"
        )

    def latch_hold(self, reason):
        if self.state != "HOLD":
            self.state = "HOLD"

            if self.position is not None:
                self._hold_altitude = self.position[2]

            self.send_velocity()
            self.get_logger().warn(reason)

    def update(self):
        if self.state == "DONE":
            return

        if (
            hasattr(self, "test_started")
            and time.monotonic() - self.test_started > 210
            and self.state != "HOLD"
        ):
            self.latch_hold("TEST TIMEOUT: HOLD")
            return

        previous = self.state

        super().update()

        if (
            previous == "RECOVER"
            and self.state == "NAVIGATE"
        ):
            self.test_replans += 1
            self.get_logger().info(
                "REPLANNING SUCCESSFUL: "
                f"{len(self.route)} route points"
            )

        if self.state == "BLOCKED" and self.test_replans >= 1:
            self.latch_hold(
                "SECOND OBSTACLE: HOLD. "
                "Further recovery disabled in this test."
            )

        if self.state == "HOVER":
            self.latch_hold(
                "DESTINATION REACHED: HOLD | "
                f"Replans={self.test_replans}"
            )

        if self.state in ("LAND", "DISARM"):
            self.latch_hold("LANDING DISABLED: HOLD")


def main():
    rclpy.init()

    preflight = Node("replan_flight_preflight")

    readings = {
        "position": None,
        "odom_time": 0.0,
        "scan_time": 0.0,
    }

    def on_odom(msg):
        p = msg.pose.pose.position
        readings["position"] = (p.x, p.y, p.z)
        readings["odom_time"] = time.monotonic()

    def on_scan(msg):
        if msg.ranges:
            readings["scan_time"] = time.monotonic()

    subscriptions = [
        preflight.create_subscription(
            Odometry,
            "/model/quadcopter/odometry",
            on_odom,
            qos_profile_sensor_data
        ),
        preflight.create_subscription(
            LaserScan,
            "/quadcopter/scan",
            on_scan,
            qos_profile_sensor_data
        )
    ]

    trial = None

    try:
        deadline = time.monotonic() + 10.0

        while time.monotonic() < deadline:
            rclpy.spin_once(preflight, timeout_sec=0.2)

            now = time.monotonic()

            if (
                readings["position"] is not None
                and now - readings["odom_time"] < 1.0
                and now - readings["scan_time"] < 1.0
            ):
                break

        now = time.monotonic()
        p = readings["position"]

        sensors_ok = (
            p is not None
            and now - readings["odom_time"] < 1.0
            and now - readings["scan_time"] < 1.0
        )

        grounded = (
            p is not None
            and math.hypot(p[0], p[1]) < 0.3
            and 0.0 <= p[2] < 0.2
        )

        publishers = sum(
            len(preflight.get_publishers_info_by_topic(topic))
            for topic in (
                "/quadcopter/cmd_vel",
                "/quadcopter/enable"
            )
        )

        print("\n===== REPLANNING PREFLIGHT =====")
        print("Sensors:", "PASS" if sensors_ok else "FAIL")
        print("Position:", p)
        print("Starting position:", "PASS" if grounded else "FAIL")
        print("Other flight publishers:", publishers)

        if not (sensors_ok and grounded and publishers == 0):
            raise RuntimeError("PREFLIGHT FAILED: No flight permitted")

        preflight.destroy_node()
        preflight = None

        trial = ControlledReplanTrial()

        print("PREFLIGHT PASSED")
        rclpy.spin(trial)

    except KeyboardInterrupt:
        pass

    finally:
        if trial is not None:
            if rclpy.ok() and not trial.dry_run:
                trial.send_velocity()
            trial.destroy_node()

        if preflight is not None:
            preflight.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
