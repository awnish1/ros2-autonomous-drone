import math
import time
import json
import heapq
import xml.etree.ElementTree as ET

from pathlib import Path
from functools import lru_cache

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


class SensorSnapshot(Node):

    def __init__(self):
        super().__init__('lidar_replan_snapshot')

        self.odom = None
        self.scan = None

        self.create_subscription(
            Odometry,
            '/model/quadcopter/odometry',
            self.odom_callback,
            qos_profile_sensor_data
        )

        self.create_subscription(
            LaserScan,
            '/quadcopter/scan',
            self.scan_callback,
            qos_profile_sensor_data
        )

    def odom_callback(self, msg):
        self.odom = msg

    def scan_callback(self, msg):
        self.scan = msg


# Collect live sensor measurements
rclpy.init()
node = SensorSnapshot()

deadline = time.monotonic() + 15.0

try:
    while (
        (node.odom is None or node.scan is None)
        and time.monotonic() < deadline
    ):
        rclpy.spin_once(node, timeout_sec=0.2)

    odom = node.odom
    scan = node.scan

finally:
    node.destroy_node()
    rclpy.shutdown()

if odom is None or scan is None:
    raise SystemExit(
        "ERROR: No sensor data. Check Gazebo, bridge and Play."
    )

p = odom.pose.pose.position
q = odom.pose.pose.orientation

x, y = p.x, p.y

yaw = math.atan2(
    2 * (q.w*q.z + q.x*q.y),
    1 - 2 * (q.y*q.y + q.z*q.z)
)

print(f"Drone position: ({x:.2f}, {y:.2f})")

# Read only the originally known walls
world_file = Path(
    "src/drone_description/worlds/obstacle_world.sdf"
)

world = ET.parse(world_file).getroot().find("world")

known = []

for model in world.findall("model"):

    if not model.get("name", "").startswith("wall_"):
        continue

    pose = list(map(float, model.findtext("pose").split()))

    size = list(map(
        float,
        model.findtext(
            "./link/collision/geometry/box/size"
        ).split()
    ))

    known.append((
        pose[0], pose[1],
        size[0], size[1]
    ))


# Convert LiDAR detections to world coordinates
unknown_hits = []

for i, distance in enumerate(scan.ranges):

    if not (
        math.isfinite(distance)
        and scan.range_min <= distance <= scan.range_max
    ):
        continue

    angle = yaw + scan.angle_min + i * scan.angle_increment

    px = x + distance * math.cos(angle)
    py = y + distance * math.sin(angle)

    # Ignore hits on already known walls
    known_hit = any(
        abs(px - ox) <= sx/2 + 0.15
        and abs(py - oy) <= sy/2 + 0.15
        for ox, oy, sx, sy in known
    )

    if not known_hit:
        unknown_hits.append((px, py))

if not unknown_hits:
    raise SystemExit(
        "No unknown LiDAR obstacle detected. "
        "Replanning cancelled."
    )

print(f"Unknown LiDAR hit points: {len(unknown_hits)}")

# Grid configuration
resolution = 0.25
inflation = 1.35

minimum = -4.0
maximum = 10.0

count = round((maximum - minimum) / resolution) + 1


def to_cell(px, py):
    return (
        round((px - minimum) / resolution),
        round((py - minimum) / resolution)
    )


def to_xy(cell):
    return (
        minimum + cell[0]*resolution,
        minimum + cell[1]*resolution
    )


@lru_cache(maxsize=None)
def free(cell):

    a, b = cell

    if not (0 <= a < count and 0 <= b < count):
        return False

    px, py = to_xy(cell)

    # Known walls
    for ox, oy, sx, sy in known:

        if (
            abs(px-ox) <= sx/2 + inflation
            and abs(py-oy) <= sy/2 + inflation
        ):
            return False

    # Newly detected obstacles
    for ox, oy in unknown_hits:

        if math.hypot(px-ox, py-oy) <= inflation:
            return False

    return True


start = to_cell(x, y)
goal = to_cell(5.0, 5.0)

if not free(start):
    raise SystemExit(
        "Current position lies inside the inflated obstacle map. "
        "A controlled retreat is needed before replanning."
    )

if not free(goal):
    raise SystemExit("Destination is blocked.")


def heuristic(cell):
    return resolution * math.hypot(
        goal[0]-cell[0],
        goal[1]-cell[1]
    )


# Run A*
queue = [(heuristic(start), 0.0, start)]

cost = {start: 0.0}
parent = {}

while queue:

    _, g, current = heapq.heappop(queue)

    if g > cost[current] + 1e-9:
        continue

    if current == goal:
        break

    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):

            if dx == 0 and dy == 0:
                continue

            nxt = (current[0]+dx, current[1]+dy)

            if not free(nxt):
                continue

            if dx and dy:
                if not (
                    free((current[0]+dx, current[1]))
                    and free((current[0], current[1]+dy))
                ):
                    continue

            new_cost = (
                g + resolution * math.hypot(dx, dy)
            )

            if new_cost < cost.get(nxt, float('inf')):

                cost[nxt] = new_cost
                parent[nxt] = current

                heapq.heappush(
                    queue,
                    (
                        new_cost + heuristic(nxt),
                        new_cost,
                        nxt
                    )
                )

if goal not in cost:
    raise SystemExit(
        "No alternative route found with current LiDAR data."
    )

path = [goal]

while path[-1] != start:
    path.append(parent[path[-1]])

path.reverse()

# Retain route turning points
corners = [path[0]]
previous = None

for i in range(1, len(path)):

    a = path[i-1]
    b = path[i]

    direction = (b[0]-a[0], b[1]-a[1])

    if previous is not None and direction != previous:
        corners.append(a)

    previous = direction

corners.append(goal)

waypoints = []

for cell in corners:
    px, py = to_xy(cell)
    waypoints.append({
        'x': round(px, 3),
        'y': round(py, 3),
        'z': 2.0
    })

output = Path(
    "src/drone_description/worlds/astar_replanned.json"
)

if output.exists():
    raise SystemExit(
        "Replanned route already exists. "
        "Existing file preserved; no overwrite."
    )

result = {
    'start': {'x': x, 'y': y, 'z': p.z},
    'goal': {'x': 5.0, 'y': 5.0, 'z': 2.0},
    'unknown_lidar_hits': len(unknown_hits),
    'inflation_m': inflation,
    'route_length_m': round(cost[goal], 2),
    'waypoints': waypoints
}

output.write_text(json.dumps(result, indent=2) + '\n')

print("\n===== NEW A* ROUTE =====")
print(f"Route length: {cost[goal]:.2f} m")

for i, point in enumerate(waypoints):
    print(
        f"{i}: "
        f"X={point['x']:.2f}, "
        f"Y={point['y']:.2f}, "
        f"Z={point['z']:.2f}"
    )

print("\nSUCCESS: New A* route calculated!")
print("Saved:", output)
print("NOTE: Route not yet validated for flight.")
