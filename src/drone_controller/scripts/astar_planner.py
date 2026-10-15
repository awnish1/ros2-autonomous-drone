import heapq
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

# Files
world_path = Path(
    "src/drone_description/worlds/obstacle_world.sdf"
)

route_path = Path(
    "src/drone_description/worlds/astar_route.json"
)

# Read obstacles directly from Gazebo
world = ET.parse(world_path).getroot().find("world")

if world is None:
    raise SystemExit("ERROR: World not found")

obstacles = []

for model in world.findall("model"):

    name = model.get("name", "")

    if not name.startswith("wall_"):
        continue

    pose = model.findtext("pose")
    size = model.findtext(
        "./link/collision/geometry/box/size"
    )

    if not pose or not size:
        raise SystemExit(f"Invalid obstacle: {name}")

    x, y, z, *_ = map(float, pose.split())
    sx, sy, sz = map(float, size.split())

    if z - sz / 2 <= 2.0 <= z + sz / 2:
        obstacles.append((name, x, y, sx, sy))

if not obstacles:
    raise SystemExit("ERROR: No obstacles found")

# Grid configuration
resolution = 0.25
margin = 1.9

minimum = -3.5
maximum = 9.0

cells = round((maximum - minimum) / resolution) + 1


def to_xy(cell):
    return (
        round(minimum + cell[0] * resolution, 3),
        round(minimum + cell[1] * resolution, 3)
    )


def to_cell(x, y):
    return (
        round((x - minimum) / resolution),
        round((y - minimum) / resolution)
    )


def free(cell):

    if not (0 <= cell[0] < cells and 0 <= cell[1] < cells):
        return False

    x, y = to_xy(cell)

    for _, ox, oy, sx, sy in obstacles:

        if (
            abs(x - ox) <= sx / 2 + margin
            and abs(y - oy) <= sy / 2 + margin
        ):
            return False

    return True


start = to_cell(0, 0)
goal = to_cell(5, 5)

if not free(start) or not free(goal):
    raise SystemExit("ERROR: Start or goal is blocked")


def heuristic(cell):
    return resolution * math.hypot(
        goal[0] - cell[0],
        goal[1] - cell[1]
    )


# A* search
queue = [(heuristic(start), 0.0, start)]

cost = {start: 0.0}
parent = {}

while queue:

    _, current_cost, current = heapq.heappop(queue)

    if current_cost > cost[current] + 1e-9:
        continue

    if current == goal:
        break

    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):

            if dx == 0 and dy == 0:
                continue

            neighbor = (
                current[0] + dx,
                current[1] + dy
            )

            if not free(neighbor):
                continue

            # Prevent diagonal corner cutting
            if dx != 0 and dy != 0:
                if not (
                    free((current[0] + dx, current[1]))
                    and free((current[0], current[1] + dy))
                ):
                    continue

            new_cost = (
                current_cost
                + resolution * math.hypot(dx, dy)
            )

            if new_cost < cost.get(neighbor, float("inf")):

                cost[neighbor] = new_cost
                parent[neighbor] = current

                heapq.heappush(
                    queue,
                    (
                        new_cost + heuristic(neighbor),
                        new_cost,
                        neighbor
                    )
                )

if goal not in cost:
    raise SystemExit("ERROR: No path found")

# Reconstruct path
path = [goal]

while path[-1] != start:
    path.append(parent[path[-1]])

path.reverse()

# Keep only turning points
corners = [path[0]]
previous_direction = None

for i in range(1, len(path)):

    a = path[i - 1]
    b = path[i]

    direction = (
        b[0] - a[0],
        b[1] - a[1]
    )

    if (
        previous_direction is not None
        and direction != previous_direction
    ):
        corners.append(a)

    previous_direction = direction

corners.append(path[-1])

waypoints = []

for cell in corners:

    x, y = to_xy(cell)

    waypoints.append({
        "x": x,
        "y": y,
        "z": 2.0
    })

result = {
    "world": "obstacle_world.sdf",
    "resolution_m": resolution,
    "inflation_m": margin,
    "distance_m": round(cost[goal], 2),
    "waypoints": waypoints
}

# Preserve a previous generated route
if route_path.exists():
    backup = route_path.with_name("astar_route.previous.json")
    if not backup.exists():
        backup.write_bytes(route_path.read_bytes())

route_path.write_text(json.dumps(result, indent=2) + "\n")

print("\n===== A* PATH PLANNING =====")
print(f"Obstacles: {len(obstacles)}")
print(f"Route length: {cost[goal]:.2f} metres")

print("\nWaypoints:")

for i, waypoint in enumerate(waypoints):
    print(
        f"{i}: "
        f"X={waypoint['x']:.2f}, "
        f"Y={waypoint['y']:.2f}, "
        f"Z={waypoint['z']:.2f}"
    )

print("\nSUCCESS: A* route calculated!")
print("Saved to:", route_path)
