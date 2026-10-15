import json
import math
import heapq
import xml.etree.ElementTree as ET

from pathlib import Path
from functools import lru_cache

ws = Path.home() / "drone_ws"

maps = ws / "maps"
worlds = ws / "src/drone_description/worlds"

region_file = maps / "lidar_obstacle_regions.json"
previous_file = worlds / "astar_replanned.json"
world_file = worlds / "dynamic_obstacle_test.sdf"
output = worlds / "astar_lidar_safe_route.json"

if output.exists():
    raise SystemExit("Route already exists. No overwrite.")

regions_data = json.loads(region_file.read_text())
previous = json.loads(previous_file.read_text())

# Known obstacles + LiDAR-estimated unknown regions
boxes = []

for w in regions_data["known_walls"]:
    boxes.append((
        w["name"], w["x"], w["y"],
        w["width"], w["height"]
    ))

for r in regions_data["unknown_regions"]:
    boxes.append((
        r["name"],
        r["center_x"], r["center_y"],
        r["width"], r["height"]
    ))

start = (
    float(previous["start"]["x"]),
    float(previous["start"]["y"])
)

goal = (5.0, 5.0)

margin = 2.10
resolution = 0.25
minimum = -5.0
maximum = 12.0

count = round((maximum-minimum)/resolution)+1


def clearance(point, obstacles):

    x, y = point

    return min(
        math.hypot(
            max(abs(x-ox)-sx/2, 0),
            max(abs(y-oy)-sy/2, 0)
        )
        for _, ox, oy, sx, sy in obstacles
    )


def segment_clearance(a, b, obstacles):

    length = math.dist(a, b)
    steps = max(1, math.ceil(length/0.025))

    return min(
        clearance((
            a[0]+(b[0]-a[0])*i/steps,
            a[1]+(b[1]-a[1])*i/steps
        ), obstacles)
        for i in range(steps+1)
    )


# Plan recovery westward, away from the detected wall.
# This is planning only: no flight commands.
recovery = list(start)
previous_clearance = clearance(recovery, boxes)

for _ in range(150):

    if previous_clearance >= 2.25:
        break

    recovery[0] -= 0.05

    new_clearance = clearance(recovery, boxes)

    if new_clearance < previous_clearance - 1e-6:
        raise SystemExit(
            "ERROR: Recovery direction is not safe."
        )

    previous_clearance = new_clearance

else:
    raise SystemExit("ERROR: Recovery point not found.")

recovery = tuple(recovery)


def to_cell(point):
    return (
        round((point[0]-minimum)/resolution),
        round((point[1]-minimum)/resolution)
    )


def to_xy(cell):
    return (
        minimum+cell[0]*resolution,
        minimum+cell[1]*resolution
    )


@lru_cache(None)
def free(cell):

    if not (
        0 <= cell[0] < count
        and 0 <= cell[1] < count
    ):
        return False

    return clearance(to_xy(cell), boxes) >= margin


def segment_safe(a, b):
    return segment_clearance(a, b, boxes) >= margin


source = to_cell(recovery)
target = to_cell(goal)

if not free(source) or not free(target):
    raise SystemExit("ERROR: Safe start or goal blocked.")

if not segment_safe(recovery, to_xy(source)):
    raise SystemExit("ERROR: Cannot join recovery to grid.")


def heuristic(cell):
    return resolution*math.dist(cell, target)


queue = [(heuristic(source), source)]
cost = {source: 0.0}
parent = {}
visited = set()

while queue:

    _, current = heapq.heappop(queue)

    if current in visited:
        continue

    visited.add(current)

    if current == target:
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

            if not segment_safe(
                to_xy(current), to_xy(nxt)
            ):
                continue

            new_cost = (
                cost[current]
                + resolution*math.hypot(dx, dy)
            )

            if new_cost < cost.get(nxt, math.inf):

                cost[nxt] = new_cost
                parent[nxt] = current

                heapq.heappush(
                    queue,
                    (new_cost+heuristic(nxt), nxt)
                )

if target not in cost:
    raise SystemExit("ERROR: No route found.")

path = [target]

while path[-1] != source:
    path.append(parent[path[-1]])

path.reverse()

points = [to_xy(c) for c in path]

# Simplify only where the entire segment is clear
simplified = [points[0]]
i = 0

while i < len(points)-1:

    for j in range(len(points)-1, i, -1):

        if segment_safe(points[i], points[j]):
            break

    if j <= i:
        raise SystemExit("ERROR: Path simplification failed.")

    simplified.append(points[j])
    i = j

# Independent simulation verification.
# Actual wall geometry is NOT used by A*.
world = ET.parse(world_file).getroot().find("world")
actual = []

for model in world.findall("model"):

    name = model.get("name", "")

    if not (
        name.startswith("wall_")
        or name == "unexpected_wall"
    ):
        continue

    pose = list(map(float, model.findtext("pose").split()))
    size = list(map(float, model.findtext(
        "./link/collision/geometry/box/size"
    ).split()))

    actual.append((
        name, pose[0], pose[1], size[0], size[1]
    ))

route_points = [recovery] + simplified

actual_minimum = min(
    segment_clearance(a, b, actual)
    for a, b in zip(route_points[:-1], route_points[1:])
)

if actual_minimum < 1.8:
    raise SystemExit(
        f"VALIDATION FAILED: Clearance {actual_minimum:.3f} m"
    )

result = {
    "type": "offline_lidar_region_plan",
    "snapshot_start": list(start),
    "recovery_point": list(recovery),
    "recovery_flight_validated": False,
    "planning_clearance_m": margin,
    "simulated_route_clearance_m": actual_minimum,
    "unknown_space_policy": "Not yet validated",
    "waypoints": [
        {"x": x, "y": y, "z": 2.0}
        for x, y in simplified
    ]
}

with output.open("x") as f:
    json.dump(result, f, indent=2)

print("\n===== LIDAR-BASED A* REPLANNING =====")

print(f"Previous stopping position: {start}")
print(f"Proposed recovery point: {recovery}")
print(f"Planning margin: {margin:.2f} m")

print("\n===== NEW ROUTE =====")

for i, (x, y) in enumerate(simplified):
    print(f"{i}: X={x:.2f}, Y={y:.2f}, Z=2.00")

print("\n===== SIMULATION VALIDATION =====")
print(f"Actual route clearance: {actual_minimum:.3f} m")
print("PASS: Post-recovery route geometry check")

print("\nRecovery and flight validation still pending.")
print("Unknown areas have not been verified clear.")
print("DO NOT EXECUTE THIS ROUTE YET.")
print("Saved:", output)
