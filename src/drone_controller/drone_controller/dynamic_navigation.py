import heapq
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

RESOLUTION = 0.25
MIN_COORD = -5.0
MAX_COORD = 12.0
MARGIN = 2.10


class AStarPlanner:

    def __init__(self, obstacles):
        self.obstacles = obstacles
        self.cells = round(
            (MAX_COORD - MIN_COORD) / RESOLUTION
        ) + 1

    def clearance(self, point):
        x, y = point

        return min(
            math.hypot(
                max(abs(x - ox) - sx / 2, 0),
                max(abs(y - oy) - sy / 2, 0)
            )
            for _, ox, oy, sx, sy in self.obstacles
        )

    def to_cell(self, point):
        return (
            round((point[0] - MIN_COORD) / RESOLUTION),
            round((point[1] - MIN_COORD) / RESOLUTION)
        )

    def to_xy(self, cell):
        return (
            MIN_COORD + cell[0] * RESOLUTION,
            MIN_COORD + cell[1] * RESOLUTION
        )

    def free(self, cell):
        if not (
            0 <= cell[0] < self.cells
            and 0 <= cell[1] < self.cells
        ):
            return False

        return self.clearance(self.to_xy(cell)) >= MARGIN

    def segment_safe(self, a, b):
        steps = max(1, math.ceil(math.dist(a, b) / 0.025))

        for i in range(steps + 1):
            t = i / steps
            point = (
                a[0] + t * (b[0] - a[0]),
                a[1] + t * (b[1] - a[1])
            )
            if self.clearance(point) < MARGIN:
                return False

        return True

    def plan(self, start, goal):
        source = self.to_cell(start)
        target = self.to_cell(goal)

        if not self.free(source) or not self.free(target):
            raise ValueError(
                "Start or goal is inside an inflated obstacle."
            )

        if not self.segment_safe(start, self.to_xy(source)):
            raise ValueError("Cannot safely join starting grid cell.")

        if not self.segment_safe(goal, self.to_xy(target)):
            raise ValueError("Cannot safely join goal grid cell.")

        def heuristic(c):
            return RESOLUTION * math.dist(c, target)

        queue = [(heuristic(source), 0.0, source)]
        cost = {source: 0.0}
        parent = {}

        while queue:
            _, current_cost, current = heapq.heappop(queue)

            if current_cost > cost[current] + 1e-9:
                continue

            if current == target:
                break

            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    if dx == 0 and dy == 0:
                        continue

                    nxt = (
                        current[0] + dx,
                        current[1] + dy
                    )

                    if not self.free(nxt):
                        continue

                    if dx and dy:
                        if not (
                            self.free((current[0] + dx, current[1]))
                            and self.free((current[0], current[1] + dy))
                        ):
                            continue

                    if not self.segment_safe(
                        self.to_xy(current),
                        self.to_xy(nxt)
                    ):
                        continue

                    new_cost = (
                        current_cost
                        + RESOLUTION * math.hypot(dx, dy)
                    )

                    if new_cost < cost.get(nxt, math.inf):
                        cost[nxt] = new_cost
                        parent[nxt] = current

                        heapq.heappush(
                            queue,
                            (new_cost + heuristic(nxt), new_cost, nxt)
                        )

        if target not in cost:
            raise ValueError("A* could not find a safe route.")

        path = [target]

        while path[-1] != source:
            path.append(parent[path[-1]])

        path.reverse()
        points = [self.to_xy(c) for c in path]

        # Shorten the route without cutting through obstacles.
        simplified = [points[0]]
        i = 0

        while i < len(points) - 1:
            for j in range(len(points) - 1, i, -1):
                if self.segment_safe(points[i], points[j]):
                    break

            simplified.append(points[j])
            i = j

        route = [start] + simplified + [goal]

        # Remove consecutive duplicate positions.
        result = []
        for point in route:
            if not result or math.dist(point, result[-1]) > 1e-6:
                result.append(point)

        return result


def load_obstacles():
    ws = Path.home() / "drone_ws"

    map_file = ws / "maps/lidar_obstacle_regions.json"
    data = json.loads(map_file.read_text())

    known = [
        (
            wall["name"], wall["x"], wall["y"],
            wall["width"], wall["height"]
        )
        for wall in data["known_walls"]
    ]

    discovered = [
        (
            region["name"],
            region["center_x"],
            region["center_y"],
            region["width"],
            region["height"]
        )
        for region in data["unknown_regions"]
    ]

    return known, discovered


def actual_obstacles():
    # Used only for independent simulation validation.
    path = (
        Path.home()
        / "drone_ws/src/drone_description/worlds/"
        "dynamic_obstacle_test.sdf"
    )

    world = ET.parse(path).getroot().find("world")
    boxes = []

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

        boxes.append((
            name, pose[0], pose[1], size[0], size[1]
        ))

    return boxes


def minimum_route_clearance(route, obstacles):
    checker = AStarPlanner(obstacles)
    minimum = math.inf

    for a, b in zip(route[:-1], route[1:]):
        steps = max(1, math.ceil(math.dist(a, b) / 0.025))

        for i in range(steps + 1):
            t = i / steps
            p = (
                a[0] + t * (b[0] - a[0]),
                a[1] + t * (b[1] - a[1])
            )
            minimum = min(minimum, checker.clearance(p))

    return minimum


def main():
    known, discovered = load_obstacles()
    actual = actual_obstacles()

    goal = (5.0, 5.0)

    print("\n===== DYNAMIC NAVIGATION DRY RUN =====")
    print("FLIGHT COMMANDS: DISABLED")

    # First plan: only the three known walls.
    initial = AStarPlanner(known).plan((0.0, 0.0), goal)

    print("\nINITIAL A* ROUTE")
    print("Known obstacles:", len(known))
    print("Waypoints:", len(initial))

    initial_clearance = minimum_route_clearance(initial, actual)

    print(
        f"Clearance with unexpected wall: "
        f"{initial_clearance:.3f} m"
    )

    # Simulate discovering the new wall.
    print("\nUNEXPECTED OBSTACLE DISCOVERED")
    print("New LiDAR regions:", len(discovered))

    ws = Path.home() / "drone_ws"

    cached = json.loads((
        ws / "src/drone_description/worlds/"
        "astar_lidar_safe_route.json"
    ).read_text())

    recovery = tuple(cached["recovery_point"])

    print("Saved recovery proposal:", recovery)

    # Replan from the proposed post-recovery position.
    updated = AStarPlanner(
        known + discovered
    ).plan(recovery, goal)

    clearance = minimum_route_clearance(updated, actual)

    print("\nUPDATED A* ROUTE")

    for i, (x, y) in enumerate(updated):
        print(f"{i}: X={x:.2f}, Y={y:.2f}, Z=2.00")

    print("\n===== SAFETY CHECK =====")
    print(f"Post-recovery clearance: {clearance:.3f} m")

    if clearance < 1.8:
        raise SystemExit(
            "FAILED: Updated route is too close to a wall."
        )

    print("PASS: Post-recovery geometry check")
    print("Recovery manoeuvre: NOT YET VALIDATED")
    print("Live flight replanning: NOT YET ENABLED")
    print("\nSUCCESS: Dynamic planning dry run completed!")


if __name__ == '__main__':
    main()
