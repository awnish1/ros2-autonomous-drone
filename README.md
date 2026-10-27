# Autonomous Drone Navigation with ROS 2, A* and LiDAR

![ROS 2](https://img.shields.io/badge/ROS_2-Jazzy-22314E?logo=ros&logoColor=white)
![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-orange)
![Python](https://img.shields.io/badge/Python-3.x-3776AB?logo=python&logoColor=white)
![Status](https://img.shields.io/badge/Status-Simulation_Prototype-informational)

A **ROS 2-based autonomous quadcopter simulation** that combines **LiDAR sensing**, **A* path planning**, **waypoint navigation**, and **reactive obstacle handling** in Gazebo. The project demonstrates how a simulated drone can plan a route, track its position, respond to obstacles, and execute navigation logic with ROS 2 nodes.

> **Project scope:** Simulation and robotics research. This repository is not a flight-certified controller or a validated real-world drone autopilot.

**[Source code](https://github.com/awnish1/ros2-autonomous-drone)** · **[User manual](docs/ROS-2-Fundamentals-User-Manual.pdf)** · **[Installation video](https://www.youtube.com/watch?v=08o46x5SfJM)** · **[Issues](https://github.com/awnish1/ros2-autonomous-drone/issues)**

## Demonstration

**Project demo video:** _Coming soon — replace this line with the public YouTube, Google Drive, or GitHub video link once uploaded._

The demonstration is intended to show the simulated drone taking off, following planned waypoints, monitoring LiDAR measurements, responding to obstacles, and approaching its destination.

## Key Features

- **Gazebo quadcopter simulation:** Simulated vehicle, environment, and LiDAR sensor.
- **A* route planning:** Grid-based path planning with a Euclidean heuristic and 8-connected movement, including diagonals when safe.
- **Waypoint navigation:** A generated route is followed through successive position targets.
- **LiDAR obstacle awareness:** Scan data is used to estimate clearance and identify newly observed obstacles.
- **Obstacle response and replanning:** Navigation can stop, assess a recovery direction, and calculate an updated A* route when conditions permit.
- **Mission control:** Takeoff, navigation, hover, landing, disarming, and safety-hold states in the controller.
- **Testing options:** Dry-run mode and isolated takeoff, waypoint, and recovery tests.

## How It Works

```text
             Gazebo Simulation
          (Quadcopter + LiDAR)
                    |
           ROS 2 / Gazebo Bridge
                    |
          Odometry + LaserScan
                    |
            Obstacle Handling
                    |
             A* Path Planner
                    |
            Route / Waypoints
                    |
           Navigation Controller
                    |
            Velocity Commands
                    |
              Quadcopter
```

The planner searches a 2D occupancy-like grid for a safe route to the goal. The controller then uses simulated odometry to move between route waypoints, while LiDAR scans are checked for obstacles. If a path is blocked, the controller can enter a safety or recovery state and attempt to replan. Altitude is controlled separately during navigation.

## Technology Stack

| Component | Technology |
| --- | --- |
| Robotics middleware | ROS 2 Jazzy |
| Simulator | Gazebo Harmonic |
| Programming language | Python 3 |
| Path planning | A* search |
| Sensor feedback | Simulated 2D LiDAR and odometry |
| Robot interfaces | ROS 2 topics (`LaserScan`, `Odometry`, `Twist`) |
| Build system | `colcon` / `ament` |

## Getting Started

### Prerequisites

- Ubuntu 24.04, ROS 2 Jazzy, and Gazebo Harmonic
- ROS–Gazebo bridge (`ros_gz_bridge`)
- Python 3, `colcon`, and project ROS dependencies

See the **[ROS 2 installation tutorial](https://www.youtube.com/watch?v=08o46x5SfJM)** and the official **[ROS 2 Jazzy documentation](https://docs.ros.org/en/jazzy/)**.

### 1. Clone and build

```bash
# A fresh checkout at ~/drone_ws is recommended because
# some project scripts currently assume this workspace path.
git clone https://github.com/awnish1/ros2-autonomous-drone.git ~/drone_ws
cd ~/drone_ws

source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 2. Start the simulated environment

```bash
cd ~/drone_ws
gz sim -r src/drone_description/worlds/dynamic_obstacle_test.sdf
```

### 3. Connect Gazebo to ROS 2

In another terminal, start the `ros_gz_bridge` topic mappings for the simulated odometry, LiDAR, and controller commands. Use the bridge configuration from your tested setup; simply launching Gazebo does **not** provide the ROS topics to the controller.

Check the topics after bridging:

```bash
source /opt/ros/jazzy/setup.bash
source ~/drone_ws/install/setup.bash
ros2 topic list
ros2 topic echo /model/quadcopter/odometry --once
ros2 topic echo /quadcopter/scan --once
```

### 4. Run the controller in safe dry-run mode

```bash
source /opt/ros/jazzy/setup.bash
source ~/drone_ws/install/setup.bash
ros2 run drone_controller dynamic_autonomy --ros-args -p dry_run:=true
```

> **Important:** `dry_run:=true` prevents flight commands from being published. Only use `dry_run:=false` after the simulator, bridge, and sensor topics have been checked and you understand the controller's operating conditions.

## Repository Structure

```text
ros2-autonomous-drone/
├── src/
│   ├── drone_controller/
│   │   └── drone_controller/
│   │       ├── dynamic_autonomy.py       # Flight state machine and avoidance
│   │       └── dynamic_navigation.py     # A* path planner
│   └── drone_description/
│       ├── models/                       # Quadcopter simulation model
│       └── worlds/                       # Gazebo test environments
├── maps/                                 # LiDAR-derived obstacle/map data
├── docs/                                 # Add your PDF user manual here
└── README.md
```

The `docs/` folder and this `README.md` are documentation additions; upload the manual before using its link above.

## Documentation and Resources

| Resource | Link |
| --- | --- |
| ROS 2 Fundamentals — A Practical User Manual | [Project PDF](docs/ROS-2-Fundamentals-User-Manual.pdf) *(upload required)* |
| Project source code | [GitHub repository](https://github.com/awnish1/ros2-autonomous-drone) |
| ROS 2 installation video | [YouTube guide](https://www.youtube.com/watch?v=08o46x5SfJM) |
| ROS 2 Jazzy documentation | [docs.ros.org](https://docs.ros.org/en/jazzy/) |
| Gazebo Harmonic documentation | [gazebosim.org](https://gazebosim.org/docs/harmonic/) |

## Current Limitations

- Developed and tested as a **simulation prototype**, not on a physical drone.
- Navigation and sensing depend on the simulated environment and correct ROS–Gazebo topic bridging.
- Current path planning uses a 2D representation; it is not a full 3D motion planner.
- Detection and avoidance performance require further quantitative testing across different obstacle layouts and sensor conditions.
- Some file paths and environment settings assume a workspace at `~/drone_ws`.

## Future Improvements

- Neural-network-assisted perception or obstacle classification
- Better map updates and online replanning
- Three-dimensional path planning and trajectory smoothing
- More simulation scenarios and quantitative navigation benchmarks
- Evaluation on real drone hardware with an independent flight-control and safety stack

## Contributors

Aditi Yadav · Ashay Gupta · Awnish Kumar  
Department of Computer Science and Engineering, IIIT Vadodara

---

**Educational and research project — ROS 2 autonomous drone navigation in Gazebo.**
