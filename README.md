# TurtleBot 4 Autonomous Cube Finder

A COMPSYS 732 ROS 2 project that characterises the TurtleBot 4 sensor suite and uses it to find a red cube in a mapped arena. The controller combines LiDAR wall following, camera-based detection, and odometry to search, approach the target, and return to the start.

| Project phase | Documentation and evidence |
| --- | --- |
| Sensor investigation | [Wheel odometry, LiDAR, and camera tests](sensor-investigation/README.md) |
| Autonomous demo | [Controller states, SLAM alignment, and results](autonomous-cube-finder/README.md) |
| ROS 2 implementation | [Three prototypes and five autonomous search variants](ros2/README.md) |
| Run evidence | [Demo log and occupancy map](autonomous-cube-finder/evidence/) |

The demo analysis reports a **0.154 m** difference between the estimated cube location and the SLAM-map target. The return log records a separate **0.117 m** error from the starting point. The demo and sensor documents describe how the two errors were measured.
