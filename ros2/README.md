# ROS 2 Source Versions

These standalone Python nodes cover the reactive control prototypes and autonomous cube finder described in the [project documentation](../autonomous-cube-finder/README.md). The table groups related snapshots by behavior. The filenames and available copies do not establish a complete development chronology.

| Source | Behavior and changes |
| --- | --- |
| [avoidance_scan_only.py](prototypes/avoidance_scan_only.py) | LiDAR-only reactive prototype from `avoidence control.txt`. Checks the forward arc and turns toward the side with greater clearance. |
| [avoidance_physical.py](prototypes/avoidance_physical.py) | Adds an odometry subscriber and logs the robot's x/y position while driving. The reactive LiDAR logic is shared with the scan-only prototype. |
| [detect_and_stop.py](prototypes/detect_and_stop.py) | Adds OAK-D camera input, an HSV red mask, a 2,000-pixel threshold, and a stop-and-log sequence on detection. The archived source had invalid assignment syntax in its final branch. This copy uses the equivalent intended Boolean condition, `elif self.cube_detected:`. |
| [autonomous_search_backup.py](autonomous-search/autonomous_search_backup.py) | Right-wall PD search and return, visual approach and reacquisition, and recovery logic. The live overlay groups pose, yaw, and front range on one line. |
| [autonomous_search.py](autonomous-search/autonomous_search.py) | Keeps the backup snapshot's control logic and changes the camera overlays. The saved detection image adds a bottom panel for position, yaw, range, and pixel count. |
| [autonomous_search_clean.py](autonomous-search/autonomous_search_clean.py) | Keeps right-wall following and adds a cube bearing estimate from the horizontal pixel offset and a 69° camera field of view. |
| [autonomous_search (1).py](autonomous-search/autonomous_search%20%281%29.py) | Right-wall variant with bearing estimation and a gentle right-hand arc to acquire the wall when it is farther than 1.5 m. Applies acquisition during both search and return. |
| [autonomous_search_2.py](autonomous-search/autonomous_search_2.py) | Demo snapshot. Switches to left-wall following and leftward acquisition, uses namespace `T5`, lowers the acquisition threshold from 1,500 to 1,000 red pixels, and adds bearing to the saved image annotation. Identical copies in the demo folders and ZIP are represented by this single file. |

## Running a node

Use a sourced ROS 2 environment with `rclpy` and the standard message packages `geometry_msgs`, `sensor_msgs`, and `nav_msgs`. Camera-based scripts also import OpenCV, NumPy, and `cv_bridge` and use an OpenCV display window. Set the chosen script's `NAMESPACE` to match the robot and confirm the scan, odometry, camera, and velocity topics it uses. Each script has its own entry point.

The demo snapshot uses a 1.0 m switch to direct return and a 0.12 m stop radius, with a 1,000-pixel initial red-mask threshold. The report narrative lists 2.5 m, 0.10 m, and 2,000 pixels respectively. Use the snapshot for its recorded configuration.

All eight scripts passed Python syntax compilation. ROS 2 execution and physical robot behavior were not tested as part of this archive.
