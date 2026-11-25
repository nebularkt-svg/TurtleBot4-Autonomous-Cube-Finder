# Autonomous Cube Finder

The project integrated TurtleBot 4 sensing and control in a C-shaped arena. The first phase produced a SLAM map containing three candidate cylinders. In the second phase, the robot searched for a red cube on one cylinder, estimated its position, and returned to the start. The reported controller used local sensor feedback during the autonomous run and used the SLAM map for the subsequent position comparison.

## ROS 2 control

LiDAR, camera, and odometry subscribers supply range readings, red-object observations, and the robot pose to a 10 Hz finite state machine.

| State | Behavior described in Report 2 |
| --- | --- |
| SEARCHING | Follow the left wall at a 0.30 m target offset using a PD controller and front-left lookahead. The reported gains are 1.2 and 0.3. Slow down for large lateral errors and turn toward the side with more clearance when an obstacle blocks the front. |
| APPROACHING | Detect red using an HSV mask and a 2,000-pixel acquisition threshold. Use the largest contour's horizontal offset to centre the target while approaching at 0.08 m/s. |
| REACQUIRING | Sweep up to 90° to either side when the cube disappears. Return to searching if the sweep ends or the 10-second window expires. |
| DETECTED | Stop after the mask reaches 10,000 pixels and front clearance is at most 0.30 m. Record pose and save the camera observation. |
| TURNING | Rotate 180° before starting the return. |
| RETURNING | Follow the wall back through the corridor, then use odometry to steer toward the origin. |
| DONE | Stop and record the final position and elapsed time. |
| ESCAPING | Turn in place and resume the prior behavior when odometry indicates prolonged stalling or circling. |

The report describes evaluating RANSAC cylinder fitting and Nav2 waypoint navigation. It records inconsistent cylinder extraction from the available SLAM maps and lag in the navigation setup as reasons for choosing wall following in the test arena. The recorded configuration and report narrative differ in places. For example, the demo script uses a 1.0 m switch from wall following to direct return, while the report describes 2.5 m.

## Cube position error

The map and odometry frames share a start point. The report rotates the map coordinates by 60.64° to align the axes. Cylinder B, which held the cube, maps from approximately (−3.406, −1.874) m to (−0.037, −3.887) m in the odometry frame.

At detection, the report records the robot at (0.265, −3.747) m with yaw 177.4°. Adding a 0.30 m forward offset gives a cube estimate of approximately (−0.035, −3.733) m. The Euclidean separation from the transformed cylinder B position is **0.154 m**. This comparison includes uncertainty from map alignment, odometry drift, camera centring, and LiDAR range.

The demonstration log records a separate return-to-start error of **0.117 m** at a final pose of (0.114, 0.028) m and a run time of **347.7 s**.

## Supporting material

- [Demo log](evidence/demo_log.txt)
- [SLAM map metadata](evidence/demo_map.yaml) and [occupancy grid](evidence/demo_map.pgm)

Source: Report 2 and the corresponding project brief, demonstration script, and run records.

See the [ROS 2 source and version notes](../ros2/README.md) for the archived scripts and their configuration differences.
