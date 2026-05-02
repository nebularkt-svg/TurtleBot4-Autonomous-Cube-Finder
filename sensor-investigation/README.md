# TurtleBot 4 Sensor Investigation

The investigation assessed whether TurtleBot 4 could repeat routes, detect obstacles, and identify colour-coded checkpoints in a warehouse scenario. Tests used several TurtleBot 4 units with the same hardware specification, ROS 2 stack, and sensor drivers.

| Test | Method | Reported result |
| --- | --- | --- |
| A1 Linear motion | 20 runs over 0.5, 1.0, 1.5, and 2.0 m at 0.2 m/s | Mean signed command-to-travel error was −12.83%. Nine runs met the 10% tolerance. Odometry readings averaged −0.50% relative to physical measurements. |
| A2 Square loop | Three runs around a 1 m × 1 m square | Mean closing error was 0.0900 m, or 2.25% of the 4 m route. All three met the final report's 5% criterion. |
| B1 LiDAR ranging | Nine trials at 0.5, 1.0, and 2.0 m, with 50 samples per trial | All trials met the ±6% tolerance. Reported errors ranged from +0.26% to +5.10%. |
| B2 LiDAR snapshots | Four orientations in 90° increments | The proportion of valid returns ranged from 70.8% to 74.2%. Walls and obstacles appeared in all four snapshots. |
| C1–C2 Red marker detection | Camera observations at 0.5–2.5 m and a backlighting comparison | The combined target produced 16,654 red pixels at 0.5 m and 3,399 at 2.5 m. Strong backlighting prevented detection of an opaque red target in the comparison test. |

The linear runs showed a roughly fixed displacement shortfall, so fractional error was greater at short distances. The report considered motor startup effects, slip, and calibration as possible contributors. Encoder-based displacement agreed closely with physical measurements in the tested runs. The report found the platform suitable for mapped routes under normal conditions and called for checking encoder calibration and lighting before deployment.

The A2 working analysis uses a separate 0.10 m threshold, under which two of three runs pass. The final report uses 5% of the 4 m route, equivalent to 0.20 m. The B1 table gives a 0.5 m error of about 0.0013 m (1.3 mm), used here in preference to the conflicting millimetre conversion in the report prose.

## Supporting material

- [A1 working analysis](Odometry_Test_A1_Report.md) and [trial table](data/Odometry_Test_A1_Data.csv)
- [A2 working analysis](Odometry_Test_A2_Report.md) and [trial table](data/Odometry_Test_A2_Data.csv)
- [LiDAR range statistics](data/lidar_range_stats.csv)
- [Camera detection measurements](data/detections.csv)

Source: the April 20, 2026 TurtleBot 4 sensor investigation report and the corresponding laboratory records.
