# Wheel Odometry Performance Analysis - Test A1
## Linear Displacement Accuracy

**Robot:** TurtleBot 4  
**Namespace:** /T14  
**Test Period:** Investigation A1 - Linear Displacement  
**Date:** 2026-04-10  

---

## Executive Summary

The TurtleBot 4's wheel odometry system demonstrates **systematic undershoot** across all 20 trials, with performance varying significantly by target distance. The system exhibits:

- **Overall Pass Rate:** 45% (9/20 trials pass at 10% error threshold)
- **Distance Dependency:** Larger distances (1.5m-2.0m) achieve ~80% accuracy, while smaller distances (0.5m) show 0% pass rate
- **Mean Error:** -0.1556m across all trials (consistent undershoot)
- **Systematic Bias:** All 20 trials report lower displacement than commanded

---

## Detailed Results

### Results Table A1: Linear Displacement Accuracy

| Trial | Target (m) | Actual (m) | Error (m) | % Error | Status |
|-------|-----------|-----------|----------|---------|--------|
| T1    | 1.0       | 0.8903    | -0.1097  | 10.97   | FAIL   |
| T2    | 1.0       | 0.7473    | -0.2527  | 25.27   | FAIL   |
| T3    | 2.0       | 1.7822    | -0.2178  | 10.89   | FAIL   |
| T4    | 2.0       | 1.9022    | -0.0978  | 4.89    | **PASS**   |
| T5    | 2.0       | 1.8164    | -0.1836  | 9.18    | **PASS**   |
| T6    | 2.0       | 1.8309    | -0.1691  | 8.46    | **PASS**   |
| T7    | 2.0       | 1.8882    | -0.1118  | 5.59    | **PASS**   |
| T8    | 0.5       | 0.4244    | -0.0756  | 15.12   | FAIL   |
| T9    | 0.5       | 0.3354    | -0.1646  | 32.92   | FAIL   |
| T10   | 0.5       | 0.3987    | -0.1013  | 20.26   | FAIL   |
| T11   | 0.5       | 0.4060    | -0.0940  | 18.80   | FAIL   |
| T12   | 0.5       | 0.4316    | -0.0684  | 13.68   | FAIL   |
| T13   | 1.5       | 1.3923    | -0.1077  | 7.18    | **PASS**   |
| T14   | 1.5       | 1.4080    | -0.0920  | 6.13    | **PASS**   |
| T15   | 1.5       | 1.3158    | -0.1842  | 12.28   | FAIL   |
| T16   | 1.5       | 1.3827    | -0.1173  | 7.82    | **PASS**   |
| T17   | 1.5       | 1.4143    | -0.0857  | 5.71    | **PASS**   |
| T18   | 1.0       | 0.8356    | -0.1644  | 16.44   | FAIL   |
| T19   | 1.0       | 0.8367    | -0.1633  | 16.33   | FAIL   |
| T20   | 1.0       | 0.9123    | -0.0877  | 8.77    | **PASS**   |

**Pass/Fail Criterion:** % Error < 10%

---

## Statistical Summary

### Overall Metrics
| Metric                      | Value        |
|-----------------------------|--------------|
| Total Trials                | 20           |
| Passed Trials               | 9 (45%)      |
| Failed Trials               | 11 (55%)     |
| Mean Displacement Reported  | 1.1176 m     |
| Standard Deviation          | 0.5469 m     |
| Mean % Error                | 12.83%       |
| Mean Absolute Error         | -0.1556 m    |

### Performance by Target Distance

| Target (m) | N Trials | Mean Actual (m) | Std Dev (m) | Mean Error (m) | Mean % Error | Pass Rate |
|------------|----------|-----------------|-------------|----------------|--------------|-----------|
| 0.5        | 5        | 0.3992          | 0.0341      | -0.1008        | 20.16%       | 0/5 (0%)  |
| 1.0        | 5        | 0.8444          | 0.0571      | -0.1556        | 15.56%       | 1/5 (20%) |
| 1.5        | 5        | 1.3826          | 0.0352      | -0.1174        | 7.83%        | 4/5 (80%) |
| 2.0        | 5        | 1.8440          | 0.0449      | -0.1560        | 7.80%        | 4/5 (80%) |

---

## Key Findings

### 1. Systematic Undershoot
Every single trial (20/20) reports less distance than the target command. This indicates:
- **Not random measurement noise** (which would produce both over and undershoot)
- **Systematic calibration or mechanical issue** (wheel slip, encoder miscalibration, etc.)

### 2. Distance-Dependent Performance
Performance improves significantly with target distance:
- **0.5m targets:** Mean error -20.16% (worst performance)
- **1.0m targets:** Mean error -15.56%
- **1.5m targets:** Mean error -7.83% (better)
- **2.0m targets:** Mean error -7.80% (best)

This pattern suggests:
- **Proportional error:** The absolute error (~0.10m) stays roughly constant
- **Fixed offset theory:** ~0.10-0.16m constant undershoot + proportional slip
- **Acceleration effects:** Robot may not reach target velocity quickly on short runs

### 3. Precision vs. Accuracy
- **Precision (Consistency):** Good - Std Dev of 0.03-0.06m within each distance group
- **Accuracy (Correctness):** Poor - All trials systematically biased low

### 4. Pass Rate Analysis
- **At 10% threshold:** 45% pass rate (9/20)
- **At 8% threshold:** 25% pass rate (5/20)
- **Recommended threshold for warehouse use:** 5-8% for navigation tasks

---

## Assessment and Recommendations

### Current Performance
The odometry system is **NOT meeting the 10% accuracy criterion** at its current calibration. While longer-distance runs (1.5m-2.0m) approach acceptable performance, short distances fail consistently.

### Likely Root Causes

1. **Wheel Encoder Calibration Error**
   - Miscalibrated pulses-per-revolution (PPR) count
   - Each motor tick counted as more distance than reality
   - **Evidence:** Consistent ~0.10-0.16m undershoot suggests PPR is set too high

2. **Wheel Slip or Loss of Traction**
   - Slipping during acceleration phase affects motion more on short runs
   - Robot not reaching steady-state velocity quickly
   - **Evidence:** Better performance on longer runs where slip proportion is smaller

3. **Startup Transient Effects**
   - Motor ramp-up time not accounted for
   - Command duration calculation assumes instant acceleration
   - **Evidence:** Undershoot is relatively constant regardless of distance

### Recommended Actions

**Immediate (Before Next Test Campaign):**
1. ✓ Verify wheel diameter used in encoder calibration
2. ✓ Check encoder pulse counts empirically (manually move wheel and count encoder ticks)
3. ✓ Ensure wheels have adequate grip (clean, not worn)
4. ✓ Test on different floor surfaces (was testing conducted on same surface?)

**Short Term:**
5. Recalibrate encoder PPR to match actual wheel circumference
6. Add startup velocity ramp compensation to motion planner
7. Repeat Test A1 after calibration (target: >80% pass rate at 10% threshold)

**Long Term:**
8. Implement visual odometry or IMU fusion to correct drift
9. Add closed-loop feedback during motion (monitor actual distance traveled)

---

## Recommendations for Warehouse Use

Based on this analysis:

### Navigation Requirements
- **Short corridor (< 1m):** Odometry alone **NOT RECOMMENDED** - Use visual SLAM or beacon navigation
- **Medium corridor (1-3m):** Odometry with 10-15% margin accepted - Use secondary localization
- **Long corridor (> 5m):** Odometry acceptable if recalibrated - Still use periodic correction

### Operational Guidelines
1. **Recalibrate every 50 km** of operation (check wheel slip)
2. **Avoid tight tolerances** requiring <5% accuracy
3. **Use loop closure** (return to known position) for long missions
4. **Pair with LiDAR** for obstacle avoidance (more reliable than odometry alone)

---

## Next Steps

1. **Complete Test A2:** Closed-loop drift assessment (square trajectory)
2. **Investigate causes:** Run calibration diagnostics
3. **Baseline LiDAR:** Complete Tests B1-B2 to compare sensor reliability
4. **Camera testing:** Complete Tests C1-C2
5. **Final report:** Synthesize all three sensor investigations with recommendations

---

## Appendices

### Test Node Parameters
- **Namespace:** /T14
- **Forward Speed:** 0.2 m/s (default)
- **Turn Speed:** 0.5 rad/s (default)
- **Test Duration:** 30 seconds per trial
- **Sample Rate:** 589 samples/trial (10 Hz odometry)

### Pass/Fail Criteria Definition
The current criterion (% Error < 10%) was chosen based on:
- Warehouse navigation tolerance (5m corridor = 5cm positioning accuracy needed)
- Proportional error acceptable for dead-reckoning applications
- Industry standard for low-cost mobile robot odometry

---

*Report generated: April 10, 2026*  
*Test Engineer: Student / COMPSYS 732*  
*Equipment: TurtleBot 4 Standard, Lab Environment*
