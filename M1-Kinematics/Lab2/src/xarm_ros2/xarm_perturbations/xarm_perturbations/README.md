# xArm Lite 6 Challenge — CTC vs PD Under Perturbations

Task-space control via inverse kinematics on the xArm Lite 6. Compares Computed Torque Control (CTC) against a PD baseline under external perturbations.

## Project Structure

```
xarm_perturbations/
├── challenge_ik.py            # FK, Jacobian, weighted resolved-rate IK, trajectory generation
├── challenge_controllers.py   # PD baseline and CTC controllers
├── challenge_runner.py        # ROS2 node — executes trials and logs data
├── challenge_analysis.py      # Post-experiment metrics and plot generation
├── perturbation_injector.py   # Gaussian/sine perturbation publisher
├── custom_maker.py            # Previous lab PD controller (lemniscate trajectory)
├── circle_maker.py            # Circle trajectory follower
└── results/challenge/         # Trial data output directory
```

## Reference Code

This implementation is based on `compare_robust_ctc_vs_pid_lite6_IK_fixed_enhanced.py`. The mapping is:

| Reference Component | Implementation File | Function/Class |
|---|---|---|
| Weighted resolved-rate IK (W = diag(1,1,wz)) | `challenge_ik.py` | `weighted_resolved_rate_ik()` |
| DLS pseudo-inverse | `challenge_ik.py` | Inside IK loop |
| Nullspace posture stabilization | `challenge_ik.py` | Inside IK loop |
| Jdot estimation | `challenge_ik.py` | Finite-difference on qd |
| PD/PID baseline controller | `challenge_controllers.py` | `PDController` |
| CTC controller with robust term | `challenge_controllers.py` | `CTCController` |
| Robot dynamics (M, C, G, F) | `challenge_controllers.py` | `get_robot_dynamics()` |
| Trial execution | `challenge_runner.py` | `ChallengeRunner` node |
| Metrics and plots | `challenge_analysis.py` | `compute_metrics()`, `plot_*()` |

## Setup

```bash
cd ~/ros2_ws
colcon build --packages-select xarm_perturbations
source install/setup.bash
```

## Running the 4 Required Trials

All trials use the same IK-generated references and waypoint definition.

### Trial 1 — CTC, no perturbations
```bash
ros2 run xarm_perturbations challenge_runner --ros-args \
  -p controller:=ctc -p perturbations:=false
```

### Trial 2 — PD, no perturbations
```bash
ros2 run xarm_perturbations challenge_runner --ros-args \
  -p controller:=pd -p perturbations:=false
```

### Trial 3 — CTC, with perturbations
Terminal 1 (perturbation injector):
```bash
ros2 run xarm_perturbations perturbation_injector --ros-args \
  -p output_topic:=/servo_server/delta_twist_cmds \
  -p enabled:=true \
  -p mode:=gaussian \
  -p noise_std_linear:=0.01
```

Terminal 2 (controller):
```bash
ros2 run xarm_perturbations challenge_runner --ros-args \
  -p controller:=ctc -p perturbations:=true
```

### Trial 4 — PD, with perturbations
Same perturbation injector as Trial 3, then:
```bash
ros2 run xarm_perturbations challenge_runner --ros-args \
  -p controller:=pd -p perturbations:=true
```

## Task Definition

9 waypoints (8 distinct + return to home), 2 Z levels with 0.08 m separation, quintic spline transitions, 1.0–1.5 s dwells.

| WP | X (m) | Y (m)  | Z (m) | Layer |
|----|-------|--------|-------|-------|
| 1  | 0.300 | +0.100 | 0.150 | Low   |
| 2  | 0.350 | +0.050 | 0.150 | Low   |
| 3  | 0.320 | -0.080 | 0.150 | Low   |
| 4  | 0.280 | -0.100 | 0.150 | Low   |
| 5  | 0.280 | +0.100 | 0.230 | High  |
| 6  | 0.330 | +0.080 | 0.230 | High  |
| 7  | 0.350 | -0.050 | 0.230 | High  |
| 8  | 0.300 | -0.100 | 0.230 | High  |
| 9  | 0.300 | +0.100 | 0.150 | Low   |

## IK Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| wz | 2.5 | Z-axis weight |
| lambda | 0.015 | DLS damping |
| k_task | 14.0 | Task-space error gain |
| k_null | 1.5 | Nullspace posture gain |

## Post-Experiment Analysis

After all 4 trials are complete:

```bash
python3 -m xarm_perturbations.challenge_analysis \
  --data_dir path/to/results/challenge/
```

This generates:
- Joint tracking plots (6 subplots per trial)
- Task-space plots (X, Y, Z components + 3D path)
- End-effector error norm over time
- Phase portraits (e vs e_dot, all 6 joints)
- CTC vs PD comparison overlays
- Summary comparison table and `metrics_summary.csv`

## Controller Parameters

### PD Baseline
- Kp = diag(40, 40, 40, 30, 25, 20)
- Kd = diag(14, 14, 14, 10, 8, 6)
- Torque limit: 10 Nm

### CTC
- Kp = diag(80, 80, 80, 60, 50, 40)
- Kd = diag(28, 28, 28, 20, 16, 12)
- Robust term: gamma=2.5, k_robust=5.5
- Torque limit: 10 Nm

## Output Data Format

Each trial produces:
- `trial_<ctrl>_<pert>_<timestamp>.csv` — time-stamped data at control rate
- `trial_<ctrl>_<pert>_<timestamp>_metadata.json` — parameters and configuration

CSV columns: time, q1–q6, qd1–qd6, q_des1–q_des6, qd_des1–qd_des6, qdd_des1–qdd_des6, px/py/pz, px_des/py_des/pz_des, cmd_x/cmd_y/cmd_z, sat1–sat6, pert_enabled
