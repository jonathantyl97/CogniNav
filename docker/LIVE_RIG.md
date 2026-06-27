# Phase 4 — Live stereo rig

CogniNav on a physical stereo camera. Open-dataset regression must still pass after calibration or rig changes.

## Supported rigs

| Preset | Camera | Driver (not in CogniNav) |
|--------|--------|---------------------------|
| `realsense_d455` | Intel RealSense D455 IR + IMU | `realsense2_camera` |
| `zed2` | Stereolabs ZED2 | `zed_ros2_wrapper` |

## RealSense D455

```bash
# Host or container with USB passthrough
sudo apt install ros-jazzy-realsense2-camera   # or humble equivalent
ros2 launch realsense2_camera rs_launch.py \
  enable_infra1:=true enable_infra2:=true \
  enable_gyro:=true enable_accel:=true unite_imu_method:=2
```

CogniNav stack:

```bash
./scripts/run_live_viz.sh --rig realsense_d455
# Or inside container:
ros2 launch cogninav_bringup live.launch.py rig:=realsense_d455
```

## ZED2

```bash
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2
./scripts/run_live_viz.sh --rig zed2
```

## Calibration

ORB settings templates:

- `ros2_ws/src/cogninav_bringup/config/orb_realsense_d455.yaml`
- `ros2_ws/src/cogninav_bringup/config/orb_zed2.yaml`

Replace intrinsics and `IMU.T_b_c1` from `camera_info` and factory/calibration extrinsics before trusting ATE on recorded bags.

## Record warehouse bag

```bash
./scripts/record_rig_bag.sh --rig realsense_d455 --name warehouse_aisle1
```

Bags land in `~/Downloads/cogninav/`.

## Regression gate

After rig or calibration edits:

```bash
./benchmarks/run_regression_suite.sh          # EuRoC + TUM-VI (if downloaded)
./benchmarks/run_regression_suite.sh --quick  # EuRoC only
```

Phase 4 is complete when:

1. Live SLAM initializes on the rig (odom + map points publishing).
2. A warehouse rosbag is recorded.
3. Open-dataset regression still passes.
