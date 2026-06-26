# ORB-SLAM3 ROS 2 wrapper — **stereo / stereo-inertial only**

CogniNav does **not** support monocular SLAM or mono datasets.

## Modes

| Mode | When | ORB-SLAM3 enum |
|------|------|----------------|
| **Stereo-inertial** (default) | EuRoC, TUM-VI, rig with IMU | `System::STEREO_INERTIAL` |
| **Stereo** | KITTI (no IMU in wrapper) | `System::STEREO` |

## Stereo topics (convention)

| Topic | Type |
|-------|------|
| `/cam0/image_raw` | left `sensor_msgs/Image` |
| `/cam1/image_raw` | right `sensor_msgs/Image` |
| `/imu0` | `sensor_msgs/Imu` (stereo-inertial only) |
| `/cogninav/odom` | `nav_msgs/Odometry` |
| `/cogninav/map_points` | sparse SLAM map `PointCloud2` |

Depth is separate: `cogninav_depth` publishes dense `/cogninav/stereo_points` from OpenCV SGBM.

## Vendored ROS 2 wrapper

Stereo executables are adapted from [zang09/ORB_SLAM3_ROS2](https://github.com/zang09/ORB_SLAM3_ROS2) (GPLv3):

| Executable | ORB mode |
|------------|----------|
| `orb_slam3_stereo` | `System::STEREO` |
| `orb_slam3_stereo_inertial` | `System::IMU_STEREO` |

Build requires `third_party/ORB_SLAM3` from `./scripts/setup_deps.sh`.

## Headless / Iridescence

```cpp
ORB_SLAM3::System slam(vocab, settings, System::STEREO_INERTIAL, false);
```

Pangolin may still be required to **compile** stock ORB-SLAM3; viewer is **Iridescence**, not Pangolin.

## Datasets (stereo only)

- EuRoC MAV (stereo-inertial)
- TUM-VI (stereo-inertial)
- KITTI Odometry (stereo)
