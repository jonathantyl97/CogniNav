<p align="center">
  <strong style="font-size: 1.6em;">CogniNav</strong><br/>
  <em>Stereo visual SLAM and aisle guidance for warehouse AMRs</em>
</p>

<p align="center">
  <a href="https://github.com/jonathantyl97/CogniNav"><img src="https://img.shields.io/badge/ROS_2-Jazzy-22314E?style=flat-square&logo=ros" alt="ROS 2 Jazzy"></a>
  <a href="https://github.com/jonathantyl97/CogniNav"><img src="https://img.shields.io/badge/SLAM-ORB--SLAM3-blue?style=flat-square" alt="ORB-SLAM3"></a>
  <a href="https://github.com/jonathantyl97/CogniNav"><img src="https://img.shields.io/badge/sensors-stereo_only-555?style=flat-square" alt="Stereo only"></a>
  <a href="https://github.com/jonathantyl97/CogniNav"><img src="https://img.shields.io/badge/viz-Iridescence-6C5CE7?style=flat-square" alt="Iridescence"></a>
  <a href="https://github.com/jonathantyl97/CogniNav"><img src="https://img.shields.io/badge/status-Phase_6-2ea44f?style=flat-square" alt="Phase 6"></a>
</p>

---

**CogniNav** is a ROS 2 stack for autonomous mobile robots in **dynamic warehouses**: stereo ORB-SLAM3 localization, dense stereo depth, floor-line aisle detection, and in-corridor human/vehicle awareness. Visualization uses **[Iridescence](https://github.com/koide3/iridescence)** by default; ORB-SLAM3's Pangolin viewer is optional for SLAM debugging.

| Design choice | Policy |
|---------------|--------|
| Cameras | **Stereo pair only** (no monocular mode) |
| Primary ROS distro | **Jazzy** on Ubuntu 24.04 |
| SLAM core | [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3) (GPLv3) |
| Open-dataset testing | **r2b** (warehouse) + **KITTI** (road) — no hardware required |
| TorWIC | Optional; large/slow — not recommended for daily dev |

---

## Architecture

<p align="center">
  <img src="docs/assets/cogninav-architecture-dataflow-en.png" alt="CogniNav data flow" width="900"/>
</p>

---

## Quick start

### 1. Container

```bash
./docker/cogninav_jazzy.sh
```

### 2. Build ORB-SLAM3 + workspace (first time, inside container)

```bash
cd /root/cogninav
./docker/setup_deps.sh
cd ros2_ws && source /opt/ros/jazzy/setup.bash && colcon build && source install/setup.bash
```

### 3. Download a dataset (see [Datasets](#datasets) below)

### 4. Run

```bash
./scripts/run_warehouse_viz.sh --source r2b --full --build   # warehouse (default)
./scripts/run_kitti_viz.sh --seq 00 --build                # road / lanes
```

---

## Scripts (`scripts/`)

Starter launchers only — download and prep commands live in this README.

| Script | Purpose |
|--------|---------|
| `run_warehouse_viz.sh` | r2b or TorWIC bag replay + viz |
| `run_kitti_viz.sh` | KITTI road bag replay + viz |
| `run_live_viz.sh` | Live stereo rig (needs camera) |
| `run_rig_bag_viz.sh` | Replay a recorded rig bag |
| `record_rig_bag.sh` | Record live rig to `~/Downloads/cogninav/` |

Viewer flags: `--iris` (default), `--pangolin`, `--headless`, `--full` (+ lanes/depth).

---

## Datasets

Files go under `~/Downloads` (mounted as `/root/Downloads` in Docker).

### MobileNet-SSD (optional, for dynamic detection)

Prototxt is vendored in `models/`. Weights are optional — lane detection works without them.

```bash
mkdir -p ~/Downloads/cogninav/models  # or use repo models/
pip3 install gdown --break-system-packages  # if needed

wget -O models/MobileNetSSD_deploy.prototxt \
  https://raw.githubusercontent.com/chuanqi305/MobileNet-SSD/master/voc/MobileNetSSD_deploy.prototxt

gdown 0B3gersZ2cHIxRm5PMWRoTkdHdHc -O models/MobileNetSSD_deploy.caffemodel
```

### r2b_storage (recommended, ~2.9 GB, native ROS 2)

[NVIDIA r2b_storage](https://catalog.ngc.nvidia.com/orgs/nvidia/teams/isaac/resources/r2bdataset2023) — D455 IR stereo, fast replay, dense depth.

```bash
mkdir -p ~/Downloads/warehouse/r2b_storage
cd ~/Downloads/warehouse/r2b_storage
BASE=https://api.ngc.nvidia.com/v2/resources/nvidia/isaac/r2bdataset2023/versions/3/files/r2b_storage
wget -O metadata.yaml "$BASE/metadata.yaml"
wget -O r2b_storage_0.db3 "$BASE/r2b_storage_0.db3"
```

Then:

```bash
./scripts/run_warehouse_viz.sh --source r2b --full --build
./benchmarks/run_dynamic_slam.sh --source r2b
```

### KITTI odometry (road lanes + outdoor stereo, ~22 GB)

[KITTI odometry gray](https://www.cvlibs.net/datasets/kitti/eval_odometry.php) — best for lane-line detection on road imagery.

```bash
mkdir -p ~/Downloads/kitti
cd ~/Downloads/kitti
wget -c https://s3.eu-central-1.amazonaws.com/avg-kitti/data_odometry_gray.zip
unzip -q data_odometry_gray.zip
# → dataset/sequences/00/image_{0,1}/

# Inside container (after colcon build):
source /opt/ros/jazzy/setup.bash
python3 benchmarks/tools/bag_from_kitti.py 00
# → ~/Downloads/kitti/00_ros2/
```

Then:

```bash
./scripts/run_kitti_viz.sh --seq 00 --build
./benchmarks/run_dynamic_slam.sh --source kitti --seq 00
```

### TorWIC (optional warehouse, ~11 GB per sequence)

Large ROS 1 bags; slow interactive replay. Calibrations are already vendored in `config/torwic_calibrations.txt`.

```bash
pip3 install gdown rosbags --break-system-packages
mkdir -p ~/Downloads/warehouse

# Example: aisle_ccw_run_1 (Google Drive id from TorWIC-SLAM)
gdown 1WahCGK7lUGYBvXwcb5M83UHeNwQZJ0-G -O ~/Downloads/warehouse/aisle_ccw_run_1.bag

rosbags-convert \
  --src ~/Downloads/warehouse/aisle_ccw_run_1.bag \
  --dst ~/Downloads/warehouse/aisle_ccw_run_1_ros2 \
  --src-typestore ros1_noetic --dst-typestore ros2_jazzy \
  --include-topic /left_azure/rgb/image_raw/compressed \
  --include-topic /right_azure/rgb/image_raw/compressed \
  --include-topic /left_azure/imu \
  --include-topic /tf_static

rm ~/Downloads/warehouse/aisle_ccw_run_1.bag   # optional, saves disk
./scripts/run_warehouse_viz.sh --source torwic --seq aisle_ccw_run_1
```

Refresh ORB intrinsics from official calibrations:

```bash
gdown 1NVnNEi-9QDoeyrnkxtlv8dHZl4Sc79zw -O config/torwic_calibrations.txt
python3 benchmarks/tools/apply_torwic_calibrations.py
```

---

## Key topics

| Topic | Message |
|-------|---------|
| `/cogninav/odom` | `nav_msgs/Odometry` |
| `/cogninav/map_points` | `sensor_msgs/PointCloud2` |
| `/cogninav/stereo_points` | `sensor_msgs/PointCloud2` |
| `/cogninav/lane_markers` | `visualization_msgs/MarkerArray` |
| `/cogninav/aisle_guidance` | `geometry_msgs/PointStamped` |
| `/cogninav/dynamic_mask` | `sensor_msgs/Image` |
| `/cogninav/slam_mask_stats` | `std_msgs/UInt32` |

Dynamic mask: `corridor_monitor` publishes `/cogninav/dynamic_mask`; ORB-SLAM3 zeroes those pixels before feature extraction.

---

## Benchmarks

| Script | Purpose |
|--------|---------|
| `benchmarks/smoke_warehouse.sh` | Workspace build smoke |
| `benchmarks/run_warehouse_slam.sh` | ORB trajectory on bag |
| `benchmarks/run_warehouse_perception.sh` | Perception topics gate |
| `benchmarks/run_aisle_guidance.sh` | Aisle + dynamic outputs gate |
| `benchmarks/run_dynamic_slam.sh` | Dynamic-mask SLAM gate |
| `benchmarks/run_all_gates.sh` | Full automated gate suite |

```bash
./benchmarks/run_all_gates.sh
```

---

## Live rig (optional, needs hardware)

See [`docker/LIVE_RIG.md`](docker/LIVE_RIG.md).

```bash
./scripts/run_live_viz.sh --rig realsense_d455
./scripts/record_rig_bag.sh --rig realsense_d455 --name my_run
./scripts/run_rig_bag_viz.sh --rig realsense_d455 --full
```

---

## Repository layout

```
CogniNav/
  docker/              # containers, setup_deps.sh, ORB patches
  scripts/             # run_* launchers only
  benchmarks/          # gates, tools (bag_from_kitti, torwic calib)
  ros2_ws/src/         # ROS 2 packages
  models/              # MobileNet prototxt (weights optional)
```

---

## License

CogniNav ROS 2 packages: see per-package `package.xml`. **ORB-SLAM3** is **GPLv3**.

---

<p align="center">
  <sub>CogniNav — navigate structured aisles with stereo SLAM in dynamic warehouses.</sub>
</p>
