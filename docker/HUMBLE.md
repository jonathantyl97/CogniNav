# CogniNav: Jazzy vs Humble differences (Phase 3)

| Item | Jazzy (Ubuntu 24.04) | Humble (Ubuntu 22.04) |
|------|----------------------|------------------------|
| Base image | `osrf/ros:jazzy-desktop-full` | `osrf/ros:humble-desktop` |
| Container script | `docker/cogninav_jazzy.sh` | `docker/cogninav_humble.sh` |
| ORB patch | `docker/patch_orb_jazzy.sh` | `docker/patch_orb_humble.sh` |
| OpenCV (CMake) | 4.6 | 4.5 |
| `setup_deps.sh` | auto-detects distro | auto-detects distro |
| Pip extras | `--break-system-packages` on 24.04 | not required on 22.04 |

**Important:** `third_party/ORB_SLAM3` is shared via the repo mount. Rebuild ORB inside each container before SLAM:

```bash
# Jazzy
./docker/cogninav_jazzy.sh
./docker/setup_deps.sh
cd ros2_ws && colcon build

# Humble
./docker/cogninav_humble.sh
./docker/setup_deps.sh
cd ros2_ws && colcon build
```

**Phase 3 gate** (from host):

```bash
./benchmarks/run_gate.sh --humble --workspace
./benchmarks/run_gate.sh --humble --slam --source torwic --seq aisle_cw_run_1
```

Same `orb_slam3_node` and launch files run on both distros; no `#ifdef` forks in application code.
