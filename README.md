# ws_livox — FAST-LIO2 + Livox Avia on ROS 2 Humble (Ubuntu 22.04)

A ready-to-build ROS 2 workspace that runs **FAST-LIO2** (tightly-coupled LiDAR-inertial
odometry & mapping) on a **Livox Avia** solid-state LiDAR, end to end:

```
Livox Avia  ──►  livox_ros2_avia driver  ──►  FAST-LIO2  ──►  odometry + registered cloud + PCD map  ──►  RViz / rosbag2
```

Both packages are forks adapted to work together on **ROS 2 Humble / Ubuntu 22.04**:

| Package | What it is |
|---------|-----------|
| [`livox_ros2_avia`](src/livox_ros2_avia) | Livox Avia ROS 2 driver. Talks to the sensor over Ethernet, builds its own shared **Livox-SDK**, and publishes LiDAR points + IMU. Fork of [ASIG-X/livox_ros2_avia](https://github.com/ASIG-X/livox_ros2_avia) (itself derived from the official Livox driver + the Ubuntu-22.04 Livox-SDK). |
| [`FAST_LIO2-AVIA`](src/FAST_LIO2-AVIA) | The `fast_lio` package: FAST-LIO2 odometry/mapping. Fork of [hku-mars/FAST_LIO](https://github.com/hku-mars/FAST_LIO) rewired to consume **this driver's** custom message type. Bundles the `ikd-Tree` incremental k-d tree (flattened in, no submodule fetch needed). |

> The two are **coupled by message type**: `fast_lio` includes
> `livox_interfaces/msg/custom_msg.hpp` (provided by `livox_ros2_avia`), **not** the
> official `livox_ros_driver2/CustomMsg`. That is the single change that lets stock
> FAST-LIO2 run on this Avia driver. Because of it, `livox_ros2_avia` must build before
> `fast_lio` — `colcon` orders this automatically.

---

## 1. How it works

### Data flow
```
                 /livox/lidar  (livox_interfaces/CustomMsg)
 Livox Avia ──► livox_ros2_avia ───────────────────────────────►  fast_lio (fastlio_mapping)
   (UDP)          driver node    /livox/imu  (sensor_msgs/Imu)         │
                                                                       ├─► /Odometry        (nav_msgs/Odometry)
                                                                       ├─► /cloud_registered(sensor_msgs/PointCloud2, world frame)
                                                                       ├─► /path            (nav_msgs/Path, if path_en)
                                                                       └─► PCD map file (on shutdown, if pcd_save_en)
```

### Roles
- **livox_ros2_avia** — opens the Avia over the network using the broadcast code, runs an
  internal copy of the Livox-SDK, and publishes:
  - `/livox/lidar` — the *custom* Livox point format (per-point time + reflectivity + line).
  - `/livox/imu` — the Avia's built-in IMU (~200 Hz).
  The `livox_lidar_msg_launch.py` launch publishes the **custom** format (`xfer_format=1`),
  which is exactly what FAST-LIO2 wants.
- **fast_lio** — runs an iterated error-state Kalman filter that tightly fuses IMU and
  LiDAR. It maintains the global map in an `ikd-Tree` (fast incremental nearest-neighbour),
  outputs real-time 6-DoF odometry and the registered point cloud, and can dump the whole
  map to a `.pcd` on exit.

### Why these forks (vs. upstream)
- Upstream FAST_LIO2 expects `livox_ros_driver`/`livox_ros_driver2` messages. This fork
  swaps in `livox_interfaces` so it pairs with the Avia driver above.
- The Avia driver bundles a **Ubuntu 22.04-compatible Livox-SDK** built as a *shared*
  library (`livox_sdk_vendor`), avoiding the system static-lib conflict described below.

---

## 2. Prerequisites

- **Ubuntu 22.04** + **ROS 2 Humble** (desktop install, includes RViz2).
- Build tools and the libraries the two packages need:

```bash
sudo apt update
sudo apt install -y \
  ros-humble-desktop \
  python3-colcon-common-extensions \
  ros-humble-pcl-ros ros-humble-pcl-conversions \
  libpcl-dev libeigen3-dev \
  cmake build-essential git
```

- A Livox Avia reachable over Ethernet (static host IP on the Livox subnet, typically
  `192.168.1.x`). Only needed for **live** capture — the bag-replay path below needs no
  hardware.

---

## 3. Build

```bash
# 1. Clone
git clone https://github.com/Amin-Yazdanshenas/ws_livox.git
cd ws_livox

# 2. IMPORTANT — remove any stale system Livox static lib.
#    livox_sdk_vendor builds its OWN shared SDK; a leftover static lib collides.
sudo rm -f /usr/local/lib/livox_sdk_static.a   # (rename instead of delete if you prefer)

# 3. Build the whole workspace
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

Build order is automatic: `livox_interfaces` → `livox_sdk_vendor` / `livox_ros2_avia` →
`fast_lio`. Build once, then `source install/setup.bash` in every terminal.

> No `git submodule update` is needed — `ikd-Tree` is committed directly into
> `src/FAST_LIO2-AVIA/include/ikd-Tree`.

---

## 4. Configure

### Driver — point it at *your* sensor
Edit [`src/livox_ros2_avia/livox_ros2_avia/config/livox_lidar_config.json`](src/livox_ros2_avia/livox_ros2_avia/config/livox_lidar_config.json):
- `broadcast_code` — the 14-character serial under the QR code on the Avia body **plus a
  trailing `1`** (e.g. `3JEDLB100127651`).
- `enable_connect` — set to `true`.

(Skip this entirely if you are only replaying a bag.)

### FAST-LIO2 — Avia parameters
[`src/FAST_LIO2-AVIA/config/avia.yaml`](src/FAST_LIO2-AVIA/config/avia.yaml) is already set
for the Avia. Key fields:

| Field | Value | Meaning |
|-------|-------|---------|
| `lid_topic` / `imu_topic` | `/livox/lidar`, `/livox/imu` | must match the driver's topics |
| `preprocess.lidar_type` | `1` | Livox |
| `preprocess.scan_line` | `6` | Avia has 6 scan lines |
| `preprocess.blind` | `0.5` | ignore returns < 0.5 m |
| `mapping.extrinsic_T` | `[0.04165, 0.02326, -0.0284]` | LiDAR→IMU translation (Avia factory values) |
| `pcd_save.pcd_save_en` | `true` | dump the full map to PCD on exit |
| `map_file_path` | `/home/aminys/fastlio_pcd_maps/avia_fastlio_map.pcd` | **edit to your path**; make sure the folder exists |
| `publish.path_en` | `false` | set `true` if you want `/path` populated |

Create the PCD output folder before mapping:
```bash
mkdir -p ~/fastlio_pcd_maps
```

---

## 5. Run — live mapping

Open a terminal per step. Source both overlays in each:

```bash
source /opt/ros/humble/setup.bash
source ~/ws_livox/install/setup.bash
```

**Terminal A — start the Avia driver** (publishes `/livox/lidar` + `/livox/imu`):
```bash
ros2 launch livox_ros2_avia livox_lidar_msg_launch.py
```

**Terminal B — start FAST-LIO2** (loads `avia.yaml`, opens RViz):
```bash
ros2 launch fast_lio mapping.launch.py config_file:=avia.yaml
```
Move the sensor slowly; the registered cloud and trajectory grow in RViz. The PCD map is
written to `map_file_path` when you stop FAST-LIO2 with `Ctrl-C`.

**Terminal C — (optional) record a bag** of the results:
```bash
ros2 bag record /cloud_registered /Odometry /path
```

### Topics

| Topic | Type | Source |
|-------|------|--------|
| `/livox/lidar` | `livox_interfaces/msg/CustomMsg` | driver |
| `/livox/imu` | `sensor_msgs/msg/Imu` | driver |
| `/Odometry` | `nav_msgs/msg/Odometry` | fast_lio |
| `/cloud_registered` | `sensor_msgs/msg/PointCloud2` | fast_lio |
| `/path` | `nav_msgs/msg/Path` | fast_lio (needs `path_en: true`) |

---

## 6. Run — replay from a rosbag (no hardware)

To re-map from a previously recorded **raw** bag (one that has `/livox/lidar` + `/livox/imu`):

**Terminal A — FAST-LIO2:**
```bash
source /opt/ros/humble/setup.bash
source ~/ws_livox/install/setup.bash
ros2 launch fast_lio mapping.launch.py config_file:=avia.yaml
```

**Terminal B — play the bag** (loop for convenience):
```bash
source /opt/ros/humble/setup.bash
ros2 bag play rosbag2_2026_06_19-18_20_10/ --loop
```

**Terminal C — RViz** (if you launched fast_lio with `rviz:=false`, or want a second view):
```bash
source /opt/ros/humble/setup.bash
rviz2
```
In RViz set **Fixed Frame** to `camera_init` and add the `/cloud_registered`, `/Odometry`,
and `/path` displays.

> A small sample bag is included at
> [`src/livox_ros2_avia/rosbag2_2026_03_16-13_50_55`](src/livox_ros2_avia/rosbag2_2026_03_16-13_50_55).
> Note: a bag recorded from `/cloud_registered + /Odometry` (Terminal C above) is for
> *visualizing finished results*; to **re-run mapping** you need a bag of the raw
> `/livox/lidar + /livox/imu` topics instead.

---

## 7. Troubleshooting

| Symptom | Fix |
|---------|-----|
| Link error mentioning `livox_sdk_static` during build | Remove `/usr/local/lib/livox_sdk_static.a` and rebuild (see step 3). |
| Driver starts but no `/livox/lidar` | Wrong `broadcast_code`, `enable_connect:false`, or host not on the Avia's subnet. Check `ros2 topic hz /livox/lidar`. |
| FAST-LIO2 prints "wait for the IMU" forever | IMU topic mismatch — confirm `/livox/imu` is publishing and `imu_topic` in `avia.yaml` matches. |
| RViz empty | Fixed Frame must be `camera_init`; confirm `scan_publish_en: true`. |
| No PCD written | `pcd_save_en` must be `true`, `map_file_path` folder must exist, and FAST-LIO2 must be stopped with `Ctrl-C` (it writes on shutdown). |
| `package 'fast_lio' not found` | You didn't `source install/setup.bash` in that terminal. |

---

## 8. Credits & licenses

- **FAST-LIO2** — [hku-mars/FAST_LIO](https://github.com/hku-mars/FAST_LIO) (GPLv2). Avia
  adaptation in this fork.
- **ikd-Tree** — [hku-mars/ikd-Tree](https://github.com/hku-mars/ikd-Tree).
- **livox_ros2_avia** — [ASIG-X/livox_ros2_avia](https://github.com/ASIG-X/livox_ros2_avia)
  (MIT), from the official Livox driver and the Ubuntu-22.04 Livox-SDK.

Each package keeps its own `LICENSE`. This workspace is a reproducible integration of the
above for ROS 2 Humble / Ubuntu 22.04.
