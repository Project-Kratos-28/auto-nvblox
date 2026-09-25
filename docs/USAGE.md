# Using nvblox with the ZED 2i and the RealSense D435i

This guide covers everything needed to build a 3D map and a Nav2 costmap with nvblox on the
Jetson AGX Orin, using either stereo camera. It goes from a fresh clone to a running pipeline,
then covers each camera in detail, how to check that it works, tuning, and troubleshooting.

- Quick commands only: [`HOW_TO_RUN.txt`](../HOW_TO_RUN.txt)
- Design notes, measurements and history: [`README.md`](../README.md)

Contents

1. [How the pipeline works](#1-how-the-pipeline-works)
2. [One-time setup](#2-one-time-setup)
3. [Everyday workflow](#3-everyday-workflow)
4. [ZED 2i](#4-zed-2i)
5. [RealSense D435i](#5-realsense-d435i)
6. [Choosing a camera and depth source](#6-choosing-a-camera-and-depth-source)
7. [Checking that it works](#7-checking-that-it-works)
8. [Launch arguments](#8-launch-arguments)
9. [Tuning](#9-tuning)
10. [Remote operation (base station)](#10-remote-operation-base-station)
11. [Using the map in your own Nav2 stack](#11-using-the-map-in-your-own-nav2-stack)
12. [Troubleshooting](#12-troubleshooting)
13. [Reference: files, topics and frames](#13-reference-files-topics-and-frames)

---

## 1. How the pipeline works

Every node runs inside the Isaac ROS Docker container (`isaac_ros_dev_container`, ROS 2 Jazzy,
Isaac ROS 4.6). A single launch file, `kratos_nvblox_bringup/launch/bringup.launch.py`, starts
the whole chain. The `camera:=` argument picks the camera and `depth:=` picks where depth comes from.

```
                 ┌──────────── camera:=zed2i ────────────┐   ┌──────────── camera:=d435i ───────────┐
 sensor          │ ZED 2i, 1280x720 @30 (zed-ros2-wrapper)│   │ D435i, 848x480, projector flashing @60│
                 │                                        │   │   └─ realsense_splitter               │
 odometry (TF)   │ ZED visual-inertial tracking           │   │ cuVSLAM on the projector-off IR pair  │
                 │   odom -> zed_camera_link              │   │   odom -> camera0_link                │
 depth:=ess      │ ESS DNN on the gray rectified pair     │   │ ESS DNN on the projector-off IR pair  │
   (default)     │   -> /ess/depth                        │   │   -> /ess/depth                       │
 depth:=native   │ ZED SDK NEURAL depth                   │   │ D435i on-chip depth (projector on)    │
                 └────────────────────────────────────────┘   └───────────────────────────────────────┘
                                          │  depth + color + TF
                                          ▼
                 nvblox (TSDF, 2 cm voxels) ──► /nvblox_node/static_map_slice (2D ESDF slice)
                                          ▼
                 Nav2 Costmap2DROS + NvbloxCostmapLayer ──► /local_costmap/costmap
                                          ▼
                 throttled /remote/* copies ──► Zenoh bridge ──► base station (optional)
```

The pieces:

| Stage | ZED 2i | D435i |
|---|---|---|
| Driver | `zed_wrapper` (ZED SDK 5.4.1), its own container | `realsense2_camera`, loaded into `nvblox_container` |
| Odometry | ZED positional tracking with IMU fusion | Isaac ROS cuVSLAM (`/visual_slam/tracking/odometry`) |
| Robot frame | `zed_camera_link` | `camera0_link` |
| Stereo pair for ESS | `/zed/zed_node/{left,right}/gray/rect/image` | `/camera0/realsense_splitter_node/output/infra_{1,2}` |
| Native depth | `/zed/zed_node/depth/depth_registered` (32FC1, m) | `/camera0/realsense_splitter_node/output/depth` |
| Color for nvblox | `/zed/zed_node/rgb/color/rect/image` | `/camera0/color/image_raw` |
| nvblox camera preset | NVIDIA's `nvblox_zed.yaml` | NVIDIA's `nvblox_realsense.yaml` |

**ESS** is NVIDIA's DNN stereo disparity model (`dnn_stereo_disparity` v4.1.0) running on
TensorRT. It is not an Isaac ROS package here: it's a standalone Python node,
`ess/ess_stereo_node.py`, run from its own venv (`ess/.venv`, torch + TensorRT). Whatever the
camera, it publishes `/ess/depth` (16UC1, millimetres, in the left camera frame) and
`/ess/camera_info`.

---

## 2. One-time setup

Skip this section on the Orin that already runs the stack. It describes how to rebuild the
setup on a new machine or after a fresh clone.

### 2.1 Host prerequisites

- Jetson AGX Orin with JetPack 7.2 (L4T R39.2). The ZED SDK build in the Dockerfile and the
  torch wheel (CUDA 13.2, Python 3.12) are tied to this release.
- Docker, with `nvidia` as the default runtime in `/etc/docker/daemon.json`.
- The Isaac ROS apt repository (`release-4.6`) and `isaac-ros-cli`, set to docker mode with
  the RealSense image layer. `~/.config/isaac-ros-cli/config.yaml`:

  ```yaml
  docker:
    image:
      additional_image_keys:
        - realsense
  ```

- `git-lfs`, which the torch wheel in `ess/wheels/` needs.

### 2.2 Clone

The scripts expect the repository at `~/kratos_nvblox`:

```bash
sudo apt install git-lfs && git lfs install
git clone https://github.com/Project-Kratos-28/auto-nvblox.git ~/kratos_nvblox
ls -lh ~/kratos_nvblox/ess/wheels/     # must be ~240 MB; a ~130 byte file means LFS did not pull
```

If the wheel is only a small pointer file, run `git lfs pull` inside the repo.

### 2.3 Mount the project into the container

`isaac-ros activate` reads extra `docker run` arguments from `~/.isaac_ros_dev-dockerargs`.
Create it with (replace `/home/bitsauto` with your home directory):

```
-v /home/bitsauto/kratos_nvblox:/workspaces/kratos_nvblox
-v /home/bitsauto/kratos_nvblox/zed/resources:/usr/local/zed/resources
-v /home/bitsauto/kratos_nvblox/zed/settings:/usr/local/zed/settings
```

The first line makes the project visible at `/workspaces/kratos_nvblox`. The other two keep
the ZED SDK's optimized neural models and the camera calibration between containers (see
[4.4](#44-first-run-model-optimization)).

### 2.4 ZED udev rule (ZED 2i only)

```bash
sudo cp ~/kratos_nvblox/docker/99-slabs.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

This makes the ZED's HID sensor interface (IMU, serial number) readable. Without it, the SDK
fails with `sensors returned an invalid serial number`. The D435i needs no host rule: the
RealSense image handles it.

### 2.5 ROS workspace

The container mounts `~/workspaces/isaac_ros-dev` as its workspace. It needs NVIDIA's nvblox
source, but only to build `realsense_splitter`, which is not shipped as a Debian package:

```bash
mkdir -p ~/workspaces/isaac_ros-dev/src && cd ~/workspaces/isaac_ros-dev/src
git clone -b release-4.6 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox.git
```

### 2.6 Build the Docker image

```bash
isaac-ros activate --build-local --build-only    # NVIDIA's Isaac ROS + realsense base image
~/kratos_nvblox/docker/build_image.sh             # kratos image on top (30+ min)
```

`build_image.sh` builds `docker/Dockerfile.kratos` and tags the result as
`cached_isaac_run_dev_image_local:latest`, the image that `start.sh` runs. The image adds:

- nvblox, cuVSLAM, the Nav2 costmap packages, RViz
- ZED SDK 5.4.1 (L4T 39.2 build) and zed-ros2-wrapper v5.4.1 in `/opt/zed_ws`
- `cloudini_ros` (point cloud compression for the remote link)

Only rebuild after changing the Dockerfile.

### 2.7 First container start

```bash
~/kratos_nvblox/start.sh
```

Then, **inside the container**:

```bash
# build realsense_splitter + kratos_nvblox_bringup into /workspaces/isaac_ros-dev/install
/workspaces/kratos_nvblox/scripts/build_ws.sh

# ESS venv (torch from the LFS wheel; it also sees the system's ROS + TensorRT Python packages)
ESS=/workspaces/kratos_nvblox/ess
python3 -m venv --system-site-packages $ESS/.venv
$ESS/.venv/bin/pip install $ESS/wheels/torch-*.whl "numpy==1.26.4"
```

The workspace and the venv are both on mounted host folders, so they survive the container's
`--rm`: you do this once, not every time. (`scripts/setup_in_container.sh` does the same thing
and also apt-installs the packages, which is only needed when running NVIDIA's plain image
instead of the kratos image.)

**TensorRT engines.** `ess/engines/*.engine` are TensorRT 10.16 plans built for the Orin's GPU.
They load only with the same TensorRT version. If the image's TensorRT changes, rebuild them
from the ONNX in NVIDIA's `dnn_stereo_disparity` v4.1.0 package, with `ess/plugins/ess_plugins.so`
loaded. The originals were built in `~/realsense/ess`.

---

## 3. Everyday workflow

```bash
# 1. host: start the container, or open another shell in the running one
~/kratos_nvblox/start.sh

# 2. container: run with the camera that is plugged in
/workspaces/kratos_nvblox/scripts/run_nvblox.sh camera:=zed2i
/workspaces/kratos_nvblox/scripts/run_nvblox.sh camera:=d435i

# 3. stop: Ctrl+C, then clean up (host or container)
~/kratos_nvblox/scripts/stop_nvblox.sh
```

`run_nvblox.sh` sources ROS, the ZED workspace and the Isaac ROS workspace, sets
`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`, and runs
`ros2 launch kratos_nvblox_bringup bringup.launch.py <your args>`.

What to know:

- **Always stop with `stop_nvblox.sh`**, never `pkill -f`. The container runs with
  `--pid=host`, so `pkill -f` patterns also match unrelated host processes. The script also
  clears stale Fast DDS shared-memory locks (they crash the next run otherwise) and reattaches
  the ZED's HID interface.
- **It's headless by default** (`run_rviz:=False`). For RViz see [7.3](#73-rviz).
- **The first launch in a new container takes about 2 minutes** while CUDA kernels compile.
  Later launches reuse the cache.
- **The container only sees USB devices that existed when it started.** To switch cameras, see
  [6.3](#63-switching-cameras).
- **Changing settings:** edits to existing files in `kratos_nvblox_bringup/` take effect on the
  next run, because the build uses symlinks. After *adding* a file, rerun `scripts/build_ws.sh`.

---

## 4. ZED 2i

### 4.1 Hardware

- Plug it into a **USB 3** port on the Orin; USB 2 can't carry 1280x720 stereo.
- Install the udev rule ([2.4](#24-zed-udev-rule-zed-2i-only)) and start the container *after*
  plugging the camera in.
- Only one program can open the ZED at a time. The `~/kratos_glim` stack also uses it: stop one
  before starting the other.

### 4.2 Run

```bash
/workspaces/kratos_nvblox/scripts/run_nvblox.sh camera:=zed2i                  # ESS depth (default)
/workspaces/kratos_nvblox/scripts/run_nvblox.sh camera:=zed2i depth:=native    # ZED SDK NEURAL depth
```

### 4.3 What runs

- **zed-ros2-wrapper** (`zed_camera.launch.py`, `camera_model:=zed2i`, `camera_name:=zed`),
  with the overrides in `kratos_nvblox_bringup/config/zed2i.yaml`:
  - grabs HD720 at 60 fps and publishes at native resolution (1280x720) at 30 Hz. NVIDIA's
    nvblox example halves the resolution to 640x360; this setup keeps full resolution.
  - publishes the RGB image and the left/right gray rectified pair, which ESS uses.
  - positional tracking with IMU fusion, publishing TF `odom -> zed_camera_link`.
  - `publish_map_tf: false`: nvblox and the costmap work in `odom`. The ZED's `map -> odom`
    loop-closure jumps would tear the map.
  - SDK depth in 32FC1 metres, 0.3–10 m.
- **Depth mode, set by the launch file:**
  - `depth:=native` → `NEURAL`. nvblox integrates `/zed/zed_node/depth/depth_registered`.
  - `depth:=ess` → `NEURAL_LIGHT`. ZED tracking needs the SDK's depth running, so the lighter
    model runs just for that. nvblox integrates `/ess/depth` instead.
- **ESS** (`depth:=ess`), configured by `config/ess_zed2i.yaml`:
  - 960x576 engine. Depth is resampled back to 1280x720.
  - baseline (12 cm) read from the right camera_info.
  - range 0.3–10 m. The ZED's wide baseline keeps disparity error low further out than on the D435i.
  - `tta: false`. Measured on the ZED 2i: 27–28 Hz with 88 % fill, versus 15 Hz and 74 % with TTA.
- **nvblox** with `nvblox_base.yaml` + `nvblox_zed.yaml` + `config/nvblox_overrides.yaml`.
- **Costmap:** `robot_base_frame: zed_camera_link`.

### 4.4 First-run model optimization

The ZED SDK optimizes each neural depth model for the GPU the first time it is used
(NEURAL_LIGHT takes about 7 minutes, NEURAL about 4.5 minutes). The log shows
`Optimizing model: ... Progress: NN%`. It also downloads the camera's factory calibration.

Both are saved in `zed/resources` and `zed/settings` (mounted over `/usr/local/zed/...`), so
this only happens again after an SDK or GPU change. The costmap waits up to 900 s for odometry
(`initial_transform_timeout` in `nvblox_costmap.yaml`), so a first run still comes up on its own.

`zed/resources` holds a model file for each depth mode. Switching between `depth:=ess` and
`depth:=native` for the first time optimizes the other model.

### 4.5 Expected performance

Measured with 2 cm voxels, MAXN power mode, headless:

| | ZED 2i + ESS | ZED 2i + NEURAL |
|---|---|---|
| depth | `/ess/depth` 1280x720, ~12.6 Hz, ~85 % fill | `depth_registered` 1280x720, ~22 Hz |
| odometry (`/zed/zed_node/odom`) | ~50 Hz | ~57 Hz |
| `/nvblox_node/static_map_slice` | ~9.7–12.5 Hz | ~12 Hz |
| `/local_costmap/costmap` | ~11.4 Hz | ~12.3 Hz |

### 4.6 ZED-specific problems

| Symptom | Cause and fix |
|---|---|
| `sensors returned an invalid serial number`, camera won't open | The HID sensor interface lost its driver. The ZED SDK detaches `usbhid` and doesn't reattach it on exit, so `/dev/hidraw*` disappears. Run `scripts/stop_nvblox.sh` (it calls `scripts/zed_hid_rebind.sh`), then restart the container. Also check the udev rule from 2.4. |
| Several minutes of `Optimizing model` | Normal on the first run of a depth mode ([4.4](#44-first-run-model-optimization)). |
| CUDA error 801 in the ZED node or ESS | The process started before the container finished adding `admin` to the `render` group. Use `start.sh` / `rover_up.sh`, which wait for it, and restart the pipeline. |
| Camera not found after re-plugging | Restart the container: it copies device permissions only when it starts. |

---

## 5. RealSense D435i

### 5.1 Hardware

- Plug it into a **USB 3** port. The profiles need USB 3 bandwidth: 848x480 depth and IR at
  60 fps, plus color and IMU.
- Start the container after plugging it in.

### 5.2 Run

```bash
/workspaces/kratos_nvblox/scripts/run_nvblox.sh camera:=d435i                  # ESS depth (default)
/workspaces/kratos_nvblox/scripts/run_nvblox.sh camera:=d435i depth:=native    # on-chip depth
```

`camera:=d435i` is the launch file's default, so it can be left out.

### 5.3 What runs

- **realsense2_camera**, loaded into `nvblox_container`, with `config/d435i.yaml`. This is
  NVIDIA's `realsense_emitter_flashing.yaml` with the resolution raised to the D435i's native
  848x480:
  - depth and both IR streams at 848x480 and **60 fps**, with `emitter_on_off: true`, so the IR
    dot projector is on in every other frame.
  - color at 848x480 @30, gyro and accel at 200 Hz.
- **realsense_splitter** splits the 60 fps stream in two:
  - **projector-off** IR frames (30 fps) go to cuVSLAM and ESS. Both need images without the
    dot pattern: cuVSLAM would track the dots, which move with the camera.
  - **projector-on** depth frames (30 fps) are the `depth:=native` source. The on-chip stereo
    needs the dots on textureless surfaces.
- **cuVSLAM** (NVIDIA's `vslam.launch.py`, loaded into the same container) publishes TF
  `odom -> camera0_link` and `/visual_slam/tracking/odometry`.
- **ESS** (`depth:=ess`), configured by `config/ess_d435i.yaml`:
  - 960x576 engine on the projector-off IR pair, `input_norm: stretch` (the IR images are dim).
  - `tta: true`, a second pass on the flipped pair that keeps only pixels where both passes
    agree. The D435i's dim IR needs this filtering.
  - range 0.2–6 m: with the 5 cm baseline, disparity error grows quickly beyond that.
  - baseline read from the infra2 camera_info.
- **nvblox** with `nvblox_base.yaml` + `nvblox_realsense.yaml` + `config/nvblox_overrides.yaml`.
- **Costmap:** `robot_base_frame: camera0_link`.

### 5.4 Expected performance

Measured with 2 cm voxels, MAXN power mode, headless:

| | D435i + ESS |
|---|---|
| depth | `/ess/depth` 848x480, ~18 Hz |
| odometry (cuVSLAM) | ~24–30 Hz |
| `/nvblox_node/static_map_slice` | ~9–14 Hz |

### 5.5 D435i-specific problems

| Symptom | Cause and fix |
|---|---|
| cuVSLAM loses tracking / odometry jumps | Keep the IR cameras' view textured and well lit, and avoid fast rotations. cuVSLAM only sees the projector-off IR frames, so it relies on natural texture. |
| Few depth pixels on blank walls with `depth:=ess` | ESS sees the projector-off pair and needs natural texture. Try `depth:=native`, which uses the projected dots. |
| No `/camera0/...` topics | Camera not visible in the container: check `rs-enumerate-devices` inside it, re-plug, restart the container. Make sure it's on USB 3. |

---

## 6. Choosing a camera and depth source

### 6.1 Camera

| | ZED 2i | D435i |
|---|---|---|
| Stereo baseline | 12 cm, good depth out to ~10 m | 5 cm, trusted to ~6 m |
| Resolution into nvblox | 1280x720 | 848x480 |
| Odometry | ZED visual-inertial (~50 Hz) | cuVSLAM (~24–30 Hz) |
| Active IR projector | no | yes, helps native depth on blank surfaces |
| Extra first-run cost | model optimization, minutes | none |

The ZED 2i is the usual choice on the rover (range, odometry rate). The D435i works better for
close range indoors on untextured surfaces with `depth:=native`.

### 6.2 Depth source

| | `depth:=ess` (default) | `depth:=native` |
|---|---|---|
| ZED 2i | ESS on the gray pair, sharp edges, ~12.6 Hz into nvblox | SDK NEURAL, ~22 Hz, smoother |
| D435i | ESS on projector-off IR, filtered with TTA | on-chip depth with projected dots |
| GPU | ESS uses the GPU heavily | ZED NEURAL uses the GPU; D435i on-chip uses none |

### 6.3 Switching cameras

1. Stop the pipeline: Ctrl+C, then `~/kratos_nvblox/scripts/stop_nvblox.sh`.
2. Unplug one camera and plug in the other (USB 3).
3. Restart the container: `exit` its shell (it is `--rm`), then run `~/kratos_nvblox/start.sh`.
4. Run with the other `camera:=` value.

---

## 7. Checking that it works

### 7.1 Log milestones

In the launch output (or `logs/latest.log` when started with `rover_up.sh`):

1. `kratos_nvblox_bringup: camera=... depth=... -> nvblox depth <topic>`: arguments parsed.
2. Camera driver up (ZED: `Camera successfully opened`; D435i: `RealSense Node Is Up!`).
3. ESS prints its latency periodically (`latency ms: ...`).
4. `Managed nodes are active`: the costmap has odometry TF and is publishing.

### 7.2 Topic rates

From a second shell in the container:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
/workspaces/kratos_nvblox/scripts/topic_rates.py \
    /ess/depth /zed/zed_node/odom /nvblox_node/static_map_slice /local_costmap/costmap
```

(For the D435i use `/visual_slam/tracking/odometry` instead of `/zed/zed_node/odom`.)

Use `topic_rates.py` rather than `ros2 topic hz`. `ros2 topic hz` deserializes every message
in Python and badly undercounts large images; for example, it reads 1280x720 depth at ~5 Hz
when ~11 Hz is published.

`~/kratos_nvblox/rover_status.sh` (host) gives a one-shot health summary: rates, ESS latency,
GPU load, RAM, temperature, log errors and the Zenoh link.

### 7.3 RViz

Either start it with the pipeline:

```bash
/workspaces/kratos_nvblox/scripts/run_nvblox.sh camera:=zed2i run_rviz:=True
```

or attach it to a pipeline that's already running (host terminal):

```bash
xhost +local:
docker exec -it -u admin -e DISPLAY=$DISPLAY -e CAMERA=zed2i isaac_ros_dev_container \
    /workspaces/kratos_nvblox/scripts/run_rviz.sh          # CAMERA=d435i for the RealSense layout
```

The configs (`kratos_nvblox_bringup/rviz/{zed2i,d435i}.rviz`) show the nvblox mesh, color
layer, ESDF slice point cloud, costmap, depth image and camera image. The depth panel is
remapped to whatever nvblox is integrating, so it also works with `depth:=native`.

RViz on the Orin takes GPU time from ESS and nvblox; expect slightly lower rates while it runs.

### 7.4 From a host terminal

The container uses `--network host`, so host ROS tools see the topics once discovery matches:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ros2 topic list
```

---

## 8. Launch arguments

Pass them to `run_nvblox.sh` or `rover_up.sh`. The full list is always available with
`ros2 launch kratos_nvblox_bringup bringup.launch.py --show-args`.

| Argument | Default | Meaning |
|---|---|---|
| `camera` | `d435i` | `d435i` or `zed2i` |
| `depth` | `ess` | `ess` (ESS DNN) or `native` (ZED NEURAL / D435i on-chip) |
| `run_rviz` | `False` | RViz on the Orin's display |
| `run_costmap` | `True` | standalone Nav2 costmap; set `False` when a full Nav2 stack provides one |
| `costmap_params` | `config/nvblox_costmap.yaml` | costmap parameter file |
| `ess_params` | `config/ess_<camera>.yaml` | ESS parameter file (e.g. `bench/ess_zed2i_notta.yaml`) |
| `ess_root` | `/workspaces/kratos_nvblox/ess` | ESS node, engines and `.venv` |
| `rviz_config` | `rviz/<camera>.rviz` | RViz config |
| `run_remote` | `True` | throttled `/remote/*` topics for the base station |
| `remote_costmap_hz` | `2.0` | rate of `/remote/costmap` |
| `remote_image_hz` | `5.0` | rate of `/remote/color/compressed` |
| `remote_esdf_hz` | `4.0` | rate of `/remote/esdf_slice[/cloudini]` |
| `remote_jpeg_quality` | `60` | JPEG quality of the ZED compressed color stream |
| `run_zenoh` | `True` | Zenoh bridge to the base station |
| `zenoh_root` | `/workspaces/kratos_nvblox/zenoh` | bridge binary and `rover.json5` |
| `run_foxglove` | `False` | Foxglove websocket viewer |
| `foxglove_port` | `8765` | Foxglove port |
| `stream_bandwidth_mbps` | `5.0` | cap on nvblox's mesh visualization stream |

---

## 9. Tuning

The GPU is the bottleneck: ESS, nvblox, the ZED SDK and RViz all share it. Run in MAXN power
mode (`sudo nvpmodel -m 0 && sudo jetson_clocks`).

| File (`kratos_nvblox_bringup/config/`) | Setting | Effect |
|---|---|---|
| `ess_<camera>.yaml` | `tta` | second ESS pass on the flipped pair. Stricter depth, roughly half the rate. ZED off, D435i on. |
| `ess_<camera>.yaml` | `engine_path: .../light_ess_fp16.engine` | 480x288 network, ~3 ms, coarser depth |
| `ess_<camera>.yaml` | `min_depth_m`, `max_depth_m` | range passed to nvblox |
| `ess_<camera>.yaml` | `confidence_threshold`, `edge_disparity_px` | drop low-confidence pixels / flying pixels at depth edges |
| `nvblox_overrides.yaml` | `voxel_size` (0.02) | 0.03 is about 3x less nvblox work; 0.05 (NVIDIA default) is blocky |
| `nvblox_overrides.yaml` | `projective_integrator_max_integration_distance_m` (5.0) | how far depth is integrated |
| `nvblox_overrides.yaml` | `update_esdf_rate_hz` (15) | rate of the 2D slice, which drives the costmap rate |
| `nvblox_costmap.yaml` | `width`/`height` (10 m), `resolution` (0.02) | costmap window; keep `resolution` = `voxel_size` |
| `nvblox_costmap.yaml` | `inflation_radius` (0.5), `footprint` | robot size and clearance |
| `nvblox_costmap.yaml` | `update_frequency`/`publish_frequency` (20/20) | Nav2 only publishes on update ticks; 20/20 lets the costmap track the ~12 Hz slice |
| `zed2i.yaml` | `grab_resolution`, `pub_frame_rate` | HD1080 costs about 2x GPU and grabs at most 30 fps |
| `d435i.yaml` | `*_profile` | D435i resolutions and rates |

**Slice height.** The costmap is built from the ESDF between `esdf_slice_min_height` (-0.1 m)
and `esdf_slice_max_height` (0.3 m), relative to the robot frame. With no robot, that frame is
the camera itself. On a robot, set these for the camera's mounting height, so the floor is
excluded and every obstacle the robot could hit is included.

---

## 10. Remote operation (base station)

On the rover, over SSH. Everything is started detached, so the pipeline keeps running if the
link drops:

```bash
ssh bitsauto@<rover-ip> ~/kratos_nvblox/rover_up.sh camera:=zed2i     # or camera:=d435i
ssh bitsauto@<rover-ip> ~/kratos_nvblox/rover_status.sh
ssh bitsauto@<rover-ip> ~/kratos_nvblox/rover_down.sh                 # --all also stops the container
```

`rover_up.sh` starts the container if needed, starts the pipeline in the background (log:
`logs/latest.log`), waits for the costmap and prints a status summary.

ROS 2 DDS stays on the Orin. Only the Zenoh bridge (`zenoh/rover.json5`, tcp/7447) uses the
radio, and it forwards only this allow-list:

| Topic | Default rate |
|---|---|
| `/remote/costmap` | 2 Hz |
| `/remote/color/compressed` | 5 Hz, JPEG q60 |
| `/remote/esdf_slice/cloudini` | 4 Hz, Cloudini-compressed (~620 KB → ~30 KB) |
| `/tf`, `/tf_static`, odometry | capped at 30 / 20 Hz |

With everything subscribed, this measured about 2.9 Mbit/s.

On the base station (zenoh-bridge-ros2dds 1.10.1; copy `zenoh/`):

```bash
zenoh-bridge-ros2dds -c base_station.json5 -e tcp/<rover-ip>:7447
export ROS_DOMAIN_ID=42        # in the GUI's environment; keep the default discovery range
ros2 run cloudini_ros cloudini_topic_converter --ros-args \
    -p topic_input:=/remote/esdf_slice/cloudini -p topic_output:=/esdf_slice -p compressing:=false
```

To let the base station send data to the rover (e.g. `/goal_pose`), add the topic under
`subscribers` in `zenoh/rover.json5`.

---

## 11. Using the map in your own Nav2 stack

1. Launch with `run_costmap:=False` so the standalone costmap doesn't collide with yours.
2. Copy the `nvblox_layer` block from `config/nvblox_costmap.yaml` into your `local_costmap`
   params (`nvblox_map_slice_topic: /nvblox_node/static_map_slice`). For a `global_costmap`, use
   `/nvblox_node/combined_map_slice`.
3. Set `nav2_costmap_global_frame` to that costmap's `global_frame` (`odom` here).
4. Publish a static transform from `base_link` to the camera frame (`zed_camera_link` or
   `camera0_link`) and set `robot_base_frame: base_link`.
5. Adjust the slice heights ([9](#9-tuning)) for the mounting height.

The `nvblox::nav2::NvbloxCostmapLayer` plugin is installed only in the container, so run Nav2
there (`nav2_bringup` is installed) or install `ros-jazzy-nvblox-nav2` on the host.

---

## 12. Troubleshooting

| Symptom | Cause and fix |
|---|---|
| RViz shows "No tf data" / blank images, but the pipeline runs | Discovery mismatch. Every terminal needs `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`; the scripts set it. With subnet discovery, a Wi-Fi drop leaves running nodes undiscoverable. |
| Costmap never activates (`Managed nodes are active` missing) | No odometry TF yet. ZED: model optimization may still be running (up to 15 min). D435i: check that cuVSLAM is tracking (`/visual_slam/tracking/odometry`). |
| A new run crashes with a glibc robust-mutex assertion | Stale Fast DDS shared-memory locks from a force-killed run. Run `stop_nvblox.sh` (it runs `fastdds shm clean`). |
| `ess_stereo` dies with `No module named torch` | The ESS venv is missing. Create it ([2.7](#27-first-container-start)). |
| ESS: `Could not load TensorRT engine` | The engine doesn't match the TensorRT version. Rebuild it ([2.7](#27-first-container-start)). |
| `Package 'kratos_nvblox_bringup' not found` | Workspace not built: `scripts/build_ws.sh` in the container. |
| Rates far below the tables | Not in MAXN; RViz running on the Orin; `tta: true` on the ZED; first launch still compiling CUDA kernels. |
| Map smears / ghosts when moving | Odometry drift or jumps. Keep `publish_map_tf: false` on the ZED. On the D435i, check cuVSLAM tracking. Lower `max_depth_m` to drop noisy far depth. |
| `kratos image not built yet` from `start.sh` | Build it ([2.6](#26-build-the-docker-image)). Without it, NVIDIA's base image lacks nvblox and the ZED SDK. |

ESS and the Zenoh bridge respawn automatically if they exit. Other nodes don't.

---

## 13. Reference: files, topics and frames

### Files

| Path | What |
|---|---|
| `start.sh` | host: start or attach to the container |
| `scripts/run_nvblox.sh` | container: run the pipeline |
| `scripts/stop_nvblox.sh` | host/container: stop and clean up |
| `scripts/run_rviz.sh` | container: RViz for a running pipeline |
| `scripts/build_ws.sh` | container: build `realsense_splitter` + `kratos_nvblox_bringup` |
| `scripts/setup_in_container.sh` | container: full setup on NVIDIA's plain image |
| `scripts/topic_rates.py` | rate probe that doesn't undercount large images |
| `scripts/zed_hid_rebind.sh` | host: reattach the ZED's HID interface |
| `rover_up.sh`, `rover_status.sh`, `rover_down.sh` | remote bringup over SSH |
| `kratos_nvblox_bringup/launch/bringup.launch.py` | the whole pipeline |
| `kratos_nvblox_bringup/config/` | camera, ESS, nvblox and costmap parameters |
| `kratos_nvblox_bringup/rviz/` | RViz layouts per camera |
| `ess/` | ESS node, TensorRT engines, plugin, torch wheel (LFS), `.venv` (not in git) |
| `zed/` | persisted ZED models and calibration |
| `zenoh/` | Zenoh bridge binary and configs |
| `docker/` | Dockerfile, image build script, ZED udev rule |
| `bench/`, `bench_zed/` | alternative ESS params, offline ZED depth benchmark |

### Main topics

| Topic | Type | Notes |
|---|---|---|
| `/ess/depth` | Image 16UC1 (mm) | `depth:=ess` |
| `/ess/camera_info` | CameraInfo | |
| `/ess/disparity`, `/ess/confidence` | Image | ESS debugging |
| `/ess/latency_ms` | Float32 | |
| `/zed/zed_node/depth/depth_registered` | Image 32FC1 (m) | ZED `depth:=native` |
| `/camera0/realsense_splitter_node/output/depth` | Image | D435i `depth:=native` |
| `/zed/zed_node/odom` | Odometry | ZED |
| `/visual_slam/tracking/odometry` | Odometry | D435i (cuVSLAM) |
| `/nvblox_node/mesh` | nvblox mesh | visualization |
| `/nvblox_node/static_esdf_pointcloud` | PointCloud2 | ESDF slice as a cloud |
| `/nvblox_node/static_map_slice` | nvblox DistanceMapSlice | input of the costmap layer |
| `/local_costmap/costmap` | OccupancyGrid | 2 cm cells, 10x10 m rolling window |
| `/remote/*` | | throttled copies for the base station |

### Frames

```
odom ──► zed_camera_link ──► (ZED camera frames, from the wrapper's robot_state_publisher)   camera:=zed2i
odom ──► camera0_link    ──► (RealSense frames, from realsense2_camera)                      camera:=d435i
```

`odom` is the fixed frame for nvblox and the costmap. There is no `map` frame and no
`base_link` until you add one ([11](#11-using-the-map-in-your-own-nav2-stack)).
