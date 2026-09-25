# kratos_nvblox

nvblox on the AGX Orin (JetPack 7.2 / R39.2) in the Isaac ROS 4.6 container
(ROS 2 Jazzy), fed by a RealSense D435i or a ZED 2i, producing a Nav2 costmap.

Pipeline (all in the container):

```
camera:=d435i (default)
  D435i 848x480, projector flashing @60 -> realsense_splitter
     projector-off IR pair @30 -> cuVSLAM -> TF odom -> camera0_link
                               -> ESS DNN stereo -> /ess/depth            (depth:=ess, default)
     projector-on depth @30                                              (depth:=native)
camera:=zed2i
  ZED 2i 1280x720 @30, ZED visual-inertial tracking -> TF odom -> zed_camera_link
     gray rectified pair -> ESS DNN stereo -> /ess/depth                 (depth:=ess, default)
     ZED SDK NEURAL depth -> /zed/zed_node/depth/depth_registered        (depth:=native)
then
  depth + color + TF -> nvblox (2 cm voxels) -> NvbloxCostmapLayer -> /local_costmap/costmap
```

ESS is the node and TensorRT engines from `~/realsense` rs_depth_bench, copied to `ess/`.

**Full guide (setup from a fresh clone, each camera in detail, tuning, troubleshooting):
[`docs/USAGE.md`](docs/USAGE.md).**

## Run

```bash
# host: start or attach to the container (image is already built)
~/kratos_nvblox/start.sh

# inside the container
/workspaces/kratos_nvblox/scripts/run_nvblox.sh camera:=d435i                # D435i + ESS (headless)
/workspaces/kratos_nvblox/scripts/run_nvblox.sh camera:=zed2i                # ZED 2i + ESS
/workspaces/kratos_nvblox/scripts/run_nvblox.sh camera:=zed2i depth:=native  # ZED 2i + ZED NEURAL depth
/workspaces/kratos_nvblox/scripts/run_nvblox.sh camera:=d435i depth:=native  # D435i on-chip depth
/workspaces/kratos_nvblox/scripts/run_nvblox.sh camera:=zed2i run_rviz:=True   # RViz on the rover display
# run_nvblox.sh just sources the workspaces and runs
#   ros2 launch kratos_nvblox_bringup bringup.launch.py <args>
# `ros2 launch kratos_nvblox_bringup bringup.launch.py --show-args` lists every argument.

# after editing kratos_nvblox_bringup/ (new files; edits to existing ones are symlinked)
/workspaces/kratos_nvblox/scripts/build_ws.sh

# stop (host or container). Use this rather than pkill -f: the container runs with
# --pid=host, so pkill -f patterns also hit host processes.
~/kratos_nvblox/scripts/stop_nvblox.sh

# RViz separately, for a pipeline started with run_rviz:=False (CAMERA picks the config)
xhost +local:                                                     # host, once per login
docker exec -it -u admin -e DISPLAY=$DISPLAY -e CAMERA=zed2i isaac_ros_dev_container \
    /workspaces/kratos_nvblox/scripts/run_rviz.sh
```

The scripts set `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`. With the default
subnet discovery, a Wi-Fi drop/reconnect leaves already-running nodes
undiscoverable, so RViz shows "No tf data" and blank images even though
the pipeline is still running. Export the same variable in any host terminal
that should see these topics.

The container uses `--network host`, so the host's Jazzy tools see every topic
(with `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` exported, see above).
The first launch after the container is created is slow (~2 min) while CUDA
kernels compile. Later launches reuse the cache.

## Remote operation (Rocket 5AC link to the base station)

Use `rover_up.sh` / `rover_status.sh` / `rover_down.sh` over SSH (see HOW_TO_RUN.txt). They run the
container and the pipeline detached from the SSH session (`setsid`, `docker exec -d`), so link
dropouts do not stop the rover.

- ROS 2 DDS stays on the Orin (LOCALHOST discovery). Only the Zenoh bridge
  (`zenoh/zenoh-bridge-ros2dds` 1.10.1, `zenoh/rover.json5`, tcp/7447) uses the radio, and it
  forwards only an allow-list: `/remote/costmap`, `/remote/color/compressed`,
  `/remote/esdf_slice/cloudini`, TF, odometry, footprint, ESS latency (rate-capped).
- `/remote/*` are throttled copies made by `throttle_relay.py` (republishes serialized messages,
  no decoding); the ESDF slice is compressed with Cloudini (5 mm resolution, 622 KB -> ~30 KB).
  The ZED's JPEG stream is set to quality 60 (95 default: 262 KB/frame; 60: 65 KB).
- Measured over the bridge with every topic subscribed: ~2.9 Mbit/s. For comparison, the first
  Foxglove setup (unthrottled ESDF cloud, JPEG 95) needed ~66 Mbit/s.
- Base station: `zenoh/base_station.json5`, ROS_DOMAIN_ID 42 with default (SUBNET) discovery.
  Tested on the Orin with a second bridge: a Fast DDS subscriber with LOCALHOST discovery
  receives nothing from the bridge's CycloneDDS writers; SUBNET works.
- Headless by default (`run_rviz:=False`); RViz on the Orin also costs GPU that ESS/nvblox use.
- ESS and the Zenoh bridge respawn if they exit.

## Image

`start.sh` runs `kratos/isaac_ros_nvblox:latest`, built by `docker/build_image.sh` from
`docker/Dockerfile.kratos` on top of NVIDIA's Isaac ROS 4.6 + realsense image. It already
contains nvblox, cuVSLAM, the Nav2 costmap packages and the ZED SDK 5.4.1 (L4T 39.2 build)
with zed-ros2-wrapper v5.4.1, so a fresh `--rm` container needs no setup.
(`scripts/setup_in_container.sh` is only needed with NVIDIA's plain image. After a fresh
clone of the workspace, run `scripts/build_ws.sh` once.)

- NVIDIA's own ZED layer installs the SDK built for L4T 38.4 (JetPack 7.0) and turns the SDK's
  depth off, so the Dockerfile installs the SDK itself.
- The wrapper's optional NITROS transport is disabled at build time: it does not compile
  against the NITROS packages that come with nvblox.

## ZED 2i notes

- Host udev rule `/etc/udev/rules.d/99-slabs.rules` (copy in `docker/99-slabs.rules`, from the
  SDK minus its ZED-GMSL i2c line) makes the camera's HID sensor interface readable. Without
  it the SDK fails with "sensors returned an invalid serial number". A container copies device
  permissions when it starts, so restart it after changing udev rules or re-plugging a camera.
- The SDK optimizes each neural depth model for this GPU on first use (NEURAL_LIGHT ~7 min,
  NEURAL ~4.5 min) and downloads the camera's factory calibration. Both are kept in
  `zed/resources` and `zed/settings`, mounted over `/usr/local/zed/{resources,settings}` via
  `~/.isaac_ros_dev-dockerargs`, so this happens once, not per container.
- depth:=ess still runs the SDK depth (NEURAL_LIGHT) because ZED tracking needs it.
- `publish_map_tf: false`: nvblox and the costmap work in `odom`; ZED's map->odom loop-closure
  jumps would tear the map.

## Outputs

Measured, 2 cm voxels, MAXN, headless:

| Topic | D435i + ESS | ZED 2i + ESS | ZED 2i + NEURAL |
|---|---|---|---|
| depth (resolution) | `/ess/depth` 848x480, ~18 Hz | `/ess/depth` 1280x720, ~12.6 Hz, ~85 % fill | `/zed/.../depth_registered` 1280x720, ~22 Hz |
| odometry | cuVSLAM ~24-30 Hz | ZED ~50 Hz | ZED ~57 Hz |
| `/nvblox_node/static_map_slice` | ~9-14 Hz | ~9.7-12.5 Hz | ~12 Hz |
| `/local_costmap/costmap` (2 cm cells) | | ~11.4 Hz | ~12.3 Hz |

The costmap publishes at the nvblox slice rate (`update_esdf_rate_hz: 15`). It used to run at
1.7 Hz: Nav2 only publishes on update ticks, and the old 5 Hz update / 2 Hz publish setting
published every third tick. Measured: 10/10 -> 6 Hz, 20/10 -> 7 Hz, 20/20 -> 11.4 Hz.
Inside Nav2 the controller reads the costmap directly at `update_frequency`; the topic is for
RViz and other consumers.

## Tuning (GPU is the limit)

| File | Setting | Effect |
|---|---|---|
| `kratos_nvblox_bringup/config/ess_<camera>.yaml` | `tta` | second ESS pass on the flipped pair, keeps only pixels both agree on. ZED 2i: off by default (27 Hz, 88 % fill; on: 15 Hz, 74 %). D435i: on (its dim IR needs the filtering) |
| `kratos_nvblox_bringup/config/ess_<camera>.yaml` | `engine_path: .../light_ess_fp16.engine` | 480x288 network, ~3 ms, coarser depth |
| `kratos_nvblox_bringup/config/nvblox_overrides.yaml` | `voxel_size: 0.03` | ~3x less nvblox work than 2 cm |
| `kratos_nvblox_bringup/config/d435i.yaml`, `kratos_nvblox_bringup/config/zed2i.yaml` | profiles / grab_resolution | camera resolution / rates |

## Using it in your own Nav2

Copy the `nvblox_layer` block from `kratos_nvblox_bringup/config/nvblox_costmap.yaml` into your
`local_costmap` (static slice) / `global_costmap` (`/nvblox_node/combined_map_slice`)
params. Set `nav2_costmap_global_frame` to that costmap's `global_frame`.

The `nvblox::nav2::NvbloxCostmapLayer` plugin is installed only in the container.
Run Nav2 there (nav2_bringup is installed), or install `ros-jazzy-nvblox-nav2`
on the host from the Isaac ROS apt repo.

On a robot, publish a static `base_link -> camera0_link` TF, set
`robot_base_frame: base_link`, and tune `esdf_slice_min/max_height` in nvblox for
the camera's mounting height.

## Files

- `kratos_nvblox_bringup/`: ROS 2 package (ament_cmake), built into `~/workspaces/isaac_ros-dev/install`
  - `launch/bringup.launch.py`: the whole pipeline. Args: `camera` (d435i|zed2i), `depth`
    (ess|native), `run_rviz`, `run_costmap`, `costmap_params`, `ess_root`, `ess_params`, `rviz_config`
  - `config/`: cameras (`d435i.yaml`, `zed2i.yaml`), ESS (`ess_d435i.yaml`, `ess_zed2i.yaml`),
    `nvblox_overrides.yaml`, `nvblox_costmap.yaml`
  - `rviz/`: `d435i.rviz`, `zed2i.rviz` (picked by `camera`)
- `docker/`: `Dockerfile.kratos`, `build_image.sh`, `99-slabs.rules`
- `zed/`: persisted ZED SDK models and calibration
- `zenoh/`: zenoh-bridge-ros2dds binary, `rover.json5`, `base_station.json5`
- `rover_up.sh`, `rover_status.sh`, `rover_down.sh`: remote bringup over SSH; `logs/`: their logs
- `ess/`: ESS node, engines (TensorRT 10.16, same as host), plugin, torch wheel, `.venv`
- `scripts/`: run/stop/build helpers, `topic_rates.py` (rate probe that doesn't undercount large images)
- `bench/`: alternative parameter files (e.g. ESS without TTA)

## Host changes made during setup

- Isaac ROS apt repo (`release-4.6`) and `isaac-ros-cli`, docker mode, realsense layer
  (`~/.config/isaac-ros-cli/config.yaml`)
- `/etc/udev/rules.d/99-slabs.rules` (ZED)
- Docker default runtime set to `nvidia` (backup: `/etc/docker/daemon.json.bak-20260924`)
- `~/.isaac_ros_dev-dockerargs` mounts this folder at `/workspaces/kratos_nvblox` and
  `zed/{resources,settings}` over `/usr/local/zed/`
- Workspace: `~/workspaces/isaac_ros-dev` (nvblox release-4.6 source, realsense_splitter build)
