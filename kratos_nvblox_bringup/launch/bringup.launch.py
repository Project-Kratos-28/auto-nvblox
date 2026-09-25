"""Kratos nvblox bringup: stereo camera -> depth + odometry -> nvblox -> Nav2 costmap.

    ros2 launch kratos_nvblox_bringup bringup.launch.py camera:=d435i
    ros2 launch kratos_nvblox_bringup bringup.launch.py camera:=zed2i
    ros2 launch kratos_nvblox_bringup bringup.launch.py camera:=zed2i depth:=native run_rviz:=False

camera:=d435i (RealSense D435i)
    848x480, projector flashing @60 -> realsense_splitter
        projector-off IR pair @30 -> cuVSLAM              TF odom -> camera0_link
                                  -> ESS    (depth:=ess)     /ess/depth
        projector-on depth   @30  ->        (depth:=native)  on-chip depth

camera:=zed2i (ZED 2i)
    1280x720 @30 (zed-ros2-wrapper) -> ZED visual-inertial tracking   TF odom -> zed_camera_link
        gray rectified pair -> ESS  (depth:=ess)             /ess/depth
        ZED SDK NEURAL depth        (depth:=native)          /zed/zed_node/depth/depth_registered

then  depth + color + TF -> nvblox (2 cm voxels)             /nvblox_node/static_map_slice
                         -> Costmap2DROS + NvbloxCostmapLayer /local_costmap/costmap

ESS runs as its own process from ess_root (TensorRT + torch venv); everything else is ROS packages.

Remote operation (the rover is reached over a WLAN link, e.g. Ubiquiti Rocket 5AC):
  - Headless by default: no RViz on the Orin (it also costs GPU time ESS and nvblox need).
  - DDS stays on the Orin (ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST in scripts/run_nvblox.sh), so
    full-rate images, depth and the 12 Hz costmap never cross the radio link.
  - run_remote: throttled copies for the base station under /remote/: costmap (remote_costmap_hz),
    JPEG color (remote_image_hz, remote_jpeg_quality), ESDF slice Cloudini-compressed (remote_esdf_hz).
  - run_zenoh: zenoh-bridge-ros2dds forwards only /remote/*, TF and odometry (zenoh/rover.json5).
  - run_foxglove (off): optional websocket viewer with a similar whitelist.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

SHARE = get_package_share_directory('kratos_nvblox_bringup')
CONFIG = os.path.join(SHARE, 'config')
CONTAINER = 'nvblox_container'   # NVIDIA's vslam.launch.py loads cuVSLAM into this container by name

CAMERAS = {
    'd435i': {
        # depth option -> (depth image, depth camera_info)
        'depth': {
            'ess': ('/ess/depth', '/ess/camera_info'),
            'native': ('/camera0/realsense_splitter_node/output/depth', '/camera0/depth/camera_info'),
        },
        'color': ('/camera0/color/image_raw', '/camera0/color/camera_info'),
        'color_compressed': '/camera0/color/image_raw/compressed',
        'odom': '/visual_slam/tracking/odometry',
        'base_frame': 'camera0_link',
        'nvblox_config': 'config/nvblox/specializations/nvblox_realsense.yaml',
    },
    'zed2i': {
        'depth': {
            'ess': ('/ess/depth', '/ess/camera_info'),
            'native': ('/zed/zed_node/depth/depth_registered', '/zed/zed_node/depth/camera_info'),
        },
        'color': ('/zed/zed_node/rgb/color/rect/image', '/zed/zed_node/rgb/color/rect/camera_info'),
        'color_compressed': '/zed/zed_node/rgb/color/rect/image/compressed',
        'odom': '/zed/zed_node/odom',
        'base_frame': 'zed_camera_link',
        'nvblox_config': 'config/nvblox/specializations/nvblox_zed.yaml',
    },
}


def d435i(nvblox_share, depth, context):
    """D435i driver + splitter (in the nvblox container) and cuVSLAM for odometry."""
    camera = ComposableNode(
        name='camera0', namespace='',
        package='realsense2_camera', plugin='realsense2_camera::RealSenseNodeFactory',
        parameters=[os.path.join(CONFIG, 'd435i.yaml'), {'camera_name': 'camera0'}])
    # Splits the flashing-projector stream: projector-off IR frames for cuVSLAM and ESS,
    # projector-on depth frames for depth:=native.
    splitter = ComposableNode(
        name='realsense_splitter_node', namespace='camera0',
        package='realsense_splitter', plugin='nvblox::RealsenseSplitterNode',
        parameters=[{'input_qos': 'SENSOR_DATA', 'output_qos': 'SENSOR_DATA'}],
        remappings=[
            ('input/infra_1', '/camera0/infra1/image_rect_raw'),
            ('input/infra_1_metadata', '/camera0/infra1/metadata'),
            ('input/infra_2', '/camera0/infra2/image_rect_raw'),
            ('input/infra_2_metadata', '/camera0/infra2/metadata'),
            ('input/depth', '/camera0/depth/image_rect_raw'),
            ('input/depth_metadata', '/camera0/depth/metadata'),
            ('input/pointcloud', '/camera0/depth/color/points'),
            ('input/pointcloud_metadata', '/camera0/depth/metadata'),
        ])
    # cuVSLAM, loaded into the nvblox container (NVIDIA's launch, delayed as in their example).
    vslam = TimerAction(period=1.0, actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nvblox_share, 'launch/perception/vslam.launch.py')),
        launch_arguments={'container_name': CONTAINER, 'camera': 'realsense'}.items())])
    return [camera, splitter], [vslam]


def zed2i(nvblox_share, depth, context):
    """zed-ros2-wrapper's own launch: robot_state_publisher (camera frames) + zed_node in its own
    container. Its positional tracking publishes odom -> zed_camera_link."""
    zed = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('zed_wrapper'), 'launch', 'zed_camera.launch.py')),
        launch_arguments={
            'camera_model': 'zed2i',
            'camera_name': 'zed',
            'ros_params_override_path': os.path.join(CONFIG, 'zed2i.yaml'),
            'publish_tf': 'true',
            'publish_map_tf': 'false',
            # Tracking needs the SDK's depth running even when nvblox uses ESS; the lighter
            # model is enough for that.
            'param_overrides': ';'.join([
                'depth.depth_mode:=' + ('NEURAL' if depth == 'native' else 'NEURAL_LIGHT'),
                # image_transport's JPEG default (95) is ~260 KB per 1280x720 frame; 60 is ~65 KB.
                # Only the remote link uses the compressed stream.
                '.zed_node.rgb.color.rect.image.compressed.jpeg_quality:=' +
                LaunchConfiguration('remote_jpeg_quality').perform(context),
            ]),
        }.items())
    return [], [zed]


def launch_setup(context):
    nvblox_share = get_package_share_directory('nvblox_examples_bringup')
    camera = LaunchConfiguration('camera').perform(context)
    depth = LaunchConfiguration('depth').perform(context)
    cam = CAMERAS[camera]
    depth_image, depth_info = cam['depth'][depth]
    color_image, color_info = cam['color']

    container_nodes, actions = {'d435i': d435i, 'zed2i': zed2i}[camera](nvblox_share, depth, context)
    actions.insert(0, LogInfo(msg=f'kratos_nvblox_bringup: camera={camera} depth={depth} '
                                  f'-> nvblox depth {depth_image}'))

    nvblox = ComposableNode(
        name='nvblox_node',
        package='nvblox_ros', plugin='nvblox::NvbloxNode',
        parameters=[
            os.path.join(nvblox_share, 'config/nvblox/nvblox_base.yaml'),
            os.path.join(nvblox_share, cam['nvblox_config']),
            os.path.join(CONFIG, 'nvblox_overrides.yaml'),
            {'num_cameras': 1, 'use_lidar': False,
             # Caps nvblox's mesh/layer visualization stream (only sent when subscribed).
             'layer_streamer_bandwidth_limit_mbps':
                 float(LaunchConfiguration('stream_bandwidth_mbps').perform(context))},
        ],
        remappings=[
            ('camera_0/depth/image', depth_image),
            ('camera_0/depth/camera_info', depth_info),
            ('camera_0/color/image', color_image),
            ('camera_0/color/camera_info', color_info),
        ])
    actions.insert(1, ComposableNodeContainer(
        name=CONTAINER, namespace='',
        package='rclcpp_components', executable='component_container_mt',
        composable_node_descriptions=container_nodes + [nvblox],
        output='screen'))

    if depth == 'ess':
        ess_root = LaunchConfiguration('ess_root').perform(context)
        ess_params = (LaunchConfiguration('ess_params').perform(context)
                      or os.path.join(CONFIG, f'ess_{camera}.yaml'))
        actions.append(ExecuteProcess(
            cmd=[os.path.join(ess_root, '.venv/bin/python'), os.path.join(ess_root, 'ess_stereo_node.py'),
                 '--ros-args', '--params-file', ess_params],
            name='ess_stereo', output='screen',
            # Unattended rover: restart ESS if it ever dies (e.g. a CUDA error).
            respawn=True, respawn_delay=3.0))

    # nav2_costmap_2d's executable hosts a single Costmap2DROS lifecycle node. Naming it like
    # Nav2's own local costmap gives the usual topics: /local_costmap/costmap etc.
    actions.append(Node(
        package='nav2_costmap_2d', executable='nav2_costmap_2d',
        name='local_costmap', namespace='local_costmap', output='screen',
        parameters=[LaunchConfiguration('costmap_params'), {'robot_base_frame': cam['base_frame']}],
        condition=IfCondition(LaunchConfiguration('run_costmap'))))

    # Remote interface for the base station: throttled copies of heavy topics under /remote/.
    # Onboard consumers keep the full-rate originals; only /remote/* (+ TF, odometry) are meant
    # to cross the radio link, whatever carries them (Zenoh bridge, Foxglove, ...).
    run_remote = IfCondition(LaunchConfiguration('run_remote'))
    actions.append(Node(
        package='kratos_nvblox_bringup', executable='throttle_relay.py', name='remote_costmap_relay',
        parameters=[{'input_topic': '/local_costmap/costmap', 'output_topic': '/remote/costmap',
                     'max_rate_hz': float(LaunchConfiguration('remote_costmap_hz').perform(context))}],
        condition=run_remote))
    actions.append(Node(
        package='kratos_nvblox_bringup', executable='throttle_relay.py', name='remote_esdf_relay',
        parameters=[{'input_topic': '/nvblox_node/static_esdf_pointcloud', 'output_topic': '/remote/esdf_slice',
                     'max_rate_hz': float(LaunchConfiguration('remote_esdf_hz').perform(context))}],
        condition=run_remote))
    actions.append(Node(
        package='kratos_nvblox_bringup', executable='throttle_relay.py', name='remote_image_relay',
        parameters=[{'input_topic': cam['color_compressed'], 'output_topic': '/remote/color/compressed',
                     'max_rate_hz': float(LaunchConfiguration('remote_image_hz').perform(context)),
                     'reliable': False}],
        condition=run_remote))

    # The ESDF slice is a PointCloud2 (~620 KB); Cloudini brings it to ~30 KB at 5 mm resolution
    # (voxels are 20 mm). Decode on the base station with cloudini_ros / point_cloud_transport.
    actions.append(Node(
        package='cloudini_ros', executable='cloudini_topic_converter', name='remote_esdf_cloudini',
        parameters=[{'topic_input': '/remote/esdf_slice', 'topic_output': '/remote/esdf_slice/cloudini',
                     'compressing': True, 'resolution': 0.005}],
        condition=run_remote))

    # Zenoh bridge: the only process talking over the radio link. Forwards the allow-listed
    # /remote/* topics, TF and odometry (zenoh/rover.json5); DDS itself stays on the Orin.
    zenoh_root = LaunchConfiguration('zenoh_root').perform(context)
    actions.append(ExecuteProcess(
        cmd=[os.path.join(zenoh_root, 'zenoh-bridge-ros2dds'), '-c', os.path.join(zenoh_root, 'rover.json5')],
        name='zenoh_bridge', output='screen', respawn=True, respawn_delay=3.0,
        condition=IfCondition(LaunchConfiguration('run_zenoh'))))

    # Optional Foxglove viewer (websocket); off by default: the base station GUI is PyQt5.
    run_foxglove = IfCondition(LaunchConfiguration('run_foxglove'))
    actions.append(Node(
        package='foxglove_bridge', executable='foxglove_bridge', name='foxglove_bridge',
        parameters=[{
            'port': int(LaunchConfiguration('foxglove_port').perform(context)),
            'address': '0.0.0.0',
            'topic_whitelist': [
                '^/tf$', '^/tf_static$',
                '^/remote/.*',
                '^' + cam['odom'] + '$',
                '^/local_costmap/published_footprint$',
                '^/nvblox_node/mesh$',                     # capped by stream_bandwidth_mbps
                '^/nvblox_node/esdf_slice_bounds$',
                '^/ess/latency_ms$',
                '^/diagnostics$',
            ],
            'max_qos_depth': 1,
            # Drop messages for a slow client instead of queueing (default 10 MB).
            'send_buffer_limit': 2000000,
            'num_threads': 2,
        }],
        arguments=['--ros-args', '--log-level', 'ERROR'],
        condition=run_foxglove))

    # The rviz configs show /ess/depth; remap so the panel shows whatever nvblox integrates.
    rviz_config = (LaunchConfiguration('rviz_config').perform(context)
                   or os.path.join(SHARE, 'rviz', f'{camera}.rviz'))
    actions.append(Node(
        package='rviz2', executable='rviz2', name='rviz2', arguments=['-d', rviz_config],
        remappings=[('/ess/depth', depth_image)] if depth_image != '/ess/depth' else [],
        output='log', condition=IfCondition(LaunchConfiguration('run_rviz'))))
    return actions


def generate_launch_description():
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_costmap', output='screen',
        parameters=[{'autostart': True, 'node_names': ['local_costmap/local_costmap'],
                     'bond_timeout': 0.0}],
        condition=IfCondition(LaunchConfiguration('run_costmap')))

    return LaunchDescription([
        DeclareLaunchArgument('camera', default_value='d435i', choices=list(CAMERAS),
                              description='Stereo camera feeding nvblox'),
        DeclareLaunchArgument('depth', default_value='ess', choices=['ess', 'native'],
                              description="ess: NVIDIA ESS on the camera's stereo pair; native: the "
                                          "camera's own depth (D435i on-chip / ZED SDK NEURAL)"),
        DeclareLaunchArgument('run_rviz', default_value='False', choices=['True', 'False', 'true', 'false'],
                              description='RViz on the Orin itself (needs a display; off for remote use)'),
        DeclareLaunchArgument('run_remote', default_value='True', choices=['True', 'False', 'true', 'false'],
                              description='Throttled /remote/* topics for the base station'),
        DeclareLaunchArgument('run_zenoh', default_value='True', choices=['True', 'False', 'true', 'false'],
                              description='zenoh-bridge-ros2dds to the base station (zenoh/rover.json5)'),
        DeclareLaunchArgument('zenoh_root', default_value='/workspaces/kratos_nvblox/zenoh',
                              description='zenoh-bridge-ros2dds binary and rover.json5'),
        DeclareLaunchArgument('run_foxglove', default_value='False', choices=['True', 'False', 'true', 'false'],
                              description='foxglove_bridge websocket viewer (port foxglove_port)'),
        DeclareLaunchArgument('foxglove_port', default_value='8765'),
        DeclareLaunchArgument('remote_costmap_hz', default_value='2.0',
                              description='Rate of /remote/costmap (the onboard costmap stays at full rate)'),
        DeclareLaunchArgument('remote_image_hz', default_value='5.0',
                              description='Rate of /remote/color/compressed'),
        DeclareLaunchArgument('remote_esdf_hz', default_value='4.0',
                              description='Rate of /remote/esdf_slice[/cloudini] (~30 KB per message compressed)'),
        DeclareLaunchArgument('remote_jpeg_quality', default_value='60',
                              description='JPEG quality of the ZED compressed color stream'),
        DeclareLaunchArgument('stream_bandwidth_mbps', default_value='5.0',
                              description="Cap on nvblox's mesh visualization stream"),
        DeclareLaunchArgument('run_costmap', default_value='True', choices=['True', 'False', 'true', 'false'],
                              description='Standalone Nav2 costmap; set False when a full Nav2 stack runs'),
        DeclareLaunchArgument('costmap_params', default_value=os.path.join(CONFIG, 'nvblox_costmap.yaml')),
        DeclareLaunchArgument('ess_root', default_value='/workspaces/kratos_nvblox/ess',
                              description='ESS node, TensorRT engines and its .venv'),
        DeclareLaunchArgument('ess_params', default_value='',
                              description='ESS parameter file (default: config/ess_<camera>.yaml)'),
        DeclareLaunchArgument('rviz_config', default_value='',
                              description='RViz config (default: rviz/<camera>.rviz)'),
        OpaqueFunction(function=launch_setup),
        lifecycle_manager,
    ])
