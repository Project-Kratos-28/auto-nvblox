"""ZED 2i for the depth benchmark: live (to record an SVO) or replaying an SVO with a chosen depth mode.

Same camera settings as kratos (config/zed2i.yaml: HD720, 30 fps published, gray rect pair),
plus inline overrides:
  svo:=live         depth_mode NONE and tracking off (recording only needs the raw stereo + IMU)
  svo:=<file.svo2>  replay every frame (svo_realtime false: grab blocks until processed), with
                    depth_mode:=NEURAL_LIGHT|NEURAL|NEURAL_PLUS and ZED tracking (odom TF) on

  ros2 launch /workspaces/kratos_nvblox/bench_zed/launch/zed_bench.launch.py svo:=live
  ros2 launch ... svo:=/workspaces/kratos_nvblox/bench_zed/data/lab.svo2 depth_mode:=NEURAL
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource

CONFIG = '/workspaces/kratos_nvblox/config/zed2i.yaml'


def setup(context):
    svo = context.launch_configurations['svo']
    mode = context.launch_configurations['depth_mode']
    live = svo == 'live'
    overrides = {
        'depth.depth_mode': 'NONE' if live else mode,
        'pos_tracking.pos_tracking_enabled': 'false' if live else 'true',
        'svo.svo_realtime': 'false',
        'svo.svo_loop': 'false',
        'svo.use_svo_timestamps': 'true',
        'general.pub_frame_rate': '0.0' if not live else '30.0',   # replay: publish every grabbed frame
    }
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('zed_wrapper'), 'launch', 'zed_camera.launch.py')),
        launch_arguments={
            'camera_model': 'zed2i',
            'camera_name': 'zed',
            'ros_params_override_path': CONFIG,
            'svo_path': svo,
            'publish_tf': 'true',
            'publish_map_tf': 'false',
            'param_overrides': ';'.join(f'{k}:={v}' for k, v in overrides.items()),
        }.items())]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('svo', default_value='live', description='live | path to .svo2'),
        DeclareLaunchArgument('depth_mode', default_value='NEURAL',
                              description='NEURAL_LIGHT | NEURAL | NEURAL_PLUS (replay only)'),
        OpaqueFunction(function=setup),
    ])
