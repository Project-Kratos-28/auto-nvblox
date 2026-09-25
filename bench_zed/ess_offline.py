"""Run ESS on every stereo pair of a replay bag and write its depth to a new bag.

Uses the ESS node's own processing (ess/ess_stereo_node.py: EssStereoNode.compute) with the
parameters from config/ess_zed2i.yaml, so the result is what the live node would publish, but for
every frame (live, the node drops frames it cannot keep up with). Output: /ess/depth (16UC1 mm,
left gray frame, same stamps as the input) and /ess/camera_info.

Run in the bench container with the ESS environment:
  /workspaces/kratos_nvblox/ess/.venv/bin/python ess_offline.py data/replay_neural_light data/ess
"""
import os
import sys
import time

import numpy as np
import rclpy.serialization as ser
import rosbag2_py
import torch
import yaml
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image

ROOT = '/workspaces/kratos_nvblox'
sys.path.insert(0, f'{ROOT}/ess')
from ess_stereo_node import EssEngine, EssStereoNode  # noqa: E402

LEFT = '/zed/zed_node/left/gray/rect/image'
RIGHT = '/zed/zed_node/right/gray/rect/image'
LEFT_INFO = '/zed/zed_node/left/gray/rect/camera_info'
RIGHT_INFO = '/zed/zed_node/right/gray/rect/camera_info'


def reader(path):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=path, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    r.set_filter(rosbag2_py.StorageFilter(topics=[LEFT, RIGHT, LEFT_INFO, RIGHT_INFO]))
    return r


def main(src, dst, config=f'{ROOT}/config/ess_zed2i.yaml'):
    p = yaml.safe_load(open(config))['ess_stereo']['ros__parameters']
    node = EssStereoNode.__new__(EssStereoNode)          # processing only, no ROS node
    node.ess = EssEngine(p['engine_path'], p['plugin_path'])
    node.bridge = CvBridge()
    node.stream = torch.cuda.Stream()
    node.conf_threshold = p.get('confidence_threshold', 0.1)
    node.input_norm = p.get('input_norm', 'stretch')
    node.tta = p.get('tta', True)
    node.tta_tolerance_px = p.get('tta_tolerance_px', 2.0)
    node.edge_disparity_px = p.get('edge_disparity_px', 0.0)
    node.min_depth_m = p.get('min_depth_m', 0.1)
    node.max_depth_m = p.get('max_depth_m', 20.0)
    node.fx = node.baseline_m = None
    left_info = None

    w = rosbag2_py.SequentialWriter()
    w.open(rosbag2_py.StorageOptions(uri=dst, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    for name, typ in (('/ess/depth', 'sensor_msgs/msg/Image'), ('/ess/camera_info', 'sensor_msgs/msg/CameraInfo')):
        w.create_topic(rosbag2_py.TopicMetadata(name=name, type=typ, serialization_format='cdr'))

    pending = {LEFT: {}, RIGHT: {}}
    r = reader(src)
    n, times = 0, []
    while r.has_next():
        topic, data, t = r.read_next()
        if topic == RIGHT_INFO and node.fx is None:
            info = ser.deserialize_message(data, CameraInfo)
            node.fx, node.baseline_m = info.p[0], -info.p[3] / info.p[0]
            print(f'fx {node.fx:.1f}  baseline {node.baseline_m * 1000:.1f} mm')
        elif topic == LEFT_INFO:
            left_info = ser.deserialize_message(data, CameraInfo)
        elif topic in (LEFT, RIGHT):
            msg = ser.deserialize_message(data, Image)
            stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
            pending[topic][stamp] = msg
            if node.fx is None or left_info is None or stamp not in pending[LEFT] or stamp not in pending[RIGHT]:
                continue
            lmsg, rmsg = pending[LEFT].pop(stamp), pending[RIGHT].pop(stamp)
            for q in pending.values():                  # drop unmatched older frames
                for s in [s for s in q if s < stamp]:
                    del q[s]
            t0 = time.perf_counter()
            with torch.cuda.stream(node.stream):
                depth_mm, *_ = node.compute(lmsg, rmsg)
            times.append((time.perf_counter() - t0) * 1000)
            out = node.bridge.cv2_to_imgmsg(depth_mm, '16UC1')
            out.header = lmsg.header
            left_info.header = lmsg.header
            w.write('/ess/depth', ser.serialize_message(out), t)
            w.write('/ess/camera_info', ser.serialize_message(left_info), t)
            n += 1
            if n % 100 == 0:
                print(f'{n} frames, {np.median(times[-100:]):.1f} ms/frame', flush=True)
    del w
    print(f'done: {n} frames -> {dst}; median {np.median(times):.1f} ms/frame')


if __name__ == '__main__':
    main(*sys.argv[1:])
