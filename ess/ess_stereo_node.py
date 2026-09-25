"""Stereo depth from the D435i IR pair using NVIDIA ESS (DNN stereo disparity) on TensorRT.

Run with the ESS uv environment's Python (torch + TensorRT); the launch file does this via `prefix`.
Pre-/post-processing follows Isaac ROS's ESS pipeline: IR -> RGB, resize to the network size without
keeping the aspect ratio, normalize (x - 127.5) / 127.5, and drop pixels whose ESS confidence is
below `confidence_threshold`. The disparity is predicted at the network resolution and resampled to
the camera resolution, so /ess/depth has the same frame and format as the camera's on-chip depth
(16UC1, millimetres) and the VPI node's output.

Two additions, measured on a fixed evaluation set (~/realsense/ess/eval_ess.py, pr_curves.py):
- `input_norm: stretch` maps the joint 1st-99th intensity percentile of the IR pair to 0..255. The dim
  IR images otherwise leave ESS unsure in dark regions: accurate coverage (depth within 5 % of the
  reference) rises from 64 % to 82 % of pixels.
- `tta: true` runs ESS a second time on the vertically flipped pair (flipping both images keeps the
  epipolar geometry) and keeps pixels where both disparities agree within `tta_tolerance_px`,
  averaging them. It removes most of ESS's remaining gross errors for ~10 ms more.
"""
import ctypes
import os
import time

import message_filters
import numpy as np
import rclpy
import tensorrt as trt
import torch
import torch.nn.functional as F
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float32

TRT_LOGGER = trt.Logger(trt.Logger.WARNING)


class EssEngine:
    """Minimal TensorRT runner for the ESS engine using torch CUDA tensors for I/O."""

    def __init__(self, engine_path, plugin_path):
        # The ESS engine uses custom layers from ess_plugins.so; load it before deserializing.
        ctypes.CDLL(plugin_path, mode=ctypes.RTLD_GLOBAL)
        trt.init_libnvinfer_plugins(TRT_LOGGER, '')
        with open(engine_path, 'rb') as f:
            self.engine = trt.Runtime(TRT_LOGGER).deserialize_cuda_engine(f.read())
        if self.engine is None:
            raise RuntimeError(f'Could not load TensorRT engine {engine_path}')
        self.context = self.engine.create_execution_context()
        self.tensors = {}
        for i in range(self.engine.num_io_tensors):
            name = self.engine.get_tensor_name(i)
            shape = tuple(self.engine.get_tensor_shape(name))
            dtype = {trt.float32: torch.float32, trt.float16: torch.float16}[self.engine.get_tensor_dtype(name)]
            self.tensors[name] = torch.empty(shape, dtype=dtype, device='cuda')
            self.context.set_tensor_address(name, self.tensors[name].data_ptr())
        _, _, self.height, self.width = self.tensors['input_left'].shape

    def __call__(self, left, right):
        """left/right: (1, 3, H, W) float tensors on the GPU. Returns (disparity, confidence) (H, W)."""
        # Run on the caller's current stream so TensorRT is ordered after the kernels that produced
        # left/right (a private stream raced with them and intermittently read half-written inputs).
        stream = torch.cuda.current_stream()
        self.tensors['input_left'].copy_(left)
        self.tensors['input_right'].copy_(right)
        self.context.execute_async_v3(stream.cuda_stream)
        disparity = self.tensors['output_left'][0].float().clone()
        confidence = self.tensors['output_conf'][0].float().clone()
        stream.synchronize()
        return disparity, confidence


class EssStereoNode(Node):

    def __init__(self):
        super().__init__('ess_stereo')
        p = self.declare_parameter
        engine_path = os.path.expanduser(p('engine_path', '~/realsense/ess/engines/ess_fp16.engine').value)
        plugin_path = os.path.expanduser(p(
            'plugin_path',
            '~/realsense/ess/models/dnn_stereo_disparity_v4.1.0_onnx_trt10.16/plugins/aarch64/ess_plugins.so').value)
        self.conf_threshold = p('confidence_threshold', 0.1).value
        self.input_norm = p('input_norm', 'stretch').value           # stretch | none
        self.tta = p('tta', True).value
        self.tta_tolerance_px = p('tta_tolerance_px', 2.0).value
        self.min_depth_m = p('min_depth_m', 0.1).value
        self.max_depth_m = p('max_depth_m', 20.0).value
        # Drop pixels on depth discontinuities (mixed foreground/background "flying pixels"):
        # invalid where the disparity range in a 3x3 neighbourhood exceeds this (camera px). 0 = off.
        self.edge_disparity_px = p('edge_disparity_px', 0.0).value
        self.baseline_override_m = p('baseline_m', 0.0).value
        left_topic = p('left_topic', '/camera/camera/infra1/image_rect_raw').value
        right_topic = p('right_topic', '/camera/camera/infra2/image_rect_raw').value
        left_info_topic = p('left_info_topic', '/camera/camera/infra1/camera_info').value
        right_info_topic = p('right_info_topic', '/camera/camera/infra2/camera_info').value

        self.ess = EssEngine(engine_path, plugin_path)
        # All GPU work runs on one private stream (TensorRT warns about extra syncs on the default stream).
        self.stream = torch.cuda.Stream()
        self.bridge = CvBridge()
        self.fx = None
        self.baseline_m = None
        self.left_info = None
        self.latencies = []
        self.fills = []

        self.depth_pub = self.create_publisher(Image, 'ess/depth', 10)
        self.disp_pub = self.create_publisher(Image, 'ess/disparity', 10)
        self.conf_pub = self.create_publisher(Image, 'ess/confidence', 10)
        self.info_pub = self.create_publisher(CameraInfo, 'ess/camera_info', 10)
        self.latency_pub = self.create_publisher(Float32, 'ess/latency_ms', 10)

        self.create_subscription(CameraInfo, right_info_topic, self.on_right_info, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, left_info_topic, self.on_left_info, qos_profile_sensor_data)
        # Keep only the newest frame: when a frame takes longer than the camera period (e.g. with TTA),
        # older frames are dropped instead of queueing up and adding latency.
        latest = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        left_sub = message_filters.Subscriber(self, Image, left_topic, qos_profile=latest)
        right_sub = message_filters.Subscriber(self, Image, right_topic, qos_profile=latest)
        self.sync = message_filters.TimeSynchronizer([left_sub, right_sub], 10)
        self.sync.registerCallback(self.on_pair)
        self.get_logger().info(
            f'ESS: engine={os.path.basename(engine_path)} network {self.ess.width}x{self.ess.height} '
            f'confidence>={self.conf_threshold} input_norm={self.input_norm} '
            f'tta={self.tta} (tol {self.tta_tolerance_px}px) edge_filter={self.edge_disparity_px}px')

    def on_left_info(self, msg):
        self.left_info = msg

    def on_right_info(self, msg):
        # For the right camera, P[3] = -fx * baseline (ROS stereo convention).
        self.fx = msg.p[0]
        if self.baseline_override_m > 0:
            self.baseline_m = self.baseline_override_m
        elif msg.p[3] != 0.0:
            self.baseline_m = -msg.p[3] / msg.p[0]
        else:
            self.baseline_m = 0.050

    @staticmethod
    def stretch(left, right):
        """Maps the joint 1st-99th intensity percentile of the pair to 0..255 (same mapping for both,
        so they stay photometrically matched). Percentiles from every 4th pixel in each direction."""
        sample = torch.cat([left[..., ::4, ::4].flatten(), right[..., ::4, ::4].flatten()])
        lo, hi = torch.quantile(sample, torch.tensor([0.01, 0.99], device=sample.device))
        scale = 255.0 / (hi - lo).clamp(min=1.0)
        return ((left - lo) * scale).clamp(0, 255), ((right - lo) * scale).clamp(0, 255)

    def infer(self, left, right):
        """(1, 1, H, W) float 0..255 pair -> disparity (camera px) and confidence at camera resolution."""
        h, w = left.shape[-2:]
        antialias = h > self.ess.height or w > self.ess.width
        prep = lambda t: ((F.interpolate(t, size=(self.ess.height, self.ess.width), mode='bilinear',
                                         align_corners=False, antialias=antialias) - 127.5) / 127.5
                          ).expand(1, 3, -1, -1)
        disp_net, conf_net = self.ess(prep(left), prep(right))
        # Back to camera resolution. Nearest neighbour avoids blending foreground and background
        # disparities at object edges; disparity values are rescaled to camera pixels.
        disp = F.interpolate(disp_net[None, None], size=(h, w), mode='nearest')[0, 0] * (w / self.ess.width)
        conf = F.interpolate(conf_net[None, None], size=(h, w), mode='nearest')[0, 0]
        return disp, conf

    def on_pair(self, left_msg, right_msg):
        if self.fx is None:
            return
        t0 = time.perf_counter()
        with torch.cuda.stream(self.stream):
            depth_mm, disp, conf, valid = self.compute(left_msg, right_msg)
        latency_ms = (time.perf_counter() - t0) * 1000.0
        self.publish(left_msg.header, depth_mm, disp, conf, valid, latency_ms)

    def compute(self, left_msg, right_msg):
        left = torch.from_numpy(self.bridge.imgmsg_to_cv2(left_msg, 'mono8')).cuda().float()[None, None]
        right = torch.from_numpy(self.bridge.imgmsg_to_cv2(right_msg, 'mono8')).cuda().float()[None, None]
        if self.input_norm == 'stretch':
            left, right = self.stretch(left, right)
        disp, conf = self.infer(left, right)
        valid = torch.ones_like(disp, dtype=torch.bool)
        if self.tta:
            disp2, conf2 = self.infer(left.flip(-2), right.flip(-2))
            disp2, conf2 = disp2.flip(0), conf2.flip(0)
            valid = (disp - disp2).abs() <= self.tta_tolerance_px
            disp = (disp * conf + disp2 * conf2) / (conf + conf2).clamp(min=1e-6)
            conf = (conf + conf2) / 2
        valid &= (conf >= self.conf_threshold) & (disp > 0)
        if self.edge_disparity_px > 0:
            d = disp[None, None]
            local_range = F.max_pool2d(d, 3, 1, 1) + F.max_pool2d(-d, 3, 1, 1)
            valid &= local_range[0, 0] <= self.edge_disparity_px
        depth_m = (self.fx * self.baseline_m) / disp.clamp(min=1e-3)
        valid &= (depth_m >= self.min_depth_m) & (depth_m <= self.max_depth_m)
        depth_mm = torch.where(valid, depth_m * 1000.0, torch.zeros_like(depth_m)).round().to(torch.int32)
        depth_mm = depth_mm.clamp(0, 65535).cpu().numpy().astype(np.uint16)
        return depth_mm, disp, conf, valid

    def publish(self, header, depth_mm, disp, conf, valid, latency_ms):
        msg = self.bridge.cv2_to_imgmsg(depth_mm, '16UC1'); msg.header = header
        self.depth_pub.publish(msg)
        if self.disp_pub.get_subscription_count() > 0:
            msg = self.bridge.cv2_to_imgmsg(torch.where(valid, disp, 0.0).cpu().numpy(), '32FC1')
            msg.header = header
            self.disp_pub.publish(msg)
        if self.conf_pub.get_subscription_count() > 0:
            msg = self.bridge.cv2_to_imgmsg(conf.cpu().numpy(), '32FC1'); msg.header = header
            self.conf_pub.publish(msg)
        if self.left_info is not None:
            info = self.left_info
            info.header = header
            self.info_pub.publish(info)
        self.latency_pub.publish(Float32(data=float(latency_ms)))

        self.latencies.append(latency_ms)
        self.fills.append(float((depth_mm > 0).mean()))
        if len(self.latencies) == 60:
            lat = np.array(self.latencies)
            self.get_logger().info(f'latency ms: median {np.median(lat):.1f}, p95 {np.percentile(lat, 95):.1f}; '
                                   f'fill {100.0 * np.mean(self.fills):.1f}%')
            self.latencies.clear()
            self.fills.clear()


def main():
    rclpy.init()
    node = EssStereoNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
