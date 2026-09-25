"""Depth metrics for nvblox on identical ZED 2i frames (no ground truth available).

Inputs: bags with each method's depth (all in the left camera optical frame, 1280x720, same stamps),
plus one pose bag (/zed/zed_node/odom + /tf_static) shared by every method so that only depth differs.

Per method, on the frames every method produced:
  fill         share of pixels with depth in [0.3, 10] m
  noise        still camera: per-pixel temporal std / depth (median), and the share of pixels whose
               validity flickers; thick/flickering surfaces become noisy voxels and costmap flicker
  consistency  moving camera: depth of frame t reprojected into frame t+K with the shared poses vs the
               depth measured there (median |rel err|, share > 5 % / > 10 %). This is what nvblox's
               fusion needs: views that agree integrate into one thin surface
  flying       share of valid pixels at depth edges whose depth lies between the two sides
               (mixed foreground/background: phantom points in free space)
  agreement    median |rel diff| between methods where both are valid

Usage (host): python3 metrics.py data  -> results/metrics.json + printed tables
"""
import json
import os
import sys

import cv2
import numpy as np
import rclpy.serialization as ser
import rosbag2_py
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image
from tf2_msgs.msg import TFMessage

ZMIN, ZMAX = 0.3, 10.0
K_STEP = 10            # frames between reprojected pairs (1/3 s at 30 fps)
STRIDE = 3             # evaluate every 3rd frame (speed)
SUB = 2                # load depth at 1/SUB resolution (nearest; 4 methods x ~1800 frames fit in memory)

METHODS = {  # name: (bag dir, depth topic)
    'NEURAL_LIGHT': ('replay_neural_light', '/zed/zed_node/depth/depth_registered'),
    'NEURAL': ('replay_neural', '/zed/zed_node/depth/depth_registered'),
    'NEURAL_PLUS': ('replay_neural_plus', '/zed/zed_node/depth/depth_registered'),
    'ESS': ('ess', '/ess/depth'),
}
POSE_BAG = 'replay_neural_plus'


def read(path, topics):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=path, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    r.set_filter(rosbag2_py.StorageFilter(topics=list(topics)))
    while r.has_next():
        topic, data, _ = r.read_next()
        yield topic, ser.deserialize_message(data, topics[topic])


def stamp(h):
    return h.stamp.sec * 1_000_000_000 + h.stamp.nanosec


def to_m(msg):
    a = np.frombuffer(msg.data, dtype=np.float32 if msg.encoding == '32FC1' else np.uint16)
    a = a.reshape(msg.height, msg.width).astype(np.float32)
    if msg.encoding != '32FC1':
        a /= 1000.0
    a = a[::SUB, ::SUB].copy()
    a[~np.isfinite(a) | (a < ZMIN) | (a > ZMAX)] = 0.0
    return a


def quat_mat(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def tf_mat(t, q):
    m = np.eye(4)
    m[:3, :3] = quat_mat(q)
    m[:3, 3] = [t.x, t.y, t.z]
    return m


def load_poses(path):
    """odom -> left optical frame, per stamp."""
    static, odom = {}, {}
    for topic, msg in read(path, {'/tf_static': TFMessage, '/zed/zed_node/odom': Odometry}):
        if topic == '/tf_static':
            for t in msg.transforms:
                static[t.child_frame_id] = (t.header.frame_id, tf_mat(t.transform.translation, t.transform.rotation))
        else:
            p = msg.pose.pose
            odom[stamp(msg.header)] = (msg.child_frame_id, tf_mat(p.position, p.orientation))
    base = next(iter(odom.values()))[0]
    # chain base (zed_camera_link) -> ... -> left optical frame
    frame, chain = 'zed_left_camera_optical_frame', np.eye(4)
    while frame != base:
        parent, m = static[frame]
        chain = m @ chain
        frame = parent
    return {s: m @ chain for s, (_, m) in odom.items()}


def load_depths(path, topic, stamps=None):
    out, info = {}, None
    info_topic = topic.rsplit('/', 1)[0] + '/camera_info'
    for t, msg in read(path, {topic: Image, info_topic: CameraInfo}):
        if t == info_topic:
            info = info or msg
        elif stamps is None or stamp(msg.header) in stamps:
            out[stamp(msg.header)] = to_m(msg)
    return out, info


def reproject(d0, T0, T1, K):
    """Depth d0 seen from pose T0 (odom<-cam) rendered into camera T1: (u1, v1, predicted z) arrays."""
    h, w = d0.shape
    v, u = np.nonzero(d0)
    z = d0[v, u]
    x = (u - K[0, 2]) * z / K[0, 0]
    y = (v - K[1, 2]) * z / K[1, 1]
    P = np.linalg.inv(T1) @ T0 @ np.stack([x, y, z, np.ones_like(z)])
    zp = P[2]
    ok = zp > ZMIN
    u1 = np.round(P[0, ok] / zp[ok] * K[0, 0] + K[0, 2]).astype(int)
    v1 = np.round(P[1, ok] / zp[ok] * K[1, 1] + K[1, 2]).astype(int)
    inside = (u1 >= 0) & (u1 < w) & (v1 >= 0) & (v1 < h)
    return u1[inside], v1[inside], zp[ok][inside]


def flying_share(d):
    valid = d > 0
    k = np.ones((3, 3), np.uint8)
    dmax = cv2.dilate(d, k)
    dmin = -cv2.dilate(-np.where(valid, d, 1e6).astype(np.float32), k)
    edge = valid & ((dmax - dmin) > 0.10 * d)
    between = edge & (d > dmin * 1.03) & (d < dmax * 0.97)
    return between.sum() / max(valid.sum(), 1)


def main(data):
    poses = load_poses(os.path.join(data, POSE_BAG))
    methods = {m: v for m, v in METHODS.items() if os.path.isdir(os.path.join(data, v[0]))}
    depth, K = {}, None
    for m, (bag, topic) in methods.items():
        depth[m], info = load_depths(os.path.join(data, bag), topic)
        if K is None:   # intrinsics of the subsampled image (pixel centres at SUB*i)
            K = np.array(info.k).reshape(3, 3) / SUB
            K[2, 2] = 1.0
        print(f'{m}: {len(depth[m])} frames')
    common = sorted(set.intersection(*[set(v) for v in depth.values()]) & set(poses))
    print(f'common frames with poses: {len(common)}')

    # still segments: < 2 mm and < 0.1 deg between consecutive frames, runs of >= 15 frames
    still = []
    run = [common[0]]
    for a, b in zip(common, common[1:]):
        dT = np.linalg.inv(poses[a]) @ poses[b]
        ang = np.degrees(np.arccos(np.clip((np.trace(dT[:3, :3]) - 1) / 2, -1, 1)))
        if np.linalg.norm(dT[:3, 3]) < 0.002 and ang < 0.1:
            run.append(b)
        else:
            if len(run) >= 15:
                still.append(run)
            run = [b]
    if len(run) >= 15:
        still.append(run)
    print(f'still segments: {[len(r) for r in still]}')

    res = {}
    for m in methods:
        D = depth[m]
        r = {'fill': float(np.mean([(D[s] > 0).mean() for s in common[::STRIDE]]))}
        # temporal noise on still segments
        stds, flick = [], []
        for seg in still:
            stack = np.stack([D[s] for s in seg[:60]])
            valid = stack > 0
            often = valid.mean(0) >= 0.8
            z = np.where(valid, stack, np.nan)
            with np.errstate(invalid='ignore'):
                rel = np.nanstd(z, 0) / np.nanmedian(z, 0)
            stds.append(np.nanmedian(rel[often]))
            flick.append(float(((valid.mean(0) > 0.1) & (valid.mean(0) < 0.9)).mean()))
        r['temporal_rel_std'] = float(np.mean(stds)) if stds else None
        r['flicker_share'] = float(np.mean(flick)) if flick else None
        # multi-view consistency on moving frames
        errs = []
        still_set = {s for seg in still for s in seg}
        for i in range(0, len(common) - K_STEP, STRIDE):
            a, b = common[i], common[i + K_STEP]
            if a in still_set and b in still_set:
                continue
            u, v, zp = reproject(D[a], poses[a], poses[b], K)
            zb = D[b][v, u]
            ok = zb > 0
            errs.append(np.abs(zp[ok] - zb[ok]) / zb[ok])
        e = np.concatenate(errs) if errs else np.array([np.nan])
        r['consistency_median'] = float(np.median(e))
        r['consistency_gt5'] = float((e > 0.05).mean())
        r['consistency_gt10'] = float((e > 0.10).mean())
        r['flying_share'] = float(np.mean([flying_share(D[s]) for s in common[::STRIDE * 3]]))
        res[m] = r
    agree = {}
    for a in methods:
        for b in methods:
            if a < b:
                diffs = []
                for s in common[::STRIDE * 3]:
                    ok = (depth[a][s] > 0) & (depth[b][s] > 0)
                    diffs.append(np.median(np.abs(depth[a][s][ok] - depth[b][s][ok]) / depth[b][s][ok]))
                agree[f'{a} vs {b}'] = float(np.median(diffs))
    os.makedirs('results', exist_ok=True)
    json.dump({'methods': res, 'agreement': agree, 'frames': len(common),
               'still_segments': [len(s) for s in still]}, open('results/metrics.json', 'w'), indent=2)
    print(f'\n{"method":<14}{"fill":>7}{"noise%":>8}{"flicker":>9}{"cons.med%":>10}{">5%":>7}{">10%":>7}{"flying":>8}')
    for m, r in res.items():
        f = lambda x, s=100: f'{s * x:.2f}' if x is not None else '-'
        print(f'{m:<14}{r["fill"]:7.3f}{f(r["temporal_rel_std"]):>8}{f(r["flicker_share"], 1):>9}'
              f'{f(r["consistency_median"]):>10}{f(r["consistency_gt5"]):>7}{f(r["consistency_gt10"]):>7}'
              f'{f(r["flying_share"]):>8}')
    print('\nagreement (median |rel diff|):')
    for k, v in agree.items():
        print(f'  {k}: {100 * v:.2f} %')


if __name__ == '__main__':
    main(sys.argv[1] if len(sys.argv) > 1 else 'data')
