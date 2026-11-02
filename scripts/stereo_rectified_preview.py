"""Rectify live JPEG feeds using their published CameraInfo (no depth estimation)."""
import argparse
import json
import time
from collections import deque
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage, Image


class Preview(Node):
    def __init__(self):
        super().__init__('stereo_rectified_preview')
        self.maps, self.frames, self.info = {}, {}, {}
        self.counts = dict(left=0, right=0)
        self.publishers_by_side = {}
        for side in self.counts:
            self.publishers_by_side[side] = self.create_publisher(
                Image, f'/stereo/{side}/camera/image_rect', 2)
            self.create_subscription(CameraInfo, f'/stereo/{side}/camera/camera_info',
                                     lambda msg, s=side: self.camera_info(s, msg), qos_profile_sensor_data)
            self.create_subscription(CompressedImage, f'/stereo/{side}/camera/image_raw/compressed',
                                     lambda msg, s=side: self.frame(s, msg), qos_profile_sensor_data)

    def camera_info(self, side, msg):
        if msg.k[0] <= 0 or msg.width != 640 or msg.height != 360:
            return
        signature = tuple(msg.k) + tuple(msg.d) + tuple(msg.r) + tuple(msg.p)
        if side in self.info and self.info[side]['signature'] == signature:
            return
        self.maps[side] = cv2.initUndistortRectifyMap(
            np.array(msg.k).reshape(3, 3), np.array(msg.d),
            np.array(msg.r).reshape(3, 3), np.array(msg.p).reshape(3, 4)[:, :3],
            (msg.width, msg.height), cv2.CV_32FC1)
        self.info[side] = dict(signature=signature, width=msg.width, height=msg.height,
                               fx=msg.k[0], projection_tx=msg.p[3], frame_id=msg.header.frame_id)

    def frame(self, side, msg):
        if side not in self.maps:
            return
        raw = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if raw is None or raw.shape[:2] != (360, 640):
            return
        rect = cv2.remap(raw, *self.maps[side], cv2.INTER_LINEAR)
        self.frames[side] = (rect, time.monotonic())
        self.counts[side] += 1
        out = Image()
        out.header = msg.header
        out.height, out.width = rect.shape[:2]
        out.encoding, out.step = 'bgr8', rect.shape[1] * 3
        out.data = rect.tobytes()
        self.publishers_by_side[side].publish(out)

    def preview(self):
        if len(self.frames) != 2:
            return None
        panel = np.hstack([self.frames[s][0] for s in ('left', 'right')]).copy()
        for y in range(40, 360, 40):
            cv2.line(panel, (0, y), (1279, y), (0, 255, 0), 1)
        for side, x in [('left', 8), ('right', 648)]:
            age = time.monotonic() - self.frames[side][1]
            cv2.putText(panel, f'{side.upper()}  age {age:.1f}s', (x, 24),
                        cv2.FONT_HERSHEY_SIMPLEX, .6, (0, 255, 255), 1)
        return panel


class DepthPreview(Preview):
    """Diagnostic SGBM depth from bounded, one-use timestamp pairs."""

    def __init__(self, max_skew_ms):
        super().__init__()
        self.max_skew = max_skew_ms / 1000
        self.pending = {s: deque(maxlen=5) for s in ('left', 'right')}
        self.last_stamp = dict(left=0, right=0)
        self.skews = deque(maxlen=1000)
        self.pairs = 0
        self.rejected = 0
        self.depth_panel = None
        self.updated = 0
        self.valid_fraction = 0
        self.matcher = cv2.StereoSGBM_create(
            minDisparity=0, numDisparities=128, blockSize=5,
            P1=8*25, P2=32*25, disp12MaxDiff=1, uniquenessRatio=15,
            speckleWindowSize=100, speckleRange=2,
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY)
        self.depth_pub = self.create_publisher(Image, '/stereo/depth/image', 2)

    def frame(self, side, msg):
        count = self.counts[side]
        super().frame(side, msg)
        if self.counts[side] == count:
            return
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        if stamp <= self.last_stamp[side]:
            self.rejected += 1
            return
        self.last_stamp[side] = stamp
        if len(self.pending[side]) == self.pending[side].maxlen:
            self.rejected += 1
        self.pending[side].append((stamp, self.frames[side][0], msg.header))
        left, right = self.pending['left'], self.pending['right']
        while left and right:
            delta = (left[0][0] - right[0][0]) / 1e9
            if abs(delta) > self.max_skew:
                (left if delta < 0 else right).popleft()
                self.rejected += 1
                continue
            l, r = left.popleft(), right.popleft()
            self.compute(l, r, delta)

    def compute(self, left, right, delta):
        lp = np.array(self.info['left']['signature'][-12:]).reshape(3, 4)
        rp = np.array(self.info['right']['signature'][-12:]).reshape(3, 4)
        if not np.allclose(lp[:, :3], rp[:, :3]) or rp[0, 3] >= lp[0, 3]:
            raise RuntimeError('Depth requires matching rectified intrinsics and positive baseline')
        gray = [cv2.cvtColor(v[1], cv2.COLOR_BGR2GRAY) for v in (left, right)]
        disparity = self.matcher.compute(*gray).astype(np.float32) / 16
        depth = np.full(disparity.shape, np.nan, np.float32)
        valid = disparity > 0
        depth[valid] = (lp[0, 3] - rp[0, 3]) / disparity[valid]
        valid &= (depth >= .3) & (depth <= 5)
        depth[~valid] = np.nan
        self.valid_fraction = float(valid.mean())
        colour = cv2.applyColorMap(
            (255 * (1 - np.nan_to_num(np.clip((depth-.3)/4.7, 0, 1), nan=1))).astype(np.uint8),
            cv2.COLORMAP_JET)
        colour[~valid] = 0
        crop = depth[160:200, 300:340]
        centre = crop[np.isfinite(crop)]
        label = f'Centre: {np.median(centre):.2f} m' if len(centre) >= 100 else 'Centre: no reliable match'
        for im in (colour,):
            cv2.rectangle(im, (300, 160), (340, 200), (255, 255, 255), 1)
        # Separate legend preserves the full camera/depth image area. Use the
        # same linear metre-to-JET mapping as the depth pixels above.
        legend = np.full((360, 150, 3), 30, np.uint8)
        cv2.putText(legend, 'Depth (m)', (10, 25), cv2.FONT_HERSHEY_SIMPLEX,
                    .55, (255, 255, 255), 1)
        scale_depth = np.linspace(.3, 5, 261)
        scale_values = (255 * (1 - np.clip((scale_depth-.3)/4.7, 0, 1))).astype(np.uint8)
        scale = cv2.applyColorMap(scale_values[:, None], cv2.COLORMAP_JET)
        legend[45:306, 12:42] = np.repeat(scale, 30, axis=1)
        for metres in (.3, .5, 1, 1.5, 2, 2.5, 3, 3.5, 4, 4.5, 5):
            y = 45 + round((metres-.3)/4.7 * 260)
            cv2.line(legend, (42, y), (49, y), (255, 255, 255), 1)
            cv2.putText(legend, f'{metres:g}', (55, y+4),
                        cv2.FONT_HERSHEY_SIMPLEX, .4, (255, 255, 255), 1)
        cv2.rectangle(legend, (12, 327), (30, 343), (0, 0, 0), -1)
        cv2.putText(legend, 'Unknown', (38, 340), cv2.FONT_HERSHEY_SIMPLEX,
                    .4, (255, 255, 255), 1)
        self.depth_panel = np.hstack((left[1], colour, legend))
        cv2.putText(self.depth_panel, label, (650, 55), cv2.FONT_HERSHEY_SIMPLEX, .6, (255,255,255), 1)
        cv2.putText(self.depth_panel, 'Near RED (0.3m) - Far BLUE (5m); black = unknown',
                    (650, 340), cv2.FONT_HERSHEY_SIMPLEX, .45, (255,255,255), 1)
        self.skews.append(abs(delta)*1000)
        self.pairs += 1
        self.updated = time.monotonic()
        out = Image()
        out.header = left[2]
        out.height, out.width = depth.shape
        out.encoding, out.step = '32FC1', depth.shape[1]*4
        out.data = depth.tobytes()
        self.depth_pub.publish(out)

    def preview(self):
        if self.depth_panel is None:
            return None
        panel = self.depth_panel.copy()
        cv2.putText(panel, f'Pair gap {self.skews[-1]:.1f} ms | age {time.monotonic()-self.updated:.1f}s | '
                    f'valid {self.valid_fraction:.0%}', (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, .6, (255,255,255), 1)
        return panel


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--seconds', type=float, default=0)
    parser.add_argument('--snapshot', type=Path)
    parser.add_argument('--depth', action='store_true', help='Show diagnostic stereo depth in metres')
    parser.add_argument('--max-skew-ms', type=float, default=30, help='Maximum accepted header timestamp gap')
    args = parser.parse_args()
    rclpy.init()
    if not 0 < args.max_skew_ms <= 50:
        parser.error('--max-skew-ms must be greater than zero and at most 50')
    node = DepthPreview(args.max_skew_ms) if args.depth else Preview()
    window = 'Stereo depth - Q to close' if args.depth else 'Rectified stereo - latest frames, not synchronized - Q to close'
    start = time.monotonic()
    try:
        while rclpy.ok() and (not args.seconds or time.monotonic() - start < args.seconds):
            rclpy.spin_once(node, timeout_sec=.05)
            panel = node.preview()
            if not args.headless:
                if panel is None:
                    panel = np.zeros((360, 1280, 3), np.uint8)
                    cv2.putText(panel, 'Waiting for calibrated images from BOTH cameras...',
                                (20, 160), cv2.FONT_HERSHEY_SIMPLEX, .8, (255, 255, 255), 1)
                cv2.imshow(window, panel)
                if cv2.waitKey(1) & 255 in (ord('q'), 27):
                    break
        if args.snapshot and node.preview() is not None:
            args.snapshot.parent.mkdir(parents=True, exist_ok=True)
            if not cv2.imwrite(str(args.snapshot), node.preview()):
                raise RuntimeError('Unable to write snapshot')
        print(json.dumps(dict(frames=node.counts, camera_info=node.info), indent=2))
        if args.depth:
            print(json.dumps(dict(pairs=node.pairs, rejected_frames=node.rejected,
                                 median_skew_ms=float(np.median(node.skews)) if node.skews else None,
                                 p95_skew_ms=float(np.percentile(node.skews, 95)) if node.skews else None,
                                 last_valid_fraction=node.valid_fraction), indent=2))
            if not node.pairs:
                raise RuntimeError('No stereo pairs within the timestamp limit')
        if not all(node.counts.values()):
            raise RuntimeError('Did not receive calibrated images from both cameras')
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
