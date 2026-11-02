"""Wait for mapping sensor TF before starting RTAB-Map's image subscribers."""
import argparse
import time

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--lidar', action='store_true')
    args = parser.parse_args()
    rclpy.init()
    node = Node('mapping_transform_gate')
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    frames = ['left_camera_optical_frame']
    if args.lidar:
        frames.append('laser')
    deadline = time.monotonic() + 15
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if all(buffer.can_transform('stereo_camera_link', frame, Time())
                   for frame in frames):
                print('Mapping sensor transforms ready.', flush=True)
                return
        raise RuntimeError('Missing mapping sensor TF: ' + ', '.join(
            frame for frame in frames
            if not buffer.can_transform('stereo_camera_link', frame, Time())))
    finally:
        listener.unregister()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
