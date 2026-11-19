#!/usr/bin/env python3
"""Run after launching HW1: receive RGB images and move to one fixed point."""
import time

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo

import numpy as np
from cv_bridge import CvBridge
import cv2

from hw01_sim.arm_client import ArmClient


def find_marker(cvb: CvBridge, image: Image) -> tuple[float, float]:
    # FIXME: Account for partially occluded/nonexistant marker
    cvimage = cvb.imgmsg_to_cv2(image, desired_encoding='bgr8')
    lower_red = np.array([0, 0, 150])
    upper_red = np.array([60, 60, 255])

    mask = cv2.inRange(cvimage, lower_red, upper_red)

    M = cv2.moments(mask, binaryImage=True)
    if M["m00"] > 0:
        return (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

def image_to_3d(K, cix: int, ciy: int, cz: float) -> tuple[float, float]:
    # See 'report.md' for an explanation
    cx = cz * (cix - K[0][2]) / K[0][0]
    cy = cz * (ciy - K[1][2]) / K[1][1]

    return (cx, cy)

def main():
    rclpy.init()
    node = Node('hw01_check_setup')
    arm = ArmClient(node)
    cvb = CvBridge()

    image_received = None
    depth_received = None

    raw_image   = None
    depth_image = None
    camera_info = None

    # Camera data uses best-effort QoS. No image processing is needed for this check.
    def on_image(image: Image):
        nonlocal image_received
        nonlocal raw_image
        if image_received is None:
            print(f'Image: {image.width} x {image.height}, encoding={image.encoding}', flush=True)
        if image.width > 0 and image.height > 0 and len(image.data) >= image.step * image.height:
            image_received = time.monotonic()
            raw_image = image

    def on_depth(depth: Image):
        nonlocal depth_received
        nonlocal depth_image
        if depth_received is None:
            print(f'Depth: {depth.width} x {depth.height}, encoding={depth.encoding}', flush=True)
        if depth.width > 0 and depth.height > 0 and len(depth.data) >= depth.step * depth.height:
            depth_received = time.monotonic()
            depth_image = depth

    def on_info(info: CameraInfo):
        nonlocal camera_info
        if camera_info is None:
            print("Info received") # TODO: Print info parameters(K, D, etc)
        camera_info = info

    node.create_subscription(Image, '/camera/image_raw', on_image, qos_profile_sensor_data)
    node.create_subscription(Image, '/camera/depth/image_raw', on_depth, qos_profile_sensor_data)
    node.create_subscription(CameraInfo, '/camera/camera_info', on_info, qos_profile_sensor_data)

    motion = None
    cx = 0; cy = 0
    image_ok = False; depth_ok = False; motion_ok = False

    try:
        while rclpy.ok() and not (image_ok and depth_ok and camera_info is not None):
            rclpy.spin_once(node, timeout_sec=0.1)
            image_ok = image_received is not None and time.monotonic() - image_received < 2.0
            depth_ok = depth_received is not None and time.monotonic() - depth_received < 2.0

        if image_ok and depth_ok and camera_info is not None:
            cvdepth = cvb.imgmsg_to_cv2(depth_image, desired_encoding="passthrough")

            cix, ciy = find_marker(cvb, raw_image)
            cz = cvdepth[ciy][cix] # CV2 coordinate ordering is swapped for some reason

            K = np.reshape(np.array(camera_info.k), (3, 3))
            cx, cy = image_to_3d(K, cix, ciy, cz)

            print(f"{cx = }, {cy = }")
            # TODO: base_link transformation
        # TODO: motion based on `./check_setup.py`

        print(f'{"PASS" if image_ok else "FAIL"}: fresh camera image reception')
        print(f'{"PASS" if depth_ok else "FAIL"}: fresh camera depth reception')
        print(f'{"PASS" if motion_ok else "FAIL"}: fixed-point arm movement')

        if motion is not None and motion.done():
            print(motion.result().message)
        return 0 if image_ok and motion_ok else 1

    except KeyboardInterrupt:
        return 1

    finally:
        arm.cancel()
        for _ in range(5):
            if rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.05)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
