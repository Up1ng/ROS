"""Exercise the real ROS node with instrumented images and a simulated clock."""

import math
import subprocess
import time

import cv2
from geometry_msgs.msg import Twist
from lane_controller.image_codec import from_bgr
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image


def main():
    rclpy.init()
    node = Node('autorace_stop_probe')
    frames = node.create_publisher(Image, '/color/image', qos_profile_sensor_data)
    clock = node.create_publisher(Clock, '/clock', qos_profile_sensor_data)
    commands = []
    node.create_subscription(Twist, '/cmd_vel', lambda msg: commands.append(
        (msg.linear.x, msg.angular.z)), 10)
    good = np.full((300, 400, 3), 40, np.uint8)
    cv2.line(good, (80, 130), (80, 299), (0, 220, 255), 9)
    cv2.line(good, (280, 130), (280, 299), (250, 250, 250), 9)
    proc = subprocess.Popen(['ros2', 'launch', 'lane_controller', 'lane.launch.py'])
    tick = 20.0

    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.01)

    def send(image, age=0.0):
        nonlocal tick
        for _ in range(5):
            tick += 0.05
            msg = Clock()
            msg.clock.sec = math.floor(tick)
            msg.clock.nanosec = round((tick % 1) * 1e9)
            clock.publish(msg)
            spin(0.04)
            frame = from_bgr(image)
            frame.header.stamp.sec = math.floor(tick - age)
            frame.header.stamp.nanosec = round(((tick - age) % 1) * 1e9)
            frames.publish(frame)
            spin(0.04)
        assert commands, 'Controller did not issue a command'
        return commands[-1]

    try:
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline and not frames.get_subscription_count():
            spin(0.1)
        assert frames.get_subscription_count() == 1
        spin(0.3)
        assert commands[-1] == (0.0, 0.0)
        forward = send(good)
        assert forward[0] > 0 and forward[1] > 0, forward
        print('Left target: positive angular.z', forward, flush=True)
        assert send(np.zeros_like(good)) == (0, 0)
        print('Both borders gone: zero Twist', flush=True)
        assert send(good)[0] > 0
        assert send(good, age=1) == (0, 0)
        print('Old but valid image: zero Twist', flush=True)
        assert send(good)[0] > 0
        spin(1.4)
        assert commands[-1] == (0, 0)
        print('Clock and camera paused: zero Twist', flush=True)
    finally:
        proc.send_signal(2)
        proc.wait(timeout=10)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
