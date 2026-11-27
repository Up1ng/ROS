"""Bridge camera images to bounded Twist commands and visual feedback."""

from dataclasses import asdict
import math
import time

import cv2
from geometry_msgs.msg import Twist, Vector3Stamped
from lane_controller.detector import DetectorConfig, debug_panel, detect
from lane_controller.image_codec import from_bgr, to_bgr
from lane_controller.pid import PID, PidConfig
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


class LaneController(Node):
    def __init__(self):
        super().__init__('lane_controller')
        self.defaults = dict(asdict(DetectorConfig()), **asdict(PidConfig()),
                             speed=0.09, image_timeout=0.35, wall_timeout=1.1)
        for name, value in self.defaults.items():
            self.declare_parameter(name, list(value) if isinstance(value, tuple) else value)
        self.settings = {name: self.get_parameter(name).value for name in self.defaults}
        self.detector, pid_config = self.parse(self.settings)
        self.pid = PID(pid_config)
        self.camera = self.create_subscription(
            Image, 'color/image', self.on_image,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.command = self.create_publisher(Twist, 'cmd_vel', 1)
        self.debug = self.create_publisher(Image, '~/debug/image', 1)
        self.telemetry = self.create_publisher(Vector3Stamped, '~/telemetry', 10)
        self.last_received = None
        self.last_stamp = None
        self.last_clock = None
        self.last_status = ''
        self.wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.watchdog = self.create_timer(0.05, self.on_watchdog, clock=self.wall_clock)
        self.add_on_set_parameters_callback(self.validate_update)
        self.add_post_set_parameters_callback(self.apply_update)

    @staticmethod
    def parse(values):
        detector = DetectorConfig(**{
            key: tuple(values[key]) if isinstance(default, tuple) else values[key]
            for key, default in asdict(DetectorConfig()).items()
        })
        pid_config = PidConfig(**{key: values[key] for key in asdict(PidConfig())})
        detector.validate()
        pid_config.validate()
        for key in ('image_timeout', 'wall_timeout'):
            if not math.isfinite(values[key]) or values[key] <= 0:
                raise ValueError(f'{key} must be finite and positive')
        if not math.isfinite(values['speed']) or not 0 <= values['speed'] <= 0.25:
            raise ValueError('speed must be between 0 and 0.25 m/s')
        return detector, pid_config

    def candidate(self, parameters):
        return self.settings | {p.name: p.value for p in parameters if p.name in self.defaults}

    def validate_update(self, parameters):
        try:
            self.parse(self.candidate(parameters))
        except (TypeError, ValueError) as exc:
            return SetParametersResult(successful=False, reason=str(exc))
        return SetParametersResult(successful=True)

    def apply_update(self, parameters):
        candidate = self.candidate(parameters)
        if candidate == self.settings:
            return
        self.settings = candidate
        self.detector, pid_config = self.parse(candidate)
        self.pid = PID(pid_config)
        self.stop('parameters changed; waiting for a new frame')

    def simulation_time(self):
        now = self.get_clock().now().nanoseconds / 1e9
        if self.last_clock is not None and now < self.last_clock:
            self.last_stamp = None
            self.last_received = None
            self.stop('clock moved backwards')
        self.last_clock = now
        return now

    def publish(self, status, error=None, angular=0.0):
        twist = Twist()
        if error is not None:
            twist.linear.x = self.settings['speed']
            twist.angular.z = angular
        self.command.publish(twist)
        trace = Vector3Stamped()
        trace.header.stamp = self.get_clock().now().to_msg()
        trace.vector.x = float('nan') if error is None else error
        trace.vector.y = twist.angular.z
        trace.vector.z = twist.linear.x
        self.telemetry.publish(trace)
        if status != self.last_status:
            self.get_logger().info(status)
            self.last_status = status

    def stop(self, reason):
        self.pid.reset()
        self.publish(reason)

    def on_watchdog(self):
        now = self.simulation_time()
        if (self.last_received is None or self.last_stamp is None
                or time.monotonic() - self.last_received > self.settings['wall_timeout']
                or now - self.last_stamp > self.settings['image_timeout']):
            self.stop('camera timeout')

    def on_image(self, message):
        received = time.monotonic()
        now = self.simulation_time()
        stamp = message.header.stamp.sec + message.header.stamp.nanosec / 1e9
        if now <= 0 or stamp <= 0 or abs(now - stamp) > self.settings['image_timeout']:
            self.stop('invalid or stale image timestamp')
            return
        if self.last_stamp is not None and stamp <= self.last_stamp:
            self.stop('non-increasing image timestamp')
            return
        self.last_stamp = stamp
        self.last_received = received
        try:
            image = to_bgr(message)
            result = detect(image, self.detector)
        except (cv2.error, ValueError, TypeError) as exc:
            self.get_logger().warning(f'Invalid camera frame: {exc}', throttle_duration_sec=2.0)
            self.stop('invalid image')
            return
        if result.error is None:
            self.stop('lane lost')
            label = 'STOP: lane lost'
        else:
            angular = self.pid.update(result.error, stamp)
            self.publish('tracking lane', result.error, angular)
            label = f'e={result.error:+.3f}  w={angular:+.3f} rad/s'
        if self.debug.get_subscription_count():
            preview = debug_panel(image, result, self.detector, label)
            self.debug.publish(from_bgr(preview, message.header))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = LaneController()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            if rclpy.ok():
                node.stop('controller shutdown')
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
