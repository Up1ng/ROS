"""Record a fresh, repeatable AutoRace trial on the course from my_robot."""

import argparse
import csv
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import cv2
from geometry_msgs.msg import Twist, Vector3Stamped
from lane_controller.image_codec import to_bgr
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
import yaml


def world_with_observer(source, dest):
    """Preserve the course and add an overhead camera for evidence."""
    tree = ET.parse(source)
    world = tree.getroot().find('world')
    # Use the same Sensors engine for the observer and the robot's cameras.
    world.append(ET.fromstring('''
      <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
        <render_engine>ogre2</render_engine>
      </plugin>'''))
    world.append(ET.fromstring('''
      <model name="autorace_observer">
        <static>true</static><pose>0 0 6.3 0 1.57079632679 0</pose>
        <link name="camera"><sensor name="overhead" type="camera">
          <topic>/autorace/overhead</topic><always_on>true</always_on><update_rate>8</update_rate>
          <camera><horizontal_fov>0.88</horizontal_fov>
            <image><width>512</width><height>512</height><format>R8G8B8</format></image>
            <clip><near>0.1</near><far>20</far></clip>
          </camera>
        </sensor></link>
      </model>'''))
    tree.write(dest, encoding='unicode')


def prepare(out, gains):
    bringup = Path(get_package_share_directory('robot_bringup'))
    desc = Path(get_package_share_directory('robot_description'))
    lane = Path(get_package_share_directory('lane_controller'))
    world_with_observer(bringup / 'worlds/course.sdf', out / 'course-observer.sdf')
    robot = ET.fromstring(subprocess.check_output(['xacro', desc / 'urdf/robot.urdf.xacro']))
    for extension in robot.findall('gazebo'):
        for plugin in extension.findall('plugin'):
            if plugin.get('name') == 'gz::sim::systems::Sensors':
                extension.remove(plugin)
    ET.ElementTree(robot).write(out / 'robot.urdf', encoding='unicode')
    bridge = yaml.safe_load((bringup / 'config/robot_bridge.yaml').read_text())
    # Only bridge the topics measured in this trial. Gazebo still uses the upstream model.
    bridge = [item for item in bridge if item['ros_topic_name'] in
              ('/clock', '/color/image', '/cmd_vel', '/odom', '/tf')]
    bridge.append({'ros_topic_name': '/autorace/overhead', 'gz_topic_name': '/autorace/overhead',
                   'ros_type_name': 'sensor_msgs/msg/Image', 'gz_type_name': 'gz.msgs.Image',
                   'direction': 'GZ_TO_ROS'})
    (out / 'bridge.yaml').write_text(yaml.safe_dump(bridge, sort_keys=False))
    config = yaml.safe_load((lane / 'config/lane.yaml').read_text())
    config['lane_controller']['ros__parameters'].update(gains)
    (out / 'parameters.yaml').write_text(yaml.safe_dump(config, sort_keys=False))
    return bringup


class Probe(Node):
    def __init__(self):
        super().__init__('autorace_trial', parameter_overrides=[
            Parameter('use_sim_time', value=True)])
        self.top = None
        self.raw = None
        self.debug = None
        self.odom = None
        self.rows = []
        self.last_command = None
        self.forward_at = None
        self.create_subscription(Image, '/autorace/overhead', self.on_top, qos_profile_sensor_data)
        self.create_subscription(Image, '/color/image', self.on_raw, qos_profile_sensor_data)
        self.create_subscription(Image, '/lane_controller/debug/image', self.on_debug, 1)
        self.create_subscription(Odometry, '/odom', lambda m: setattr(self, 'odom', m), 10)
        self.create_subscription(Vector3Stamped, '/lane_controller/telemetry', self.trace, 20)
        self.create_subscription(Twist, '/cmd_vel', self.on_command, 20)

    def on_top(self, msg):
        self.top = to_bgr(msg)

    def on_raw(self, msg):
        self.raw = to_bgr(msg)

    def on_debug(self, msg):
        self.debug = to_bgr(msg)

    def on_command(self, msg):
        self.last_command = time.monotonic(), msg.linear.x, msg.angular.z

    def trace(self, msg):
        if self.odom is None:
            return
        if self.forward_at is None:
            if msg.vector.z <= 0:
                return
            self.forward_at = stamp(msg)
        p = self.odom.pose.pose.position
        q = self.odom.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y**2 + q.z**2))
        self.rows.append((stamp(msg) - self.forward_at, msg.vector.x, msg.vector.y,
                          msg.vector.z, p.x, p.y, yaw))


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9


def spin_until(node, predicate, seconds):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
        if predicate():
            return True
    return False


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path)
    parser.add_argument('--x', type=float, default=1.15)
    parser.add_argument('--y', type=float, default=-2.2)
    parser.add_argument('--yaw', type=float, default=0.0)
    parser.add_argument('--seconds', type=float, default=12.0, help='simulation time')
    parser.add_argument('--kp', type=float, default=None)
    parser.add_argument('--ki', type=float, default=None)
    parser.add_argument('--kd', type=float, default=None)
    parser.add_argument('--speed', type=float, default=None)
    parser.add_argument('--capture-only', action='store_true')
    parser.add_argument('--rviz', action='store_true')
    parser.add_argument('--video', action='store_true')
    parser.add_argument('--camera-loss', action='store_true')
    parser.add_argument('--inspect', action='store_true')
    args = parser.parse_args()
    out = args.output.resolve()
    out.mkdir(parents=True)
    gains = {name: getattr(args, name) for name in ('kp', 'ki', 'kd', 'speed')
             if getattr(args, name) is not None}
    bringup = prepare(out, gains)
    processes = []

    def start(name, command):
        with (out / f'{name}.log').open('w') as stream:
            process = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT,
                                       start_new_session=True,
                                       env=dict(os.environ, PYTHONUNBUFFERED='1'))
        processes.append(process)
        return process

    def stop(process):
        if process.poll() is not None:
            return
        if process.args[:2] == ['ros2', 'launch']:
            process.send_signal(signal.SIGINT)
        else:
            os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=5)

    os.environ['GZ_SIM_RESOURCE_PATH'] = os.pathsep.join(str(p) for p in (
        bringup / 'worlds', os.getenv('GZ_SIM_RESOURCE_PATH', '')) if p)
    rclpy.init()
    node = Probe()
    video = None
    try:
        sim = start('gazebo', ['gz', 'sim', '-s', '-r', '--headless-rendering', '-v', '3',
                               out / 'course-observer.sdf'])
        bridge = start('bridge', ['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                                  '--ros-args', '-p', f'config_file:={out / "bridge.yaml"}'])
        assert spin_until(node, lambda: node.get_clock().now().nanoseconds > 0 or
                          sim.poll() is not None, 110), 'No /clock from Gazebo'
        assert sim.poll() is None, 'Gazebo exited'
        # Use the same fixed start pose on every comparison and wait before enabling control.
        with (out / 'spawn.log').open('w') as stream:
            subprocess.run(['ros2', 'run', 'ros_gz_sim', 'create', '-world', 'course',
                            '-name', 'robot', '-file', out / 'robot.urdf',
                            '-x', str(args.x), '-y', str(args.y), '-z', '0.08',
                            '-Y', str(args.yaw)], stdout=stream, stderr=subprocess.STDOUT,
                           timeout=35, check=True)
        ready = spin_until(node, lambda: all(x is not None for x in
                                            (node.raw, node.top, node.odom)), 110)
        assert ready, 'Missing camera, overhead, or odometry after spawn'
        cv2.imwrite(str(out / 'start-camera.png'), node.raw)
        cv2.imwrite(str(out / 'start-overhead.png'), node.top)
        if args.rviz:
            rviz = start('rviz', ['rviz2', '-d', Path(__file__).resolve().parents[2] /
                                  'labs/autorace/camera.rviz', '--ros-args', '-p',
                                  'use_sim_time:=true'])
            spin_until(node, lambda: False, 4)
            assert rviz.poll() is None
            subprocess.run(['import', '-window', 'root', str(out / 'rviz-camera.png')],
                           timeout=15, check=True)
            stop(rviz)
        if args.capture_only:
            return
        start('controller', ['ros2', 'launch', 'lane_controller', 'lane.launch.py',
                             f'config:={out / "parameters.yaml"}'])
        assert spin_until(node, lambda: node.forward_at is not None or node.debug is not None,
                          70), 'Controller did not receive frames'
        if node.debug is not None:
            cv2.imwrite(str(out / 'first-debug.png'), node.debug)
        assert spin_until(node, lambda: node.forward_at is not None, 15), 'No valid lane at spawn'
        if args.video:
            video = cv2.VideoWriter(str(out / 'trial.mp4'), cv2.VideoWriter_fourcc(*'mp4v'),
                                    8, (960, 360))
            assert video.isOpened(), 'Could not open video output'
        if args.inspect:
            for name, command in (
                ('topics', ['ros2', 'topic', 'list', '-t', '--no-daemon', '--spin-time', '2']),
                ('camera-info', ['ros2', 'topic', 'info', '/color/image', '-v', '--no-daemon', '--spin-time', '2']),
                ('cmd-info', ['ros2', 'topic', 'info', '/cmd_vel', '-v', '--no-daemon', '--spin-time', '2']),
                ('camera-hz', ['timeout', '10s', 'ros2', 'topic', 'hz', '/color/image']),
            ):
                start(name, command)
        next_capture = 0.0
        end = time.monotonic() + max(90, args.seconds * 25)
        sample_times = {1.0, args.seconds * 0.5, args.seconds * 0.9}
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
            elapsed = node.get_clock().now().nanoseconds / 1e9 - node.forward_at
            if node.debug is not None:
                for threshold in sorted(sample_times):
                    if elapsed >= threshold:
                        cv2.imwrite(str(out / f'debug-{threshold:.1f}s.png'), node.debug)
                        sample_times.remove(threshold)
                if video is not None and elapsed >= next_capture:
                    next_capture = elapsed + 0.125
                    video.write(np.hstack((cv2.resize(node.top, (360, 360)),
                                           cv2.resize(node.debug, (600, 360)))))
            if elapsed >= args.seconds:
                break
        else:
            raise RuntimeError('Trial did not finish in simulation time')
        if args.camera_loss:
            previous = node.last_command
            assert previous is not None and previous[1] > 0, 'Robot already stopped before loss'
            before = time.monotonic()
            # Pause Gazebo, leaving the ROS command subscriber and its steady watchdog active.
            paused = subprocess.run(['gz', 'service', '-s', '/world/course/control',
                                     '--reqtype', 'gz.msgs.WorldControl', '--reptype',
                                     'gz.msgs.Boolean', '--timeout', '3000', '--req',
                                     'pause: true'], capture_output=True, text=True,
                                    timeout=8, check=True)
            assert 'data: true' in paused.stdout, paused.stdout + paused.stderr
            assert spin_until(node, lambda: node.last_command is not None
                              and node.last_command[0] >= before and node.last_command[1:] == (0, 0),
                              5), 'Zero Twist was not published after camera stalled'
            (out / 'camera-loss.txt').write_text(
                f'Last forward Twist: {previous[1:]}\n'
                f'Zero Twist after Gazebo pause: {node.last_command[1:]}\n'
                f'Wall time from pause request: {node.last_command[0] - before:.3f} s\n')
        cv2.imwrite(str(out / 'end-overhead.png'), node.top)
    finally:
        if video is not None:
            video.release()
        with (out / 'trace.csv').open('w', newline='') as stream:
            writer = csv.writer(stream)
            writer.writerow(('time_s', 'error', 'angular_rad_s', 'linear_m_s', 'x_m', 'y_m', 'yaw'))
            writer.writerows(node.rows)
        for process in reversed(processes):
            stop(process)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
