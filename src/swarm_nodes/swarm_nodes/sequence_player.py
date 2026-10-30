"""Fly a scripted Twist sequence -- the acceptance test for the velocity interface.

Deliverable 5 is done when "each drone, in GUIDED, follows a scripted Twist sequence in SITL and
outdoors". This plays that sequence: a list of timed velocity steps published to /droneN/cmd_vel,
so it goes through command_bridge and inherits the clamp and the watchdog like everything else.

    ros2 run swarm_nodes sequence_player --ros-args -p namespaces:="['drone1']"
    ros2 run swarm_nodes sequence_player --ros-args -p sequence_file:=/path/to/square.yaml

The built-in default is a 10 m square at 0.5 m/s, returning to where it started, which makes the
drift obvious: if it does not come back to its starting point, something is wrong with the frame
or the velocity scaling.

A sequence file is a YAML list of steps, each with a duration and the velocity to hold:

    - {duration: 4.0, x: 0.5}       # 2 m east
    - {duration: 4.0, y: 0.5}       # 2 m north
    - {duration: 4.0, x: -0.5}
    - {duration: 4.0, y: -0.5}
    - {duration: 2.0}               # all zero: hold

x, y, z are m/s in world ENU, yaw is rad/s. Omitted axes are zero.

Ctrl-C stops the sequence and holds zero velocity. It does NOT land -- first_flight owns landing,
and the bridge's watchdog takes over if this node dies outright.
"""
import signal
import time

import rclpy
import yaml
from rclpy.node import Node
from geometry_msgs.msg import Twist

# A 10 m square at 0.5 m/s that ends where it began. Chosen because it makes errors obvious:
# if the drone does not come back to its starting point, something is wrong with the frame or
# with how velocities are being scaled.
DEFAULT_SEQUENCE = [
    {'duration': 2.0},
    {'duration': 12.0, 'x': 0.25},
    {'duration': 12.0, 'y': 0.25},
    {'duration': 12.0, 'x': -0.25},
    {'duration': 12.0, 'y': -0.25},
    {'duration': 3.0},
]

_interrupted = False


def _on_signal(signum, frame):
    global _interrupted
    _interrupted = True


class SequencePlayer(Node):
    def __init__(self):
        super().__init__('sequence_player')
        self.drone_ns = self.declare_parameter('namespaces', ['drone1']).value
        rate_hz = self.declare_parameter('rate_hz', 20.0).value
        path = self.declare_parameter('sequence_file', '').value
        self.period = 1.0 / rate_hz

        self.steps = DEFAULT_SEQUENCE
        if path:
            with open(path) as handle:
                self.steps = yaml.safe_load(handle)
            self.get_logger().info(f'loaded {len(self.steps)} steps from {path}')

        self.pubs = {ns: self.create_publisher(Twist, f'/{ns}/cmd_vel', 10)
                     for ns in self.drone_ns}

        total = sum(float(s.get('duration', 0.0)) for s in self.steps)
        self.get_logger().info(
            f'{len(self.steps)} steps, {total:.0f}s total, on {", ".join(self.drone_ns)} '
            f'at {rate_hz:.0f} Hz -- requires command_bridge and an airborne drone')

    def publish(self, step):
        msg = Twist()
        msg.linear.x = float(step.get('x', 0.0))
        msg.linear.y = float(step.get('y', 0.0))
        msg.linear.z = float(step.get('z', 0.0))
        msg.angular.z = float(step.get('yaw', 0.0))
        for pub in self.pubs.values():
            pub.publish(msg)

    def run(self):
        for index, step in enumerate(self.steps, 1):
            if _interrupted or not rclpy.ok():
                break
            duration = float(step.get('duration', 0.0))
            described = ', '.join(f'{k}={step[k]}' for k in ('x', 'y', 'z', 'yaw') if k in step)
            self.get_logger().info(
                f'step {index}/{len(self.steps)}: {duration:.1f}s  {described or "hold"}')

            # Hold this velocity for 'duration' seconds, re-sending it 20 times a second.
            # Re-sending matters: the bridge treats a command older than half a second as stale.
            end = time.monotonic() + duration
            while time.monotonic() < end and rclpy.ok() and not _interrupted:
                self.publish(step)
                rclpy.spin_once(self, timeout_sec=self.period)

        # Finish on zero so the drone stops rather than drifting off on the final velocity.
        self.publish({})
        if _interrupted:
            self.get_logger().warn('interrupted: holding zero velocity')
        else:
            self.get_logger().info('sequence complete: holding zero velocity')


def main():
    rclpy.init()
    signal.signal(signal.SIGINT, _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)
    node = SequencePlayer()
    try:
        node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
