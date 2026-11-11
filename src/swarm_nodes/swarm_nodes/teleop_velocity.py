"""Keyboard velocity teleop for one drone, for bench and SITL testing.

Publishes a Twist to /<drone_id>/cmd_vel while a key is held -- the same topic the formation
controller uses, so this drives the real velocity interface rather than a separate path.

    w / s   +x / -x        a / d   -y / +y        q / e   -z / +z
    left / right arrow     yaw rate              Ctrl-C  quit

REQUIRES command_bridge to be running. The bridge is what clamps these speeds, forwards them to
MAVROS and holds position if you stop pressing keys. Without it, nothing reaches the drone.

AXES ARE WORLD ENU, NOT BODY. +x is EAST and +y is NORTH -- "w" does not mean "forward" and does
not depend on where the drone is pointing.

Run it in SITL first, and on a real aircraft only with an RC safety pilot on the sticks and the
geofence enabled.
"""
import select
import sys
import termios
import time
import tty

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from geometry_msgs.msg import Twist

INPUT_POLL_HZ = 100.0


class TeleopVelocity(Node):
    def __init__(self):
        super().__init__('teleop_velocity')
        drone_id = self.declare_parameter('drone_id', '').value
        self.speed = self.declare_parameter('speed', 0.5).value
        self.yaw_rate = self.declare_parameter('yaw_rate', 0.5).value
        rate_hz = self.declare_parameter('rate_hz', 20.0).value
        self.hold_timeout = self.declare_parameter('hold_timeout', 0.5).value

        if not drone_id:
            raise ValueError(
                'drone_id is required, e.g. --ros-args -p drone_id:=drone1')
        drone_id = '/' + drone_id.strip('/')

        self.msg = Twist()
        self.last_key = 0.0

        topic = f'{drone_id}/cmd_vel'
        self.publisher = self.create_publisher(
            Twist, topic, qos_profile_system_default)

        self.create_timer(1.0 / INPUT_POLL_HZ, self.read_key)
        self.create_timer(1.0 / rate_hz, self.publish_command)

        self.get_logger().info(
            f'{topic} at {rate_hz:.0f} Hz, {self.speed} m/s, {self.yaw_rate} rad/s')
        self.get_logger().warn(
            'axes are world ENU: w = EAST, a/d = north/south, NOT body-relative. '
            'Requires command_bridge running -- it clamps these speeds and holds position '
            'when you stop pressing keys.')

    def stop(self):
        """Zero every axis -- what we send when no key is being held."""
        self.msg.linear.x = 0.0
        self.msg.linear.y = 0.0
        self.msg.linear.z = 0.0
        self.msg.angular.z = 0.0

    def read_key(self):
        readable, _, _ = select.select([sys.stdin], [], [], 0)
        if not readable:
            # Terminal key-repeat is far slower than this timer, so hold the last command briefly
            # instead of zeroing between repeats, which would make a held key stutter.
            if time.monotonic() - self.last_key > self.hold_timeout:
                self.stop()
            return

        key = sys.stdin.read(1)
        if key == '\x1b':  # escape sequence: arrow keys arrive as ESC [ A-D
            key = (key + sys.stdin.read(2))[-1]
        if key == '\x03':  # Ctrl-C, delivered as a byte because the terminal is raw
            raise KeyboardInterrupt

        self.last_key = time.monotonic()
        self.stop()
        match key:
            case 'w':
                self.msg.linear.x = self.speed
            case 's':
                self.msg.linear.x = -self.speed
            case 'd':
                self.msg.linear.y = self.speed
            case 'a':
                self.msg.linear.y = -self.speed
            case 'e':
                self.msg.linear.z = self.speed
            case 'q':
                self.msg.linear.z = -self.speed
            case 'C':
                self.msg.angular.z = -self.yaw_rate
            case 'D':
                self.msg.angular.z = self.yaw_rate

    def publish_command(self):
        # Sends 20 times a second no matter what, including zeros when no key is down. That is
        # deliberate: a steady stream of zeros tells the bridge "still here, just not moving", so
        # letting go of the keys makes the drone stop rather than trip the watchdog.
        self.publisher.publish(self.msg)


def main():
    if not sys.stdin.isatty():
        print('teleop_velocity needs a terminal to read keys from', file=sys.stderr)
        return

    rclpy.init()
    node = None
    fd = sys.stdin.fileno()
    settings = termios.tcgetattr(fd)
    try:
        node = TeleopVelocity()
        tty.setraw(fd)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ValueError as err:
        print(f'teleop_velocity: {err}', file=sys.stderr)
    finally:
        # Runs on every exit path, including Ctrl-C, so the shell is never left in raw mode.
        termios.tcsetattr(fd, termios.TCSADRAIN, settings)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
