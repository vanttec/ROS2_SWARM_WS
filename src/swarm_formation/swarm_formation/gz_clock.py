"""Relay Gazebo Harmonic's simulation clock onto ROS /clock, for use_sim_time in SITL.

    ros2 run swarm_formation gz_clock

ROS 2 Humble's ros_gz_bridge is built against Gazebo Fortress (ignition-transport11) and cannot
see Harmonic's topics, so this does the one thing the experiment needs from it using Harmonic's
own Python bindings (python3-gz-transport13, installed with gz-harmonic).
"""
import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock

from gz.msgs10.clock_pb2 import Clock as GzClock
from gz.transport13 import Node as GzNode


class GzClockRelay(Node):
    def __init__(self):
        super().__init__('gz_clock')
        # Gazebo ticks every 1 ms; every 5 ms is plenty for 20 Hz loops.
        self.period = self.declare_parameter('period', 0.005).value
        self.pub = self.create_publisher(Clock, '/clock', 10)
        self.last = None
        self.gz = GzNode()
        if not self.gz.subscribe(GzClock, '/clock', self.on_clock):
            raise RuntimeError('could not subscribe to Gazebo /clock')

    def on_clock(self, msg):
        t = msg.sim.sec + msg.sim.nsec * 1e-9
        # A backwards jump (Gazebo restarted) passes straight through.
        if self.last is not None and 0.0 <= t - self.last < self.period:
            return
        self.last = t
        out = Clock()
        out.clock.sec, out.clock.nanosec = msg.sim.sec, msg.sim.nsec
        self.pub.publish(out)


def main():
    rclpy.init()
    rclpy.spin(GzClockRelay())


if __name__ == '__main__':
    main()
