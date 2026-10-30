"""Read RTCM corrections from the local RTK base and fan them out to every drone.

This is what turns 1-3 m GPS into roughly 1 cm. The surveyed-in base station measures how wrong
the satellites look from a known point and emits corrections; every drone that applies them gets
the same error cancelled out. Without this the drones sit at fix type 3 (plain 3D) no matter how
good the receivers are, and "RTK-grade pose" is not true.

    base F9P on USB  ->  this node  ->  /droneN/mavros/gps_rtk/send_rtcm  ->  MAVLink  ->  drone

MAVROS's gps_rtk plugin fragments each message into GPS_RTCM_DATA and sends it over the existing
telemetry link, so no extra radio is needed.

Corrections can come from either of two places:

    # straight off the base's USB serial port -- the base must be configured to survey in and
    # emit RTCM3 by itself
    ros2 run swarm_nodes rtcm_relay --ros-args -p port:=/dev/ttyACM0

    # or over UDP from QGroundControl (NTRIP/RTK -> UDP Forwarding). Use this when QGC is doing
    # the survey-in, because a serial port has only one owner and QGC will be holding it.
    ros2 run swarm_nodes rtcm_relay --ros-args -p udp_port:=13320

The base must already be surveyed in and configured to emit RTCM3. The periodic report lists which
RTCM message numbers are arriving, which is how you confirm that: 1005 or 1006 is the base
position, 1074/1077/1084/1087/1094/1097 are the satellite observations, 1230 is GLONASS biases. If
1005 never appears, the base has not finished surveying in.

Proof it worked is on the drone, not here -- fix_type 6 (RTK Fixed) on
/droneN/mavros/gpsstatus/gps1/raw. Fix type 5 means corrections are arriving but have not
converged yet; give it a minute.
"""
import socket

import serial

import rclpy
from rclpy.node import Node
from mavros_msgs.msg import RTCM

PREAMBLE = 0xD3
# Longest possible frame is 3 + 1023 + 3 bytes. Anything beyond a few of those means we are
# looking at noise, not RTCM, so the buffer gets dropped rather than grown for ever.
MAX_BUFFER = 4096
MAX_PAYLOAD = 720          # mavros_msgs/RTCM: "data should be <= 4*180, higher will be discarded"


def crc24q(data):
    """Work out the checksum that every RTCM frame carries.

    A CRC is a number calculated from a block of data and sent alongside it. We recalculate it
    from the bytes that actually arrived; if the two differ, something got corrupted on the wire.
    Like totting up a list of numbers and sending the total too, but built so that changing any
    single bit changes the answer completely.

    We use it for two things: dropping damaged corrections instead of pushing noise to four
    aircraft, and working out where frames actually start -- see parse() below.

    This matches ArduPilot's own crc_crc24 (poly 0x1864CFB, starting from 0), checked across 505
    inputs. That matters because ArduPilot is what validates these frames at the far end.
    """
    crc = 0
    for byte in data:
        crc ^= byte << 16
        for _ in range(8):
            crc <<= 1
            if crc & 0x1000000:
                crc ^= 0x1864CFB
    return crc & 0xFFFFFF


class RtcmRelay(Node):
    def __init__(self):
        super().__init__('rtcm_relay')
        self.drone_ns = self.declare_parameter(
            'namespaces', ['drone1', 'drone2', 'drone3', 'drone4']).value
        self.port_name = self.declare_parameter('port', '/dev/ttyACM0').value
        self.baudrate = self.declare_parameter('baudrate', 115200).value
        # Set udp_port to take corrections from QGroundControl's UDP Forwarding instead of the
        # serial port. Non-zero means UDP, and the serial port is left alone entirely.
        self.udp_port = self.declare_parameter('udp_port', 0).value
        report_period = self.declare_parameter('report_period', 10.0).value
        self.reconnect_period = self.declare_parameter('reconnect_period', 5.0).value

        self.pubs = {ns: self.create_publisher(RTCM, f'/{ns}/mavros/gps_rtk/send_rtcm', 10)
                     for ns in self.drone_ns}

        self.port = None
        self.buffer = bytearray()
        self.frames = 0
        self.bytes_out = 0
        self.rejected = 0
        self.types = {}
        self.next_retry = 0.0

        self.sock = None
        self.create_timer(0.02, self.poll)
        self.create_timer(report_period, self.report)
        self.source = (f'UDP :{self.udp_port}' if self.udp_port else self.port_name)
        self.get_logger().info(
            f'relaying RTCM from {self.source} to {", ".join(self.drone_ns)}')
        if self.udp_port:
            self.open_socket()
        else:
            self.open_port()

    def open_socket(self):
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.sock.bind(('0.0.0.0', int(self.udp_port)))
            self.sock.setblocking(False)
            self.get_logger().info(
                f'listening for RTCM on UDP {self.udp_port} '
                f'(point QGroundControl\'s NTRIP/RTK -> UDP Forwarding here)')
        except OSError as err:
            self.sock = None
            self.get_logger().error(f'cannot bind UDP {self.udp_port}: {err}')

    def poll_udp(self):
        if self.sock is None:
            return
        for _ in range(64):               # drain whatever arrived since the last tick
            try:
                data, _addr = self.sock.recvfrom(4096)
            except BlockingIOError:
                break
            except OSError as err:
                self.get_logger().error(f'UDP read failed: {err}',
                                        throttle_duration_sec=10.0)
                break
            self.buffer.extend(data)
        if len(self.buffer) > MAX_BUFFER:
            self.get_logger().warn(
                f'{len(self.buffer)} B of unparsable data on UDP {self.udp_port}, discarding',
                throttle_duration_sec=15.0)
            self.buffer.clear()
        self.parse()

    # -- serial ----------------------------------------------------------------

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def open_port(self):
        try:
            self.port = serial.Serial(self.port_name, self.baudrate, timeout=0)
            self.get_logger().info(f'{self.port_name} open')
        except (serial.SerialException, OSError) as err:
            self.port = None
            self.next_retry = self.now() + self.reconnect_period
            self.get_logger().error(
                f'cannot open {self.port_name}: {err} -- is the RTK base plugged in?',
                throttle_duration_sec=10.0)

    def poll(self):
        if self.udp_port:
            self.poll_udp()
            return
        if self.port is None:
            if self.now() >= self.next_retry:
                self.open_port()
            return
        try:
            waiting = self.port.in_waiting
            if waiting:
                self.buffer.extend(self.port.read(waiting))
            if len(self.buffer) > MAX_BUFFER:
                self.get_logger().warn(
                    f'{len(self.buffer)} B of unparsable data from the base, discarding -- is it '
                    f'set to emit RTCM3?', throttle_duration_sec=15.0)
                self.buffer.clear()
        except (serial.SerialException, OSError) as err:
            self.get_logger().error(f'{self.port_name} read failed: {err}, reconnecting')
            try:
                self.port.close()
            except Exception:                                   # noqa: BLE001
                pass
            self.port = None
            self.next_retry = self.now() + self.reconnect_period
            return
        self.parse()

    # -- RTCM3 framing ---------------------------------------------------------

    def parse(self):
        """Pull whole, verified RTCM frames out of the stream of bytes arriving from the base.

        A frame looks like:
            0xD3  |  2 bytes holding the payload length  |  payload  |  3-byte checksum

        The catch: 0xD3 is just an ordinary byte value, so it can also appear by coincidence in
        the middle of someone's data. The checksum is what settles it -- if it matches, that
        really was the start of a frame; if not, we step forward one byte and keep looking.
        """
        while True:
            start = self.buffer.find(PREAMBLE)
            if start < 0:
                self.buffer.clear()
                return
            if start:
                del self.buffer[:start]          # drop junk before the preamble
            if len(self.buffer) < 3:
                return                           # need the length field
            # The length lives in the bottom 10 bits of the next two bytes.
            length = ((self.buffer[1] & 0x03) << 8) | self.buffer[2]
            total = 3 + length + 3
            if len(self.buffer) < total:
                return                           # frame still arriving

            frame = bytes(self.buffer[:total])
            want = int.from_bytes(frame[-3:], 'big')
            if crc24q(frame[:-3]) != want:
                # Either this 0xD3 was not really a frame start, or the data got damaged.
                # Either way: throw away one byte and start hunting again from the next one.
                self.rejected += 1
                del self.buffer[:1]
                continue

            del self.buffer[:total]
            self.emit(frame, length)

    def emit(self, frame, length):
        if length >= 2:
            number = (frame[3] << 4) | (frame[4] >> 4)
            self.types[number] = self.types.get(number, 0) + 1
        if len(frame) > MAX_PAYLOAD:
            self.get_logger().warn(
                f'RTCM frame of {len(frame)} B exceeds the {MAX_PAYLOAD} B limit, dropped',
                throttle_duration_sec=10.0)
            return

        msg = RTCM()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.data = list(frame)
        for pub in self.pubs.values():
            pub.publish(msg)
        self.frames += 1
        self.bytes_out += len(frame)

    def report(self):
        if self.port is None and self.sock is None:
            return
        if not self.frames:
            hint = (f'is QGroundControl forwarding to UDP {self.udp_port}?' if self.udp_port
                    else 'check it is powered, surveyed in and set to emit RTCM3')
            self.get_logger().warn(
                f'no RTCM frames from {self.source} -- {hint}', throttle_duration_sec=30.0)
            return
        seen = ', '.join(str(n) for n in sorted(self.types))
        self.get_logger().info(
            f'{self.frames} frames from {self.source}, {self.bytes_out / 1024:.1f} kB '
            f'to {len(self.drone_ns)} drones '
            f'| types: {seen}'
            + (f' | {self.rejected} rejected' if self.rejected else ''))
        if 1005 not in self.types and 1006 not in self.types:
            self.get_logger().warn(
                'no base-position message (1005/1006) yet -- the base may still be surveying in')
        self.frames = self.bytes_out = self.rejected = 0
        self.types.clear()


def main():
    rclpy.init()
    rclpy.spin(RtcmRelay())


if __name__ == '__main__':
    main()
