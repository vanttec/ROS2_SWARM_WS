"""Plot each drone's position error |e| against time from a recorded bag.

    ros2 run swarm_formation plot_errors rosbag2_2026_11_07-12_00_00 [--out errors.png]

Time comes from the message stamps, which are simulated time when the run used use_sim_time, so
SITL plots read in the same seconds the drones flew in.
"""
import argparse
import math

import matplotlib.pyplot as plt
from geometry_msgs.msg import Vector3Stamped
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('bag', help='bag directory')
    parser.add_argument('--out', help='save to this file instead of opening a window')
    args = parser.parse_args()

    reader = SequentialReader()
    reader.open(StorageOptions(uri=args.bag, storage_id='sqlite3'), ConverterOptions('', ''))
    series, t0 = {}, None
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if not topic.endswith('/formation_error'):
            continue
        msg = deserialize_message(data, Vector3Stamped)
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        t0 = t if t0 is None else t0
        e = msg.vector
        times, norms = series.setdefault(topic.split('/')[1], ([], []))
        times.append(t - t0)
        norms.append(math.sqrt(e.x * e.x + e.y * e.y + e.z * e.z))
    if not series:
        raise SystemExit(f'no /droneN/formation_error messages in {args.bag}')

    for ns in sorted(series):
        plt.plot(*series[ns], label=ns)
    plt.xlabel('time [s]')
    plt.ylabel('|e| [m]')
    plt.title('Formation position error')
    plt.grid(True)
    plt.legend()
    if args.out:
        plt.savefig(args.out, dpi=150, bbox_inches='tight')
        print(f'saved {args.out}')
    else:
        plt.show()


if __name__ == '__main__':
    main()
