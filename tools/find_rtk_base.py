#!/usr/bin/env python3
"""Find the RTK base's serial port and say what it is actually emitting.

    ./tools/find_rtk_base.py

Answers the two questions you have when you plug the base in:

    1. which /dev/tty* is it?
    2. is it already surveying in and sending RTCM3, or does it need configuring?

Deliberately standalone -- no ROS, no built workspace needed, so it works before anything else
does. It only reads from the port; it never writes, so it cannot disturb the receiver.

What the protocol tells you:

    RTCM3  the base is configured and emitting corrections. If message 1005 or 1006 is in the
           list, the survey-in has FINISHED and it knows its own position -- you are ready.
           RTCM3 without 1005 means it is still surveying; wait and look again.
    UBX    u-blox binary. The receiver works but is in its default rover mode, so it needs
           configuring as a base before it will emit corrections.
    NMEA   plain text position sentences. Same conclusion as UBX.
"""
import argparse
import glob
import subprocess
import sys
import time

try:
    import serial
except ImportError:
    sys.exit('pyserial is missing:  pip3 install --user pyserial')


def crc24q(data):
    """The checksum RTCM3 frames carry. Duplicated from swarm_nodes/rtcm_relay.py on purpose, so
    this tool runs without a built workspace."""
    crc = 0
    for byte in data:
        crc ^= byte << 16
        for _ in range(8):
            crc <<= 1
            if crc & 0x1000000:
                crc ^= 0x1864CFB
    return crc & 0xFFFFFF


def describe(port):
    """Vendor and model from udev, so a u-blox is recognisable among several serial devices."""
    try:
        out = subprocess.run(['udevadm', 'info', '-q', 'property', '-n', port],
                             capture_output=True, text=True, timeout=5).stdout
    except (OSError, subprocess.SubprocessError):
        return ''
    fields = dict(line.split('=', 1) for line in out.splitlines() if '=' in line)
    bits = [fields.get('ID_VENDOR_FROM_DATABASE') or fields.get('ID_VENDOR', ''),
            fields.get('ID_MODEL_FROM_DATABASE') or fields.get('ID_MODEL', '')]
    return ' '.join(b.replace('_', ' ') for b in bits if b).strip()


def rtcm_types(buf):
    """Pull the RTCM message numbers out of a byte sample, checking each frame's CRC."""
    found, i = {}, 0
    while i < len(buf) - 6:
        if buf[i] != 0xD3:
            i += 1
            continue
        length = ((buf[i + 1] & 0x03) << 8) | buf[i + 2]
        end = i + 3 + length + 3
        if end > len(buf):
            break
        frame = buf[i:end]
        if crc24q(frame[:-3]) == int.from_bytes(frame[-3:], 'big'):
            number = (frame[3] << 4) | (frame[4] >> 4)
            found[number] = found.get(number, 0) + 1
            i = end
        else:
            i += 1          # false preamble, resync
    return found


def sniff(port, baud, seconds):
    try:
        with serial.Serial(port, baud, timeout=0.2) as handle:
            buf = bytearray()
            end = time.time() + seconds
            while time.time() < end:
                buf.extend(handle.read(4096))
            return bytes(buf), None
    except serial.SerialException as err:
        hint = ''
        if 'Permission denied' in str(err):
            hint = ("  -> you are not in the 'dialout' group in THIS shell. "
                    "Run 'newgrp dialout', or leave and re-enter the container.")
        return b'', f'{err}{hint}'
    except OSError as err:
        return b'', str(err)


def classify(data):
    if not data:
        return 'nothing', {}
    types = rtcm_types(data)
    if types:
        return 'RTCM3', types
    if b'\xb5\x62' in data:
        return 'UBX', {}
    if b'$G' in data:
        return 'NMEA', {}
    return 'unknown', {}


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--port', help='check only this port instead of searching')
    ap.add_argument('--baud', type=int, default=115200,
                    help='ignored for USB (CDC-ACM), matters for a UART adapter')
    ap.add_argument('--seconds', type=float, default=6.0, help='how long to listen')
    args = ap.parse_args()

    ports = [args.port] if args.port else sorted(
        glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*'))
    if not ports:
        print('No serial devices found.\n'
              '  - is the base plugged in?\n'
              '  - inside a container, the device must exist there too: ls /dev/ttyACM*')
        return 1

    print(f'Listening to {len(ports)} port(s) for {args.seconds:.0f}s each...\n')
    verdicts = []
    for port in ports:
        label = describe(port)
        print(f'{port}{"  (" + label + ")" if label else ""}')
        data, err = sniff(port, args.baud, args.seconds)
        if err:
            print(f'  could not read: {err}\n')
            continue
        proto, types = classify(data)
        print(f'  {len(data)} bytes, looks like {proto}')
        if proto == 'RTCM3':
            listed = ', '.join(str(n) for n in sorted(types))
            print(f'  RTCM messages: {listed}')
            if 1005 in types or 1006 in types:
                print('  SURVEY-IN COMPLETE -- the base knows its own position. Ready to use:')
                print(f'    ros2 run swarm_nodes rtcm_relay --ros-args -p port:={port}')
                verdicts.append((port, 'ready'))
            else:
                print('  Still surveying in (no 1005/1006 yet). Leave it and check again in a '
                      'few minutes.')
                verdicts.append((port, 'surveying'))
        elif proto in ('UBX', 'NMEA'):
            print(f'  Receiver is alive but in rover mode -- it is not emitting corrections.')
            print('  It needs configuring as a base (survey-in + RTCM3 output) before use.')
            verdicts.append((port, 'needs config'))
        elif proto == 'nothing':
            print('  Silent. Wrong port, wrong baud for a UART adapter, or not powered.')
        else:
            print('  Unrecognised data.')
        print()

    ready = [p for p, v in verdicts if v == 'ready']
    if ready:
        print(f'Use: {ready[0]}')
    elif verdicts:
        print('No port is ready yet -- see the notes above.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
