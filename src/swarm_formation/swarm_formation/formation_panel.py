"""Formation experiment panel: drive the virtual leader, pick the shape, and run each drone
through GUIDED -> arm -> takeoff -> controller -> land.

Leader buttons and keys (arrows, PgUp/PgDn, Q/E, Space to stop) send a velocity while held, at
10 Hz; releasing sends zero. Nothing happens without a button press; only ARM asks to confirm.
FOLLOW MOCHILA hands x, y and yaw to the backpack; MANUAL, STOP or Space takes them back.
"""
import math
import signal
import sys
import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, qos_profile_sensor_data
from geometry_msgs.msg import Twist, Vector3Stamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import SetParametersAtomically
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger

from PyQt5.QtCore import QEvent, Qt, QTimer
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import (QApplication, QComboBox, QDoubleSpinBox, QGridLayout, QHBoxLayout,
                             QLabel, QMessageBox, QPushButton, QVBoxLayout, QWidget)

OK, WARN, BAD, DIM = '#2e7d32', '#ef6c00', '#c62828', '#757575'
STALE_AFTER = 2.0
ALL = 'ALL'
# Leader axes as (vx, vy, vz, w) unit directions, scaled by the panel's speeds.
NORTH, SOUTH, WEST, EAST = (0, 1, 0, 0), (0, -1, 0, 0), (-1, 0, 0, 0), (1, 0, 0, 0)
UP, DOWN, YAW_LEFT, YAW_RIGHT = (0, 0, 1, 0), (0, 0, -1, 0), (0, 0, 0, 1), (0, 0, 0, -1)
KEYS = {Qt.Key_Up: NORTH, Qt.Key_Down: SOUTH, Qt.Key_Left: WEST, Qt.Key_Right: EAST,
        Qt.Key_PageUp: UP, Qt.Key_PageDown: DOWN, Qt.Key_Q: YAW_LEFT, Qt.Key_E: YAW_RIGHT}


class PanelNode(Node):
    """Owns every subscription and service client; the window only reads snapshots."""

    def __init__(self):
        super().__init__('formation_panel')
        self.drone_ns = self.declare_parameter(
            'namespaces', ['drone1', 'drone2', 'drone3', 'drone4']).value
        self.leader_speed = self.declare_parameter('leader_speed', 0.3).value
        self.leader_yaw_rate = self.declare_parameter('leader_yaw_rate', 0.15).value
        self.lock = threading.Lock()
        self.data = {ns: {'state': None, 'state_t': None, 'error': None, 'error_t': None,
                          'active': False} for ns in self.drone_ns}
        self.message = 'ready'
        self.leader, self.leader_t = None, None
        self.source = None

        latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.cli = {}
        for ns in self.drone_ns:
            self.create_subscription(State, f'/{ns}/mavros/state',
                                     lambda m, ns=ns: self.put(ns, 'state', m),
                                     qos_profile_sensor_data)
            self.create_subscription(Vector3Stamped, f'/{ns}/formation_error',
                                     lambda m, ns=ns: self.put(ns, 'error', m.vector), 10)
            self.create_subscription(Bool, f'/{ns}/formation_controller/active',
                                     lambda m, ns=ns: self.put(ns, 'active', m.data), latched)
            self.cli[ns] = {
                'mode': self.create_client(SetMode, f'/{ns}/mavros/set_mode'),
                'arm': self.create_client(CommandBool, f'/{ns}/mavros/cmd/arming'),
                'takeoff': self.create_client(CommandTOL, f'/{ns}/mavros/cmd/takeoff'),
                'land': self.create_client(CommandTOL, f'/{ns}/mavros/cmd/land'),
                'enable': self.create_client(SetBool, f'/{ns}/formation_controller/enable'),
            }
        self.create_subscription(Odometry, '/formation/leader', self.on_leader,
                                 qos_profile_sensor_data)
        self.leader_pub = self.create_publisher(Twist, '/formation/leader_cmd', 10)
        self.create_subscription(String, '/formation/leader_source', self.on_source, latched)
        self.follow_cli = self.create_client(SetBool, '/formation_leader/follow')
        self.reset_cli = self.create_client(Trigger, '/formation_leader/reset_to_swarm')
        self.param_cli = self.create_client(SetParametersAtomically,
                                            '/formation_leader/set_parameters_atomically')

    def put(self, ns, key, value):
        # ROS callbacks run on a background thread, Qt on the main one.
        with self.lock:
            self.data[ns][key] = value
            if key in ('state', 'error'):
                self.data[ns][key + '_t'] = time.monotonic()

    def on_leader(self, msg):
        p, q, v = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist
        with self.lock:
            self.leader = (p.x, p.y, p.z, math.degrees(2.0 * math.atan2(q.z, q.w)),
                           math.hypot(v.linear.x, v.linear.y), v.linear.z, v.angular.z)
            self.leader_t = time.monotonic()

    def on_source(self, msg):
        with self.lock:
            self.source = msg.data

    def snapshot(self):
        with self.lock:
            return ({ns: dict(d) for ns, d in self.data.items()}, self.message,
                    self.leader, self.leader_t, self.source)

    def report(self, text):
        with self.lock:
            self.message = text
        self.get_logger().info(text)

    def call(self, client, request, label):
        """Call a service and show its answer in the status line."""
        if not client.service_is_ready():
            self.report(f'{label}: service not available')
            return
        future = client.call_async(request)

        def done(f):
            result = f.result()
            if result is None:
                self.report(f'{label}: no response')
                return
            ok = getattr(result, 'success', getattr(result, 'mode_sent', None))
            note = getattr(result, 'message', '')
            self.report(f'{label}: {"OK" if ok else "FAILED"}{" -- " + note if note else ""}')
        future.add_done_callback(done)

    # -- commands --------------------------------------------------------------

    def set_mode(self, ns, mode):
        self.call(self.cli[ns]['mode'], SetMode.Request(custom_mode=mode), f'{ns} {mode}')

    def arm(self, ns):
        self.call(self.cli[ns]['arm'], CommandBool.Request(value=True), f'{ns} ARM')

    def takeoff(self, ns, altitude):
        self.call(self.cli[ns]['takeoff'], CommandTOL.Request(altitude=float(altitude)),
                  f'{ns} TAKEOFF {altitude:.1f} m')

    def land(self, ns):
        self.call(self.cli[ns]['land'], CommandTOL.Request(), f'{ns} LAND')

    def enable(self, ns, value):
        self.call(self.cli[ns]['enable'], SetBool.Request(data=value),
                  f'{ns} controller {"ON" if value else "OFF"}')

    def leader_velocity(self, vx, vy, vz, w):
        cmd = Twist()
        cmd.linear.x, cmd.linear.y, cmd.linear.z, cmd.angular.z = vx, vy, vz, w
        self.leader_pub.publish(cmd)

    def follow(self, value):
        self.call(self.follow_cli, SetBool.Request(data=value),
                  'follow mochila' if value else 'manual leader')

    def reset_leader(self):
        self.call(self.reset_cli, Trigger.Request(), 'reset leader')

    def set_formation(self, name, spacing, transition_time):
        params = [Parameter('formation', value=name).to_parameter_msg(),
                  Parameter('spacing', value=float(spacing)).to_parameter_msg(),
                  Parameter('transition_time', value=float(transition_time)).to_parameter_msg()]
        if not self.param_cli.service_is_ready():
            self.report('formation: formation_leader not available')
            return
        # Atomic, so a rejection leaves all three unchanged.
        future = self.param_cli.call_async(SetParametersAtomically.Request(parameters=params))

        def done(f):
            result = f.result().result if f.result() else None
            if result is None:
                self.report('formation: no response')
            elif not result.successful:
                self.report(f'formation: REJECTED -- {result.reason}')
            else:
                self.report(f'formation: {name} at {spacing:.1f} m over {transition_time:.0f} s')
        future.add_done_callback(done)


class Panel(QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.cells = {}
        self.held = set()       # leader axes currently held down
        self.setWindowTitle('SWARM formation')
        mono = QFont('monospace', 11)
        columns = node.drone_ns + [ALL]

        grid = QGridLayout()
        grid.setHorizontalSpacing(14)
        for col, name in enumerate(columns, start=1):
            head = QLabel(name)
            head.setFont(QFont('monospace', 12, QFont.Bold))
            head.setAlignment(Qt.AlignCenter)
            grid.addWidget(head, 0, col)
        rows = ('mode', 'armed', 'ctrl', '|e| m')
        for row, label in enumerate(rows, start=1):
            tag = QLabel(label)
            tag.setFont(mono)
            tag.setStyleSheet(f'color: {DIM};')
            grid.addWidget(tag, row, 0)
            for col, ns in enumerate(node.drone_ns, start=1):
                cell = QLabel('—')
                cell.setFont(mono)
                cell.setAlignment(Qt.AlignCenter)
                grid.addWidget(cell, row, col)
                self.cells[(ns, label)] = cell

        self.altitude = QDoubleSpinBox()
        self.altitude.setRange(1.5, 10.0)
        self.altitude.setSingleStep(0.5)
        self.altitude.setValue(3.0)
        self.altitude.setSuffix(' m takeoff')
        for col, ns in enumerate(columns, start=1):
            box = QVBoxLayout()
            for text, action in (('GUIDED', lambda n: self.node.set_mode(n, 'GUIDED')),
                                 ('ARM', self.confirm_arm),
                                 ('TAKEOFF',
                                  lambda n: self.node.takeoff(n, self.altitude.value())),
                                 ('CTRL ON', lambda n: self.node.enable(n, True)),
                                 ('CTRL OFF', lambda n: self.node.enable(n, False)),
                                 ('LAND', self.node.land)):
                button = QPushButton(text)
                button.clicked.connect(lambda _, a=action, n=ns: self.each(n, a))
                box.addWidget(button)
            holder = QWidget()
            holder.setLayout(box)
            grid.addWidget(holder, len(rows) + 1, col)

        # Leader: hold to move.
        leader = QGridLayout()
        for text, axis, row, col in (('N ▲', NORTH, 0, 1), ('W ◀', WEST, 1, 0),
                                     ('E ▶', EAST, 1, 2), ('S ▼', SOUTH, 2, 1),
                                     ('UP', UP, 0, 3), ('DOWN', DOWN, 2, 3),
                                     ('YAW ⟲', YAW_LEFT, 0, 4), ('YAW ⟳', YAW_RIGHT, 2, 4)):
            button = QPushButton(text)
            button.pressed.connect(lambda a=axis: self.press(a))
            button.released.connect(lambda a=axis: self.release(a))
            leader.addWidget(button, row, col)
        stop = QPushButton('STOP')
        stop.clicked.connect(self.stop_leader)
        leader.addWidget(stop, 1, 1)
        reset = QPushButton('RESET LEADER\nto swarm')
        reset.clicked.connect(self.node.reset_leader)
        leader.addWidget(reset, 1, 4)

        source_row = QHBoxLayout()
        self.source_label = QLabel('')
        self.source_label.setFont(mono)
        source_row.addWidget(self.source_label, 1)
        for text, value in (('MANUAL', False), ('FOLLOW MOCHILA', True)):
            button = QPushButton(text)
            button.clicked.connect(lambda _, v=value: self.node.follow(v))
            source_row.addWidget(button)

        self.shape = QComboBox()
        self.shape.addItems(['line', 'square', 'triangle'])
        self.spacing = QDoubleSpinBox()
        self.spacing.setRange(0.5, 10.0)
        self.spacing.setSingleStep(0.5)
        self.spacing.setValue(3.0)
        self.spacing.setSuffix(' m spacing')
        self.transition = QDoubleSpinBox()
        self.transition.setRange(1.0, 60.0)
        self.transition.setValue(6.0)
        self.transition.setSuffix(' s transition')
        apply = QPushButton('APPLY FORMATION')
        apply.clicked.connect(lambda: self.node.set_formation(
            self.shape.currentText(), self.spacing.value(), self.transition.value()))
        shape_row = QHBoxLayout()
        for widget in (self.shape, self.spacing, self.transition, apply, self.altitude):
            shape_row.addWidget(widget)

        land_all = QPushButton('■   L A N D   A L L   ■')
        land_all.setFont(QFont('monospace', 14, QFont.Bold))
        land_all.setMinimumHeight(56)
        land_all.setStyleSheet(f'background-color: {BAD}; color: white; border-radius: 6px;')
        # No dialog on purpose: this is the emergency button.
        land_all.clicked.connect(lambda: [self.node.land(ns) for ns in self.node.drone_ns])

        self.status = QLabel('')
        self.status.setFont(mono)
        self.status.setWordWrap(True)
        self.leader_label = QLabel('leader: not reset yet')
        self.leader_label.setFont(mono)
        keys = QLabel('keys: arrows move · PgUp/PgDn up/down · Q/E yaw · Space stop')
        keys.setStyleSheet(f'color: {DIM};')

        outer = QVBoxLayout()
        outer.addLayout(grid)
        outer.addLayout(shape_row)
        outer.addLayout(source_row)
        outer.addWidget(self.leader_label)
        outer.addWidget(keys)
        outer.addLayout(leader)
        outer.addWidget(self.status)
        outer.addWidget(land_all)
        self.setLayout(outer)

        refresh = QTimer(self)
        refresh.timeout.connect(self.refresh)
        refresh.start(200)
        stream = QTimer(self)
        stream.timeout.connect(self.stream_leader)
        stream.start(100)
        # Application-wide, so the keys work whichever widget has focus.
        QApplication.instance().installEventFilter(self)

    def each(self, ns, action):
        if action == self.confirm_arm:
            return action(ns)       # one dialog for ALL, not four
        for target in (self.node.drone_ns if ns == ALL else [ns]):
            action(target)

    def confirm_arm(self, ns):
        box = QMessageBox(self)
        box.setIcon(QMessageBox.Warning)
        box.setWindowTitle(f'ARM {ns}?')
        box.setText('Propellers will spin immediately.\n\nArea clear? Safety pilot ready?')
        box.setStandardButtons(QMessageBox.Yes | QMessageBox.Cancel)
        # Cancel is the default, so a stray Enter does not arm.
        box.setDefaultButton(QMessageBox.Cancel)
        if box.exec_() == QMessageBox.Yes:
            for target in (self.node.drone_ns if ns == ALL else [ns]):
                self.node.arm(target)

    # -- leader ----------------------------------------------------------------

    def eventFilter(self, obj, event):
        if obj is self and event.type() == QEvent.WindowDeactivate and self.held:
            # A key released in another window never reaches us, so stop on focus loss.
            self.release_all()
            return False
        if event.type() not in (QEvent.KeyPress, QEvent.KeyRelease) or not self.isActiveWindow():
            return False
        key = event.key()
        if key == Qt.Key_Space:
            if event.type() == QEvent.KeyPress:
                self.stop_leader()
            return True
        if key not in KEYS:
            return False
        # Ignore auto-repeat while a key is held.
        if not event.isAutoRepeat():
            if event.type() == QEvent.KeyPress:
                self.press(KEYS[key])
            else:
                self.release(KEYS[key])
        return True

    def press(self, axis):
        self.held.add(axis)
        self.stream_leader()    # now, not at the next 10 Hz tick: a quick tap must still count

    def release(self, axis):
        self.held.discard(axis)
        if not self.held:
            self.node.leader_velocity(0.0, 0.0, 0.0, 0.0)

    def release_all(self):
        self.held.clear()
        self.node.leader_velocity(0.0, 0.0, 0.0, 0.0)

    def stop_leader(self):
        self.release_all()
        self.node.follow(False)

    def stream_leader(self):
        if not self.held:
            return
        v, w = self.node.leader_speed, self.node.leader_yaw_rate
        total = [sum(a[k] for a in self.held) for k in range(4)]
        self.node.leader_velocity(total[0] * v, total[1] * v, total[2] * v, total[3] * w)

    # -- display ---------------------------------------------------------------

    def set(self, ns, row, text, colour=None):
        cell = self.cells[(ns, row)]
        cell.setText(text)
        cell.setStyleSheet(f'color: {colour};' if colour else '')

    def refresh(self):
        snap, message, leader, leader_t, source = self.node.snapshot()
        now = time.monotonic()
        colour = DIM if source in (None, 'MANUAL') else OK if source == 'MOCHILA' else WARN
        self.source_label.setText(f'source: {source or "?"}')
        self.source_label.setStyleSheet(f'color: {colour};')
        if leader is None or now - leader_t > STALE_AFTER:
            self.leader_label.setText('leader: not publishing (reset it once the drones are up)')
        else:
            x, y, z, yaw, speed, vz, w = leader
            self.leader_label.setText(
                f'leader: E {x:6.2f}  N {y:6.2f}  U {z:5.2f} m  heading {yaw:4.0f}°  '
                f'| {speed:.2f} m/s, vz {vz:+.2f}, yaw rate {w:+.2f}')
        for ns, d in snap.items():
            state = d['state']
            if state is None or now - d['state_t'] > STALE_AFTER:
                self.set(ns, 'mode', 'NO DATA', BAD)
                self.set(ns, 'armed', '?', BAD)
            else:
                self.set(ns, 'mode', state.mode, OK if state.mode == 'GUIDED' else WARN)
                self.set(ns, 'armed', 'ARMED' if state.armed else 'disarmed',
                         BAD if state.armed else DIM)
            self.set(ns, 'ctrl', 'ON' if d['active'] else 'off', OK if d['active'] else DIM)
            e = d['error']
            if e is None or now - d['error_t'] > STALE_AFTER:
                self.set(ns, '|e| m', '—', DIM)
            else:
                norm = (e.x * e.x + e.y * e.y + e.z * e.z) ** 0.5
                self.set(ns, '|e| m', f'{norm:5.2f}', OK if norm < 0.5 else
                         WARN if norm < 2.0 else BAD)
        self.status.setText(message)


def main():
    rclpy.init()
    node = PanelNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()

    app = QApplication(sys.argv)
    signal.signal(signal.SIGINT, signal.SIG_DFL)   # Ctrl-C in the terminal closes the window
    window = Panel(node)
    window.show()
    try:
        app.exec_()
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
