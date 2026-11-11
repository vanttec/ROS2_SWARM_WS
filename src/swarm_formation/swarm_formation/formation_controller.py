"""Position controller for one drone, run once per namespace (-r __ns:=/droneN).

In GUIDED, ArduCopter tracks velocity itself, so the drone is modelled as p' = u. With e = p_d - p:

    u = v_d + v_max tanh(K |e| / v_max) e / |e|

V = 1/2 |e|^2 gives V' = -|e| v_max tanh(K |e| / v_max) < 0, so the error converges (README.md).

In:  odom, formation_target, mavros/state.  Out: cmd_vel (to command_bridge), formation_error.
Commands only when enabled through ~/enable, in GUIDED, armed and above min_altitude.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, qos_profile_sensor_data
from geometry_msgs.msg import Twist, Vector3Stamped
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from std_srvs.srv import SetBool

ZERO = Twist()


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def saturate(vector, k, limit):
    """k * vector near zero, smoothly capped at `limit`. Scales the norm to keep the direction."""
    norm = math.sqrt(sum(c * c for c in vector))
    if norm == 0.0:
        return [0.0] * len(vector)
    scale = limit * math.tanh(k * norm / limit) / norm
    return [c * scale for c in vector]


def clamp_norm(vector, limit):
    norm = math.sqrt(sum(c * c for c in vector))
    if norm <= limit:
        return list(vector), False
    return [c * limit / norm for c in vector], True


class FormationController(Node):
    def __init__(self):
        super().__init__('formation_controller')
        p = self.declare_parameter
        self.k = p('k', 0.8).value                  # 1/s
        self.v_max_xy = p('v_max_xy', 1.0).value    # m/s
        self.v_max_z = p('v_max_z', 0.5).value
        self.k_yaw = p('k_yaw', 0.8).value          # 1/s
        self.yaw_rate_max = p('yaw_rate_max', 0.5).value
        self.ki = p('ki', 0.0).value                # optional integral, off by default
        self.i_max = p('i_max', 0.3).value          # m/s the integral may add at most
        rate_hz = p('rate_hz', 20.0).value
        self.stale_timeout = p('stale_timeout', 0.5).value
        # A velocity setpoint below this height would cancel ArduCopter's takeoff.
        self.min_altitude = p('min_altitude', 1.5).value
        # A large error at enable means the leader was not reset where the swarm is.
        self.max_enable_error = p('max_enable_error', 2.0).value
        # A jump this large between two messages means the frame moved, not the drone.
        self.max_pose_jump = p('max_pose_jump', 1.0).value
        self.fence_min = p('fence_min', [-15.0, -15.0, 1.0]).value
        self.fence_max = p('fence_max', [25.0, 15.0, 10.0]).value
        self.frame_id = p('frame_id', 'swarm_origin').value

        self.pose = None        # (position[3], yaw, receipt time)
        self.target = None      # (position[3], velocity[3], yaw, yaw rate, receipt time)
        self.mode, self.armed = '', False
        self.enabled = False
        self.integral = [0.0, 0.0, 0.0]
        self.last_tick = None

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.error_pub = self.create_publisher(Vector3Stamped, 'formation_error', 10)
        latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.active_pub = self.create_publisher(Bool, '~/active', latched)
        self.active_pub.publish(Bool(data=False))
        self.create_subscription(Odometry, 'odom', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(Odometry, 'formation_target', self.on_target,
                                 qos_profile_sensor_data)
        self.create_subscription(State, 'mavros/state', self.on_state, qos_profile_sensor_data)
        self.create_service(SetBool, '~/enable', self.on_enable)
        self.create_timer(1.0 / rate_hz, self.tick)
        self.get_logger().info(
            f'K {self.k} 1/s, v_max {self.v_max_xy}/{self.v_max_z} m/s, ki {self.ki}; '
            f'disabled until ~/enable')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # -- inputs ----------------------------------------------------------------

    def on_odom(self, msg):
        if msg.header.frame_id != self.frame_id:
            self.get_logger().warn(f'odom in {msg.header.frame_id!r}, not {self.frame_id!r}; '
                                   f'ignored', throttle_duration_sec=5.0)
            return
        p = msg.pose.pose.position
        position = [p.x, p.y, p.z]
        if self.pose is not None:
            jump = math.dist(position, self.pose[0])
            if jump > self.max_pose_jump:
                self.disable(f'odometry jumped {jump:.2f} m -- the frame changed (odom_relay '
                             f'restarted?); reset the leader before enabling again')
        self.pose = (position, yaw_of(msg.pose.pose.orientation), self.now())

    def on_target(self, msg):
        p, v = msg.pose.pose.position, msg.twist.twist.linear
        self.target = ([p.x, p.y, p.z], [v.x, v.y, v.z], yaw_of(msg.pose.pose.orientation),
                       msg.twist.twist.angular.z, self.now())

    def on_state(self, msg):
        self.mode, self.armed = msg.mode, msg.armed
        if self.enabled and (msg.mode != 'GUIDED' or not msg.armed):
            self.disable(f'drone is {"armed" if msg.armed else "disarmed"} in {msg.mode}')

    def fresh(self, item):
        # Receipt time, not the header stamp: MAVROS stamps in wall time even under sim time.
        return item is not None and self.now() - item[-1] < self.stale_timeout

    def on_enable(self, request, response):
        if not request.data:
            self.disable('requested')
            response.success, response.message = True, 'disabled'
            return response
        problem = None
        if self.mode != 'GUIDED' or not self.armed:
            problem = f'needs GUIDED and armed (is {self.mode or "?"}, armed={self.armed})'
        elif not self.fresh(self.pose):
            problem = f'no fresh odometry in {self.frame_id}'
        elif not self.fresh(self.target):
            problem = 'no fresh formation target (has the leader been reset?)'
        elif self.pose[0][2] < self.min_altitude:
            problem = f'at {self.pose[0][2]:.2f} m, below min_altitude {self.min_altitude} m'
        else:
            error = math.dist(self.target[0], self.pose[0])
            if error > self.max_enable_error:
                problem = (f'{error:.2f} m from its target, more than max_enable_error '
                           f'{self.max_enable_error} m -- reset the leader first')
        if problem:
            response.success, response.message = False, problem
            self.get_logger().warn(f'enable refused: {problem}')
            return response
        self.integral = [0.0, 0.0, 0.0]
        self.enabled = True
        self.active_pub.publish(Bool(data=True))
        self.get_logger().info('ENABLED')
        response.success, response.message = True, 'enabled'
        return response

    def disable(self, reason):
        if not self.enabled:
            return
        self.enabled = False
        self.cmd_pub.publish(ZERO)
        self.active_pub.publish(Bool(data=False))
        self.get_logger().warn(f'DISABLED: {reason}')

    # -- control loop ----------------------------------------------------------

    def tick(self):
        now = self.now()
        dt = 0.0 if self.last_tick is None else now - self.last_tick
        self.last_tick = now
        if not (self.fresh(self.pose) and self.fresh(self.target)):
            if self.enabled:
                self.cmd_pub.publish(ZERO)
                self.get_logger().warn('odometry or target stale: holding zero velocity',
                                       throttle_duration_sec=2.0)
            return

        position, yaw, _ = self.pose
        goal, v_d, yaw_d, yaw_rate_d, _ = self.target
        goal = [min(hi, max(lo, g)) for g, lo, hi in zip(goal, self.fence_min, self.fence_max)]
        e = [g - x for g, x in zip(goal, position)]

        # Published while disabled too, so the error is visible before enabling.
        err = Vector3Stamped()
        err.header.stamp = self.get_clock().now().to_msg()
        err.header.frame_id = self.frame_id
        err.vector.x, err.vector.y, err.vector.z = e
        self.error_pub.publish(err)
        if not self.enabled:
            return

        u_xy = saturate(e[:2], self.k, self.v_max_xy)
        u_z = saturate(e[2:], self.k, self.v_max_z)
        u = [v_d[0] + u_xy[0], v_d[1] + u_xy[1], v_d[2] + u_z[0]]
        if self.ki > 0.0:
            u = [c + self.ki * i for c, i in zip(u, self.integral)]
        u_xy, sat_xy = clamp_norm(u[:2], self.v_max_xy)
        u_z, sat_z = clamp_norm(u[2:], self.v_max_z)
        u = u_xy + u_z

        # Anti-windup: integrate only while unsaturated, and cap the contribution.
        if self.ki > 0.0 and not (sat_xy or sat_z):
            cap = self.i_max / self.ki
            self.integral = [max(-cap, min(cap, i + ei * dt)) for i, ei in zip(self.integral, e)]

        # Outside the fence, drop any component pointing further out.
        for k in range(3):
            if (position[k] < self.fence_min[k] and u[k] < 0.0) or \
               (position[k] > self.fence_max[k] and u[k] > 0.0):
                u[k] = 0.0

        yaw_error = wrap(yaw_d - yaw)
        yaw_rate = yaw_rate_d + max(-self.yaw_rate_max,
                                    min(self.yaw_rate_max, self.k_yaw * yaw_error))
        yaw_rate = max(-self.yaw_rate_max, min(self.yaw_rate_max, yaw_rate))

        cmd = Twist()
        cmd.linear.x, cmd.linear.y, cmd.linear.z = u
        cmd.angular.z = yaw_rate
        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    rclpy.spin(FormationController())


if __name__ == '__main__':
    main()
