"""Virtual leader: integrates /formation/leader_cmd (a velocity, with speed and acceleration
limits) and publishes each drone's target in swarm_origin:

    p_i = p_L + R(psi_L) d_i
    v_i = v_L + w_L x (R(psi_L) d_i) + R(psi_L) d_i'     (d_i' only while changing shape)

d_i is the drone's offset in the leader frame (x forward, y left). formation, spacing and
transition_time are runtime parameters; a change that brings two drones closer than
min_separation is rejected. Nothing is published until ~/reset_to_swarm.
Target twists are in frame_id (world ENU), not in the child frame.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

FORMATIONS = ('square', 'line', 'triangle')
SQRT3_2 = math.sqrt(3.0) / 2.0


def formation_offsets(name, s):
    """Offsets for drone1..drone4, in metres. s is the closest-neighbour distance in every shape;
    the slot order keeps every transition at least 0.707 s apart."""
    h = s / 2.0
    if name == 'square':        # side s
        return [(h, h), (-h, h), (-h, -h), (h, -h)]
    if name == 'line':          # abreast, across the heading
        return [(0.0, 1.5 * s), (0.0, h), (0.0, -h), (0.0, -1.5 * s)]
    # triangle: three corners at distance s from a fourth drone on the centroid
    return [(h, SQRT3_2 * s), (0.0, 0.0), (-s, 0.0), (h, -SQRT3_2 * s)]


def closest_approach(start, goal):
    """Smallest distance between any two drones moving in straight lines from start to goal.

    Exact for the smoothstep transition: it changes the timing along the lines, not the lines.
    """
    best = math.inf
    for i in range(len(start)):
        for j in range(i + 1, len(start)):
            d0 = (start[i][0] - start[j][0], start[i][1] - start[j][1])
            d1 = (goal[i][0] - goal[j][0], goal[i][1] - goal[j][1])
            v = (d1[0] - d0[0], d1[1] - d0[1])
            vv = v[0] * v[0] + v[1] * v[1]
            t = 0.0 if vv == 0.0 else min(1.0, max(0.0, -(d0[0] * v[0] + d0[1] * v[1]) / vv))
            best = min(best, math.hypot(d0[0] + v[0] * t, d0[1] + v[1] * t))
    return best


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class FormationLeader(Node):
    def __init__(self):
        super().__init__('formation_leader')
        p = self.declare_parameter
        self.drone_ns = p('namespaces', ['drone1', 'drone2', 'drone3', 'drone4']).value
        if len(self.drone_ns) != 4:
            raise ValueError('the formations are defined for exactly four drones')
        self.formation = p('formation', 'line').value
        self.spacing = p('spacing', 3.0).value
        self.transition_time = p('transition_time', 6.0).value
        self.min_separation = p('min_separation', 1.5).value
        rate_hz = p('rate_hz', 20.0).value
        # Must stay well below the controllers' v_max (README, "Authority").
        self.max_speed_xy = p('max_speed_xy', 0.3).value
        self.max_speed_z = p('max_speed_z', 0.2).value
        self.max_yaw_rate = p('max_yaw_rate', 0.15).value
        self.max_accel = p('max_accel', 0.5).value
        self.max_yaw_accel = p('max_yaw_accel', 0.3).value
        self.cmd_timeout = p('cmd_timeout', 0.5).value
        self.frame_id = p('frame_id', 'swarm_origin').value
        self.fence_min = p('fence_min', [-15.0, -15.0, 1.0]).value
        self.fence_max = p('fence_max', [25.0, 15.0, 10.0]).value

        self.leader = None                  # [x, y, z, psi] once reset
        self.vel = [0.0, 0.0, 0.0, 0.0]     # [vx, vy, vz, w], world frame
        self.cmd = [0.0, 0.0, 0.0, 0.0]
        self.cmd_time = None
        self.last_tick = None
        offsets = formation_offsets(self.formation, self.spacing)
        self.transition = (offsets, offsets, 0.0)   # (start, goal, start_time)
        self.poses = {ns: None for ns in self.drone_ns}     # (x, y, z, yaw, receipt time)
        self.active = {ns: False for ns in self.drone_ns}

        # Reliable so RViz and ros2 topic echo can subscribe with their defaults.
        self.leader_pub = self.create_publisher(Odometry, '/formation/leader', 10)
        self.target_pubs = {}
        latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        for ns in self.drone_ns:
            self.target_pubs[ns] = self.create_publisher(
                Odometry, f'/{ns}/formation_target', qos_profile_sensor_data)
            self.create_subscription(Odometry, f'/{ns}/odom',
                                     lambda m, ns=ns: self.on_odom(ns, m),
                                     qos_profile_sensor_data)
            self.create_subscription(Bool, f'/{ns}/formation_controller/active',
                                     lambda m, ns=ns: self.active.__setitem__(ns, m.data),
                                     latched)
        self.create_subscription(Twist, '/formation/leader_cmd', self.on_cmd, 10)
        self.create_service(Trigger, '~/reset_to_swarm', self.on_reset)
        self.add_on_set_parameters_callback(self.on_parameters)
        self.create_timer(1.0 / rate_hz, self.tick)
        self.get_logger().info(
            f'{self.formation} at {self.spacing} m, min separation {self.min_separation} m. '
            f'Waiting for ~/reset_to_swarm before publishing any target.')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # -- inputs ----------------------------------------------------------------

    def on_odom(self, ns, msg):
        if msg.header.frame_id != self.frame_id:
            return
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.poses[ns] = (p.x, p.y, p.z, yaw, self.now())

    def on_cmd(self, msg):
        values = (msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z)
        if not all(math.isfinite(v) for v in values):
            return
        vx, vy = msg.linear.x, msg.linear.y
        speed = math.hypot(vx, vy)
        if speed > self.max_speed_xy:
            vx, vy = vx * self.max_speed_xy / speed, vy * self.max_speed_xy / speed
        self.cmd = [vx, vy,
                    max(-self.max_speed_z, min(self.max_speed_z, msg.linear.z)),
                    max(-self.max_yaw_rate, min(self.max_yaw_rate, msg.angular.z))]
        self.cmd_time = self.now()

    def on_reset(self, request, response):
        if any(self.active.values()):
            response.success = False
            response.message = 'disable every controller before resetting the leader'
            return response
        now = self.now()
        fresh = [pose for pose in self.poses.values() if pose and now - pose[4] < 1.0]
        if len(fresh) != len(self.drone_ns):
            response.success = False
            response.message = (f'need fresh swarm_origin odometry from all '
                                f'{len(self.drone_ns)} drones, have {len(fresh)}')
            return response
        n = len(fresh)
        # Mean heading through sin/cos, so it works across +-pi.
        psi = math.atan2(sum(math.sin(f[3]) for f in fresh), sum(math.cos(f[3]) for f in fresh))
        self.leader = [sum(f[0] for f in fresh) / n, sum(f[1] for f in fresh) / n,
                       sum(f[2] for f in fresh) / n, psi]
        self.vel = [0.0, 0.0, 0.0, 0.0]
        self.cmd_time = None
        goal = self.transition[1]
        self.transition = (goal, goal, now)
        response.success = True
        response.message = (f'leader at {self.leader[0]:.2f}, {self.leader[1]:.2f}, '
                            f'{self.leader[2]:.2f} m, heading {math.degrees(psi):.0f} deg')
        self.get_logger().info(response.message)
        return response

    def on_parameters(self, params):
        """Check a formation change before ROS applies it; start the transition if safe."""
        new = {'formation': self.formation, 'spacing': self.spacing,
               'transition_time': self.transition_time, 'min_separation': self.min_separation}
        for param in params:
            if param.name in new:
                new[param.name] = param.value
        if new['formation'] not in FORMATIONS:
            return SetParametersResult(successful=False,
                                       reason=f'formation must be one of {FORMATIONS}')
        if new['spacing'] <= 0.0 or new['transition_time'] <= 0.0:
            return SetParametersResult(successful=False,
                                       reason='spacing and transition_time must be positive')

        start, _ = self.offsets_now(self.now())
        goal = formation_offsets(new['formation'], new['spacing'])
        clearance = closest_approach(start, goal)
        if clearance < new['min_separation']:
            reason = (f'{new["formation"]} at {new["spacing"]} m: drones would come within '
                      f'{clearance:.2f} m, below min_separation {new["min_separation"]} m')
            self.get_logger().warn(f'REJECTED {reason}')
            return SetParametersResult(successful=False, reason=reason)

        if goal != self.transition[1]:
            self.transition = (start, goal, self.now())
            self.get_logger().info(
                f'-> {new["formation"]} at {new["spacing"]} m over {new["transition_time"]} s '
                f'(closest approach {clearance:.2f} m)')
        self.formation, self.spacing = new['formation'], new['spacing']
        self.transition_time, self.min_separation = new['transition_time'], new['min_separation']
        return SetParametersResult(successful=True)

    # -- output ----------------------------------------------------------------

    def offsets_now(self, now):
        """Offsets d_i and rates d_i' along a smoothstep (3u^2 - 2u^3): zero rate at both ends."""
        start, goal, t0 = self.transition
        u = min(1.0, max(0.0, (now - t0) / self.transition_time))
        sigma = u * u * (3.0 - 2.0 * u)
        rate = 6.0 * u * (1.0 - u) / self.transition_time
        offsets = [(s[0] + (g[0] - s[0]) * sigma, s[1] + (g[1] - s[1]) * sigma)
                   for s, g in zip(start, goal)]
        rates = [((g[0] - s[0]) * rate, (g[1] - s[1]) * rate) for s, g in zip(start, goal)]
        return offsets, rates

    def tick(self):
        now = self.now()
        dt = 0.0 if self.last_tick is None else now - self.last_tick
        self.last_tick = now
        if self.leader is None:
            return
        # Skip a step after a long gap (sim paused, clock jump) instead of lurching.
        if not 0.0 < dt < 0.5:
            dt = 0.0

        fresh = self.cmd_time is not None and now - self.cmd_time < self.cmd_timeout
        cmd = self.cmd if fresh else [0.0, 0.0, 0.0, 0.0]

        dvx, dvy = cmd[0] - self.vel[0], cmd[1] - self.vel[1]
        dv = math.hypot(dvx, dvy)
        step = self.max_accel * dt
        if dv > step:
            dvx, dvy = dvx * step / dv, dvy * step / dv
        self.vel[0] += dvx
        self.vel[1] += dvy
        self.vel[2] += max(-step, min(step, cmd[2] - self.vel[2]))
        yaw_step = self.max_yaw_accel * dt
        self.vel[3] += max(-yaw_step, min(yaw_step, cmd[3] - self.vel[3]))

        for k in range(3):
            self.leader[k] += self.vel[k] * dt
        self.leader[3] = wrap(self.leader[3] + self.vel[3] * dt)

        offsets, rates = self.offsets_now(now)
        # Stop the leader one formation radius short of the fence, so no target leaves it.
        radius = max(math.hypot(*d) for d in offsets + list(self.transition[1]))
        for k in range(3):
            margin = radius if k < 2 else 0.0
            lo, hi = self.fence_min[k] + margin, self.fence_max[k] - margin
            if lo > hi:
                lo = hi = (self.fence_min[k] + self.fence_max[k]) / 2.0
            if not lo <= self.leader[k] <= hi:
                self.leader[k] = min(hi, max(lo, self.leader[k]))
                self.vel[k] = 0.0

        x, y, z, psi = self.leader
        vx, vy, vz, w = self.vel
        stamp = self.get_clock().now().to_msg()
        self.leader_pub.publish(self.odometry(stamp, 'formation_leader', x, y, z, psi,
                                              vx, vy, vz, w))
        c, s = math.cos(psi), math.sin(psi)
        for ns, d, dd in zip(self.drone_ns, offsets, rates):
            rx, ry = c * d[0] - s * d[1], s * d[0] + c * d[1]      # R(psi) d
            self.target_pubs[ns].publish(self.odometry(
                stamp, f'{ns}/formation_target', x + rx, y + ry, z, psi,
                vx - w * ry + c * dd[0] - s * dd[1],
                vy + w * rx + s * dd[0] + c * dd[1], vz, w))

    def odometry(self, stamp, child, x, y, z, psi, vx, vy, vz, w):
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.child_frame_id = child
        msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = x, y, z
        msg.pose.pose.orientation.z = math.sin(psi / 2.0)
        msg.pose.pose.orientation.w = math.cos(psi / 2.0)
        msg.twist.twist.linear.x, msg.twist.twist.linear.y = vx, vy
        msg.twist.twist.linear.z = vz
        msg.twist.twist.angular.z = w
        return msg


def main():
    rclpy.init()
    rclpy.spin(FormationLeader())


if __name__ == '__main__':
    main()
