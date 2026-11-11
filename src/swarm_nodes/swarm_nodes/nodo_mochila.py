"""Adapt the RTK backpack's MAVROS topics to the swarm's standard /mochila_rtk/... names, and
report whether it is actually alive and RTK-corrected.

The backpack is a Pixhawk + RTK GPS carried on foot, not a flying vehicle -- it has no ARM,
TAKEOFF or LAND here (see dashboard.py). What the rest of the stack needs from it is the same
thing it needs from a drone: a position, an orientation, and confirmation that RTK corrections are
actually landing.

Position in the SHARED frame (comparable to /droneN/odom) comes from odom_relay, not from here --
odom_relay already lists mochila_rtk in its own namespaces and reads straight from MAVROS. What
this node adds on top is the plain re-publish (/mochila_rtk/pose, /mochila_rtk/gps, for anything
that wants the backpack's own-frame pose without subscribing to MAVROS directly) and the one thing
nothing else provides: whether the link is up and whether the fix is RTK Fixed, logged the same way
stream_setup and odom_relay report their own state instead of staying silent.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import State, GPSRAW
from rclpy.qos import qos_profile_sensor_data

# Mirrors dashboard.py's FIX_TYPES labels, for the same GPS_FIX_TYPE values.
FIX_LABELS = {0: 'no GPS', 1: 'no fix', 2: '2D', 3: '3D', 4: 'DGPS', 5: 'RTK Float',
              6: 'RTK Fixed', 7: 'static', 8: 'PPP'}


class NodoMochila(Node):
    def __init__(self):
        super().__init__('nodo_mochila')

        report_period = self.declare_parameter('report_period', 10.0).value

        self.connected = False
        self.seen_pose = False
        self.seen_gps = False
        self.fix_type = None
        self.satellites = None

        # Suscripción a la posición local ENU calculada por la Pixhawk de la mochila
        self.sub_mavros_pose = self.create_subscription(
            PoseStamped,
            '/mochila_rtk/mavros/local_position/pose',
            self.cb_mavros_pose,
            qos_profile_sensor_data
        )

        # Suscripción a las coordenadas geográficas de precisión RTK
        self.sub_mavros_gps = self.create_subscription(
            NavSatFix,
            '/mochila_rtk/mavros/global_position/global',
            self.cb_mavros_gps,
            qos_profile_sensor_data
        )

        # Estado de conexión del FCU -- lo mismo que stream_setup y first_flight usan para saber
        # si hay alguien del otro lado antes de confiar en cualquier otro dato.
        self.sub_mavros_state = self.create_subscription(
            State,
            '/mochila_rtk/mavros/state',
            self.cb_mavros_state,
            qos_profile_sensor_data
        )

        # GPS_FIX_TYPE crudo (24), no NavSatFix.status -- es lo único que distingue RTK Fixed (6)
        # de un 3D corriente (3). stream_setup ya pide este mensaje a 25 Hz para la mochila.
        self.sub_mavros_gpsraw = self.create_subscription(
            GPSRAW,
            '/mochila_rtk/mavros/gpsstatus/gps1/raw',
            self.cb_mavros_gpsraw,
            qos_profile_sensor_data
        )

        # Publicadores estandarizados para el enjambre
        self.pub_pose = self.create_publisher(PoseStamped, '/mochila_rtk/pose', 10)
        self.pub_gps = self.create_publisher(NavSatFix, '/mochila_rtk/gps', 10)

        if report_period > 0:
            self.create_timer(report_period, self.report)

        self.get_logger().info(
            'Nodo Mochila (Adaptador MAVROS) iniciado correctamente. '
            'Esperando conexión con el FCU...')

    def cb_mavros_state(self, msg: State):
        if msg.connected and not self.connected:
            self.get_logger().info('Mochila: FCU conectado.')
        elif not msg.connected and self.connected:
            self.get_logger().warn('Mochila: se perdió la conexión con el FCU.')
        self.connected = msg.connected

    def cb_mavros_pose(self, msg: PoseStamped):
        if not self.seen_pose:
            self.get_logger().info('Mochila: primera pose recibida.')
            self.seen_pose = True
        # Re-publica la pose del líder directamente a /mochila_rtk/pose
        self.pub_pose.publish(msg)

    def cb_mavros_gps(self, msg: NavSatFix):
        if not self.seen_gps:
            self.get_logger().info('Mochila: primer fix GPS recibido.')
            self.seen_gps = True
        # Re-publica el GPS corregido para monitoreo
        self.pub_gps.publish(msg)

    def cb_mavros_gpsraw(self, msg: GPSRAW):
        self.fix_type = msg.fix_type
        self.satellites = msg.satellites_visible

    def report(self):
        """Dice en voz alta lo que el silencio no puede: si está conectada, si ya llegó pose/GPS,
        y si las correcciones RTK realmente convergieron. Mismo criterio que dashboard.py."""
        if not self.connected:
            self.get_logger().warn(
                'Mochila: FCU no conectado -- revisa el cable/puerto y mavros_mochila en T0.')
            return
        if not self.seen_pose or not self.seen_gps:
            missing = []
            if not self.seen_pose:
                missing.append('pose')
            if not self.seen_gps:
                missing.append('GPS')
            self.get_logger().warn(f'Mochila: conectada pero sin {" ni ".join(missing)} todavía.')
            return
        if self.fix_type is None:
            self.get_logger().warn(
                'Mochila: sin GPS_RAW_INT (24) todavía -- stream_setup debería estar pidiéndolo.')
            return
        label = FIX_LABELS.get(self.fix_type, f'tipo {self.fix_type}')
        if self.fix_type == 6:
            self.get_logger().info(f'Mochila: {label}, {self.satellites} satélites -- RTK OK.')
        else:
            self.get_logger().warn(
                f'Mochila: {label} ({self.satellites} satélites) -- NO es RTK Fixed. '
                f'¿rtcm_relay está corriendo y llegando a mochila_rtk?')


def main(args=None):
    rclpy.init(args=args)
    nodo = NodoMochila()
    rclpy.spin(nodo)
    nodo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
