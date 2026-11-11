import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
from rclpy.qos import qos_profile_sensor_data

class NodoMochila(Node):
    def __init__(self):
        super().__init__('nodo_mochila')

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

        # Publicadores estandarizados para el enjambre
        self.pub_pose = self.create_publisher(PoseStamped, '/mochila_rtk/pose', 10)
        self.pub_gps = self.create_publisher(NavSatFix, '/mochila_rtk/gps', 10)

        self.get_logger().info('Nodo Mochila (Adaptador MAVROS) iniciado correctamente.')

    def cb_mavros_pose(self, msg: PoseStamped):
        # Re-publica la pose del líder directamente a /mochila_rtk/pose
        self.pub_pose.publish(msg)

    def cb_mavros_gps(self, msg: NavSatFix):
        # Re-publica el GPS corregido para monitoreo
        self.pub_gps.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    nodo = NodoMochila()
    rclpy.spin(nodo)
    nodo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()