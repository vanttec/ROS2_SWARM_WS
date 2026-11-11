from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# Parámetros de la Mochila RTK (Líder)
MOCHILA_NS = 'mochila_rtk'
MOCHILA_SYSID = '5'
FCU_URL = '/dev/ttyUSB0:115200'


def generate_launch_description():
    # Rutas relativas a los paquetes del enjambre y MAVROS
    node_launch = PathJoinSubstitution([FindPackageShare('mavros'), 'launch', 'node.launch'])
    apm_config = PathJoinSubstitution([FindPackageShare('mavros'), 'launch', 'apm_config.yaml'])
    plugins = PathJoinSubstitution(
        [FindPackageShare('swarm_bringup'), 'config', 'swarm_pluginlists.yaml'])

    # 1. Instancia de MAVROS para la Mochila (usando node.launch y YAMLs)
    mavros_mochila = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(node_launch),
        launch_arguments={
            'fcu_url': FCU_URL,
            'gcs_url': '',
            'tgt_system': MOCHILA_SYSID,
            'tgt_component': '1',
            'pluginlists_yaml': plugins,
            'config_yaml': apm_config,
            'namespace': MOCHILA_NS,
        }.items())

    # 2. Configurar la tasa de refresco a 25 Hz para la mochila
    streams = Node(
        package='swarm_nodes',
        executable='stream_setup',
        name='stream_setup_mochila',
        output='screen',
        parameters=[{'namespaces': [MOCHILA_NS], 'rate_hz': 25.0}])

    # 3. Nodo Adaptador de la Mochila
    nodo_mochila = Node(
        package='swarm_nodes',
        executable='nodo_mochila',
        namespace=MOCHILA_NS,
        output='screen')

    return LaunchDescription([
        mavros_mochila,
        streams,
        nodo_mochila
    ])