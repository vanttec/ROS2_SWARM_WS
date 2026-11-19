import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'swarm_formation'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='GorgeLop07',
    maintainer_email='jorgelolpez07@gmail.com',
    description='Virtual-leader formation controller for the SWARM-RSCL platform (experiment)',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'formation_leader = swarm_formation.formation_leader:main',
            'formation_controller = swarm_formation.formation_controller:main',
            'formation_panel = swarm_formation.formation_panel:main',
            'plot_errors = swarm_formation.plot_errors:main',
            'gz_clock = swarm_formation.gz_clock:main',
        ],
    },
)
