from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'swarm_formation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
    	glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='spk',
    maintainer_email='spk@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        	'straight_line_leader = swarm_formation.straight_line_leader:main',
		'circle_leader = swarm_formation.circle_leader:main',
		'figure8_leader = swarm_formation.figure8_leader:main',
        	'formation_manager = swarm_formation.formation_manager:main',
        	'follower_controller = swarm_formation.follower_controller:main',
        	'swarm_visualizer = swarm_formation.swarm_visualizer:main',
        	'plotter = swarm_formation.plotter:main',
        ],
    },
)
