import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'swarm_formation'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        # glob('launch/*.launch.py') silently misses launch.xml / launch.yaml
        # and any helper .py in launch/. Match both patterns explicitly.
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py') + glob('launch/*.launch.xml')),
        # Installed so `ros2 run swarm_formation start_swarm.sh` is not needed:
        # the script lands in share/ and is invoked by absolute path from the
        # README. Kept out of lib/ so colcon does not try to treat it as an
        # entry point.
        #
        # NOTE: data_files does not reliably preserve the executable bit. If
        # the installed copy comes out non-executable, either invoke it as
        # `bash <path>` or chmod +x it after install.
        (os.path.join('share', package_name, 'scripts'),
         glob('scripts/*.sh')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='SPK',
    maintainer_email='spk@example.com',
    description='Virtual-leader vector-field formation path following '
                '(ICC 2025 Basak and Ghosh) on PX4 SITL / Gazebo Harmonic.',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # --- leaders ---
            'straight_line_leader = swarm_formation.straight_line_leader:main',
            'circle_leader        = swarm_formation.circle_leader:main',
            'figure8_leader       = swarm_formation.figure8_leader:main',
            # NEW: sinusoidal reference path (paper Sec. IV, y = r sin x).
            # Also publishes /virtual_leader_accel with the analytic
            # chi_l_ddot needed by the follower's dDelta/dt feedforward.
            'sinusoid_leader      = swarm_formation.sinusoid_leader:main',
            # --- guidance ---
            'formation_manager    = swarm_formation.formation_manager:main',
            'follower_controller  = swarm_formation.follower_controller:main',
            # --- tooling ---
            'swarm_visualizer     = swarm_formation.swarm_visualizer:main',
            # WAS MISSING: without this entry point, `ros2 run
            # swarm_formation formation_logger` and any launch-file
            # Node(executable='formation_logger') both fail to resolve.
            'formation_logger     = swarm_formation.formation_logger:main',
            'plotter              = swarm_formation.plotter:main',
        ],
    },
)