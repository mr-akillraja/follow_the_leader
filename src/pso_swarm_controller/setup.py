from setuptools import find_packages, setup

package_name = 'pso_swarm_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@todo.todo',
    description='PSO-based Swarm Controller',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pso_node = pso_swarm_controller.pso_leader_node:main',
            'wasd_teleop = pso_swarm_controller.wasd_teleop:main',
            'dashboard = pso_swarm_controller.gui_controller:main'
        ],
    },
)
