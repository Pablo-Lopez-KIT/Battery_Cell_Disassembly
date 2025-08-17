from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'battery_cell_disassembly'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/sim_launch.py']),
        (os.path.join('share', package_name, 'urdf'),
            glob('battery_cell_disassembly/urdf/*.urdf')),
        (os.path.join('share', package_name, 'urdf', 'meshes'),
            glob('battery_cell_disassembly/urdf/meshes/*.stl')),
        (os.path.join('share', package_name, 'urdf'),
            glob('battery_cell_disassembly/urdf/*.stl')),
        (os.path.join('share', package_name, 'urdf', 'meshes', 'ur10e', 'visual'),
            glob('battery_cell_disassembly/urdf/meshes/ur10e/visual/*.dae')),
        (os.path.join('share', package_name, 'urdf', 'meshes', 'ur10e', 'collision'),
            glob('battery_cell_disassembly/urdf/meshes/ur10e/collision/*.stl')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Pablo',
    maintainer_email='kitpablolopez@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'screwout = battery_cell_disassembly.screwout:main',
            'gripper = battery_cell_disassembly.gripper:main',
            'main_combined = battery_cell_disassembly.main_combined:main',
            'gripper_activation_node = battery_cell_disassembly.gripper_activation_node:main',
            'screwout_interpreter = battery_cell_disassembly.screwout_interpreter:main',
            'gripper_interpreter = battery_cell_disassembly.gripper_interpreter:main'
        ],
    },
)
