from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'wearable_robot_mujoco'

setup(
    name=package_name,
    version='0.8.28',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['myoelbow_1dof6muscles_1dofexo.xml']),
        ('share/' + package_name + '/assets', glob('assets/*', recursive=True)),
        ('share/' + package_name + '/assets', glob('myo_sim/meshes/*', recursive=True)),
        ('share/' + package_name + '/assets', glob('myo_sim/textures/*', recursive=True)),
        ('share/' + package_name + '/config', ['patient_models.json']),
    ],
    install_requires=[
        'setuptools',
        'cvxpy>=1.7.2',
        'imageio>=2.37.0',
        'matplotlib>=3.10.6',
        'mujoco>=3.3.5',
        'numpy>=2.2.6',
    ],
    zip_safe=True,
    maintainer='donghee',
    maintainer_email='dongheepark@gmail.com',
    description='Human simulation package with MuJoCo and robot control',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'elbow_control_node = wearable_robot_mujoco.elbow_control:main',
            'elbow_vel_cmd_node = wearable_robot_mujoco.elbow_vel_cmd:main',
            'robot_control_node = wearable_robot_mujoco.robot_control:main',
            'simulation_node = wearable_robot_mujoco.simulation:main',
        ],
    },
)
