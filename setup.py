from setuptools import find_packages, setup
from glob import glob

package_name = 'sim_human'

setup(
    name=package_name,
    version='0.8.28',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['myoelbow_1dof6muscles_1dofexo.xml']),
        ('share/' + package_name + '/assets', ['assets/myoelbow_assets.xml']),
        ('share/' + package_name + '/assets', ['assets/myoelbow_1dof6muscles_1dofexo_body_revised_2.xml']),
        ('share/' + package_name + '/myo_sym', [ path for path in glob('myo_sym/*/*', recursive=True)]), 
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
            'sim_human_node = sim_human.main:main',
            'elbow_control_node = sim_human.elbow_control:main',
            'robot_control_node = sim_human.robot_control:main',
            'simulation_node = sim_human.simulation:main',
        ],
    },
)
