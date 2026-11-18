from glob import glob

from setuptools import find_packages, setup

package_name = 'ur10e_control_lab'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/urdf', glob('urdf/*.xacro')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/worlds', glob('worlds/*.sdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kuma3y1t',
    maintainer_email='kuma3y1t@gmail.com',
    description='UR10e control theory learning lab on Gazebo',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lab_node = ur10e_control_lab.lab_node:main',
            'offline_sim = ur10e_control_lab.offline_sim:main',
            'plot_log = ur10e_control_lab.plot_log:main',
            'push = ur10e_control_lab.push:main',
        ],
    },
)
