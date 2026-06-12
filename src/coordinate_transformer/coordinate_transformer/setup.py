from setuptools import find_packages, setup

package_name = 'coordinate_transformer'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/default.yaml']),
        ('share/' + package_name + '/launch', [
            'launch/transformer_launch.py',
            'launch/odin_transformer.launch.py',
        ]),
    ],
    install_requires=['setuptools', 'numpy', 'PyYAML', 'scipy'],
    zip_safe=True,
    maintainer='fjienan',
    maintainer_email='fjienan@163.com',
    description='ROS2 coordinate transformation package for odin SLAM',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'coordinate_transformer = coordinate_transformer.coordinate_transformer:main',
            'calibrate_sensor_x = coordinate_transformer.calibrate_sensor_x:main',
        ],
    },
)
