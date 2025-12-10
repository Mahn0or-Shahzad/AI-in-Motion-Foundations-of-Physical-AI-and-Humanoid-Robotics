from setuptools import setup

package_name = 'ai_motion_module2'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/gazebo_simulation.launch.py']),
        ('share/' + package_name + '/config', ['config/sensors.yaml']),
        ('share/' + package_name + '/worlds', ['worlds/simple_room.world']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AI in Motion Team',
    maintainer_email='admin@aiinmotion.edu',
    description='Module 2: Digital Twin - Gazebo & Unity for AI in Motion curriculum',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gazebo_sensor_sim = ai_motion_module2.gazebo_sensor_sim:main',
            'lidar_sim = ai_motion_module2.lidar_sim:main',
            'camera_sim = ai_motion_module2.camera_sim:main',
            'imu_sim = ai_motion_module2.imu_sim:main',
        ],
    },
)