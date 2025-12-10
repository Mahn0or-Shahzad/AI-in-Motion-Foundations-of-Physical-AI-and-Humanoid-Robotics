from setuptools import setup

package_name = 'ai_motion_module1'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AI in Motion Team',
    maintainer_email='admin@aiinmotion.edu',
    description='Module 1: ROS 2 - Robotic Nervous System for AI in Motion curriculum',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joint_control_demo = ai_motion_module1.joint_control_demo:main',
            'topic_demo = ai_motion_module1.topic_demo:main',
            'service_demo = ai_motion_module1.service_demo:main',
        ],
    },
)