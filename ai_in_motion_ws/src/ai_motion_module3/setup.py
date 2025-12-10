from setuptools import setup

package_name = 'ai_motion_module3'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/isaac_sim.launch.py']),
        ('share/' + package_name + '/config', ['config/vslam_config.yaml', 'config/nav2_config.yaml']),
        ('share/' + package_name + '/worlds', ['worlds/obstacle_course.usd']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AI in Motion Team',
    maintainer_email='admin@aiinmotion.edu',
    description='Module 3: NVIDIA Isaac AI-Robot Brain for AI in Motion curriculum',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vslam_node = ai_motion_module3.vslam_node:main',
            'isaac_nav_node = ai_motion_module3.isaac_nav_node:main',
            'nav2_bipedal_controller = ai_motion_module3.nav2_bipedal_controller:main',
            'obstacle_course_demo = ai_motion_module3.obstacle_course_demo:main',
        ],
    },
)