from setuptools import setup

package_name = 'ai_motion_module4'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/vla_config.yaml']),
        ('share/' + package_name + '/launch', ['launch/vla_system.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AI in Motion Team',
    maintainer_email='admin@aiinmotion.edu',
    description='Module 4: Vision-Language-Action (VLA) for AI in Motion curriculum',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'voice_interface_node = ai_motion_module4.voice_interface_node:main',
            'cognitive_planner_node = ai_motion_module4.cognitive_planner_node:main',
            'vla_integration_node = ai_motion_module4.vla_integration_node:main',
            'intent_classifier_node = ai_motion_module4.intent_classifier_node:main',
            'action_executor_node = ai_motion_module4.action_executor_node:main',
        ],
    },
)