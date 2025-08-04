from setuptools import find_packages, setup

package_name = 'my_package'

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
    maintainer='saeed',
    maintainer_email='saeed@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "test_node = my_package.my_node:main",
            "drew_circle = my_package.drew_circle:main",
            "pose_subscriber = my_package.pose_subscriber:main",
            "robot_control = my_package.my_robot_controll:main",
            "custom_service_subs = my_package.cheak_custom_serivce_subs:main",
            "custom_service_pub = my_package.cheak_custom_serivce_pub:main",
        ],
    },
)
