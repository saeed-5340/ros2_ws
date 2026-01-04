from setuptools import find_packages, setup

package_name = 'level_3_action_challange'

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
            "srv_node = level_3_action_challange.server_node:main",
            "cli_node = level_3_action_challange.client_node:main",
            "pub_node = level_3_action_challange.pub_node:main",
        ],
    },
)
