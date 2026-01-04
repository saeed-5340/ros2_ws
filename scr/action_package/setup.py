from setuptools import find_packages, setup

package_name = 'action_package'

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
            "count_until_server = action_package.count_until_server:main",
            "count_until_cli = action_package.count_until_cli:main",
            "even_odd_action_srv = action_package.even_odd_action_srv:main",
            "even_odd_action_cli = action_package.even_odd_action_cli:main",
            "srv_node = action_package.server_node:main",
        ],
    },
)
