from setuptools import find_packages, setup

package_name = 'service_client_package'

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
            "server = service_client_package.server:main",
            "client = service_client_package.client:main",
            "srv_even_odd = service_client_package.srv_even_odd:main",
            "cli_even_odd = service_client_package.cli_even_odd:main",
            "square_server_node = service_client_package.square_srv:main",
            "square_client_node = service_client_package.square_cli:main",
            "turtle_cli = service_client_package.turtle_cli:main",
        ],
    },
)
