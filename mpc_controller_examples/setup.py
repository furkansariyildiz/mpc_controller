from setuptools import find_packages, setup

package_name = 'mpc_controller_examples'

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
    maintainer='furkan',
    maintainer_email='furkan@todo.todo',
    description='Python reference examples for the mpc_controller library.',
    license='TODO',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'speed_tracking_mpc = mpc_controller_examples.speed_tracking_mpc:main',
        ],
    },
)
