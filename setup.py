from setuptools import setup

package_name = 'labjack'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Pierce Nichols',
    maintainer_email='pierce@ladonrobotics.com',
    description='Labjack U3 interface node',
    license='Copyright Ladon Robotics (c) 2022. All rights reserved.',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'labjack_node = labjack.labjack_node:main'
        ],
    },
)
