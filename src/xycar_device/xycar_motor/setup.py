import os
from glob import glob
from setuptools import setup

package_name = 'xycar_motor'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xytron',
    maintainer_email='xytron@todo.todo',
    description='The xycar_motor package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'xycar_motor = xycar_motor.xycar_motor:main',
            'vesc_test = xycar_motor.vesc_test:main',
            'motor_test = xycar_motor.motor_test:main'
        ],
    },
)
