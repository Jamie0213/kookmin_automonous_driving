from glob import glob
import os
from setuptools import find_packages, setup

package_name = "control_motor"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=['test']),
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=[
        "setuptools", "rclpy", "numpy", "opencv-python",
        "matplotlib", "xycar_msgs", "cv_bridge",
    ],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    entry_points={
        "console_scripts": [
            "lane_detection = control_motor.lane_detection:main",
            "kmu_motor_node = control_motor.kmu_motor_node:main",
            "trafficlight = control_motor.trafficlight:main",
            "obstacle = control_motor.obstacle:main",
            'new_lane_detection = control_motor.new_lane_detection:main',
            'rubbercone_driver_node = control_motor.rubbercone_driver_node:main',
            'is_orange = control_motor.is_orange:main',
            'is_vehicle = control_motor.is_vehicle:main',
            'new_rubbercone_driver_node = control_motor.new_rubbercone_driver_node:main',
            'vehicle_avoidance = control_motor.vehicle_avoidance:main',
            'left_lane_detection = control_motor.left_lane_detection:main',
            'imu_heading_node = control_motor.imu_heading_node:main',
        ],
    },
    options={                                  # ← 추가
        "install": {"install_scripts": f"lib/{package_name}"},
        "develop": {"script_dir":     f"lib/{package_name}"},
    },
)
