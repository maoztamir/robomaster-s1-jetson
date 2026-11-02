from setuptools import setup


PACKAGE_NAME = "s1_chassis_driver"


setup(
    name=PACKAGE_NAME,
    version="0.1.0",
    packages=["s1_can"],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{PACKAGE_NAME}"]),
        (f"share/{PACKAGE_NAME}", ["package.xml"]),
        (f"share/{PACKAGE_NAME}/config", ["config/s1_chassis.yaml"]),
    ],
    install_requires=["setuptools", "python-can>=4.3,<5", "PyYAML>=6,<7"],
    zip_safe=True,
    maintainer="maoztamir",
    maintainer_email="maoztamir@users.noreply.github.com",
    description="Direct SocketCAN ROS 2 driver for the RoboMaster S1 chassis.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "s1_chassis_node = s1_can.ros_node:main",
            "robomaster_patrol = s1_can.patrol_node:main",
            "robomaster_sensor_power = s1_can.sensor_power_node:main",
        ],
    },
)
