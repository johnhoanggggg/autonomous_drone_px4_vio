from glob import glob
from setuptools import find_packages, setup

package_name = "px4_vio_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="john",
    maintainer_email="john@example.com",
    description="Bridge OAK-D Lite Basalt VIO poses into PX4 visual odometry over uXRCE-DDS.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "basalt_odometry_test = px4_vio_bridge.basalt_odometry_test:main",
            "offboard_hover = px4_vio_bridge.offboard_hover:main",
            "px4_local_position_to_ros = px4_vio_bridge.px4_local_position_to_ros:main",
            "vio_to_px4_odometry = px4_vio_bridge.vio_to_px4_odometry:main",
        ],
    },
)
