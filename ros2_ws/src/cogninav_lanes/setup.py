from setuptools import find_packages, setup

package_name = "cogninav_lanes"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jonathan-tay",
    maintainer_email="jonathantyl97@gmail.com",
    description="Lightweight lane detection for CogniNav",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "lane_detector = cogninav_lanes.lane_node:main",
            "corridor_monitor = cogninav_lanes.corridor_monitor_node:main",
        ],
    },
)
