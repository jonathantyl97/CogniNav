from setuptools import find_packages, setup

package_name = "cogninav_viz"

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
    description="Iridescence viewer for CogniNav SLAM",
    license="MIT",
    entry_points={
        "console_scripts": [
            "iridescence_viewer = cogninav_viz.iridescence_viewer_node:main",
        ],
    },
)
