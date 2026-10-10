from setuptools import find_packages, setup

package_name = "vive_tracker_ros"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/vive_tracker.launch.py"]),
        (f"share/{package_name}/config", ["config/tracker.example.yaml"]),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "vive_tracker_node = vive_tracker_ros.node:main",
            "vive-tracker-serial = vive_tracker.serial_reader:main",
        ]
    },
)
