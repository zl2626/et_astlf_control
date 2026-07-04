from setuptools import setup

package_name = "et_astlf_path_tracking"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README.md"]),
        (f"share/{package_name}/config", ["config/et_astlf_params.yaml"]),
        (f"share/{package_name}/launch", ["launch/et_astlf_controller.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ros2 vehicle user",
    maintainer_email="user@example.com",
    description="ASTLF path-tracking controller for Ackermann ROS2 vehicles.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "et_astlf_controller_node = et_astlf_path_tracking.et_astlf_controller_node:main",
        ],
    },
)
