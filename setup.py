from glob import glob
from setuptools import setup

package_name = "et_astlf_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    package_dir={"": "src"},
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README.md"]),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/examples", glob("examples/*.csv")),
        (f"share/{package_name}/launch", glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ros2 vehicle user",
    maintainer_email="user@example.com",
    description="ET-ASTLF path tracking controller for Ackermann ROS2 vehicles.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "et_astlf_node = et_astlf_control.node:main",
            "csv_path_publisher = et_astlf_control.csv_path_publisher:main",
        ],
    },
)
