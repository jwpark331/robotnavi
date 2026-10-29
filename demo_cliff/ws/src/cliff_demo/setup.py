from glob import glob

from setuptools import setup

package_name = "cliff_demo"
setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
        ("share/" + package_name + "/urdf", glob("urdf/*")),
        ("share/" + package_name + "/worlds", glob("worlds/*")),
        ("share/" + package_name + "/params", glob("params/*")),
        ("share/" + package_name + "/maps", glob("maps/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "cliff_detector = cliff_demo.cliff_detector:main",
            "scenario = cliff_demo.scenario:main",
            "map_builder = cliff_demo.map_builder:main",
            "ir_guard = cliff_demo.ir_guard:main",
        ],
    },
)
