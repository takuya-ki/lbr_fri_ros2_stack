import glob
from setuptools import setup

package_name = "lbr_moveit_py"

setup(
    name=package_name,
    version="2.2.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob.glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="takuya-ki",
    maintainer_email="taku8926@gmail.com",
    description="Demo for using MoveIt Python API.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "hello_moveit_action = lbr_moveit_py.hello_moveit_action:main",
            "hello_moveit_commander = lbr_moveit_py.hello_moveit_commander:main",
            "dual_hello_moveit = lbr_moveit_py.dual_hello_moveit:main",
        ],
    },
)
