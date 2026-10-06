from setuptools import find_packages, setup

package_name = "rnc_demo"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/talker_listener.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Lucas David",
    maintainer_email="lucas@lucas-david-ry.com",
    description="Publisher and subscriber on a plain std_msgs/String topic.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "talker = rnc_demo.talker:main",
            "listener = rnc_demo.listener:main",
        ],
    },
)
