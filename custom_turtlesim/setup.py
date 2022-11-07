from setuptools import setup

package_name = "custom_turtlesim"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="bbolte",
    maintainer_email="ben@bolte.cc",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "controller = custom_turtlesim.controller:run_turtlesim_controller",
        ],
    },
)
