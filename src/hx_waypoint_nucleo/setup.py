# SPDX-License-Identifier: Apache-2.0
from setuptools import find_packages, setup


package_name = "hx_waypoint_nucleo"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools", "pyserial>=3.5"],
    zip_safe=False,
    maintainer="Huaxin Hou",
    maintainer_email="hxhou0207@gmail.com",
    description="Optional synchronized Nucleo magnetic-sensor capture sidecar",
    license="Apache-2.0",
)
