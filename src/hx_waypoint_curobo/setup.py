from setuptools import find_packages, setup


package_name = "hx_waypoint_curobo"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "requirements.lock"]),
    ],
    install_requires=["setuptools", "numpy", "PyYAML", "scipy"],
    zip_safe=False,
    maintainer="Huaxin Hou",
    maintainer_email="hxhou0207@gmail.com",
    description="cuRobo V2 adapter and authoritative timed-pose trajectory compiler",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "curobo_server = hx_waypoint_curobo.curobo_server:main",
        ]
    },
)
