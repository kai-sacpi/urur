from setuptools import find_packages, setup


package_name = "hx_waypoint_tools"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools", "numpy", "PyYAML"],
    zip_safe=False,
    maintainer="Huaxin Hou",
    maintainer_email="hxhou0207@gmail.com",
    description="CLI and desktop controls for the hx waypoint workflow",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "tipdraw = hx_waypoint_tools.cli:main",
            "static-grid-capture = hx_waypoint_tools.static_grid_capture:main",
            "tcp-console = hx_waypoint_tools.tcp_console:main",
            "tcp-gui = hx_waypoint_tools.tcp_gui:main",
            (
                "hx_waypoint_planning_scene = "
                "hx_waypoint_tools.planning_scene_publisher:main"
            ),
        ]
    },
)
