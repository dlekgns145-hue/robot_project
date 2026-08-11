import os
from glob import glob

from setuptools import find_packages, setup


PACKAGE_NAME = "orchard_mapper"


setup(
    name=PACKAGE_NAME,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [f"resource/{PACKAGE_NAME}"],
        ),
        (f"share/{PACKAGE_NAME}", ["package.xml", "README.md"]),
        (os.path.join("share", PACKAGE_NAME, "config"), glob("config/*.yaml")),
        (os.path.join("share", PACKAGE_NAME, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="yorong",
    maintainer_email="yorong@example.com",
    description="Replayable camera BEV map aligned with slam_toolbox coordinates",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "camera_bev_node = orchard_mapper.camera_bev_node:main",
            "global_visual_mapper = orchard_mapper.global_visual_mapper:main",
            "visual_map_saver = orchard_mapper.map_saver:main",
        ]
    },
)
