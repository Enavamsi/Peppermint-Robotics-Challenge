"""
setup.py — color_nav
====================
Single-package visual-servoing stack for TurtleBot3.
"""

from setuptools import find_packages, setup
import os
from glob import glob

PACKAGE_NAME = "color_nav"

setup(
    name=PACKAGE_NAME,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),

    data_files=[
        # ament index marker
        (
            "share/ament_index/resource_index/packages",
            [f"resource/{PACKAGE_NAME}"],
        ),
        # package manifest
        (f"share/{PACKAGE_NAME}", ["package.xml"]),
        # launch files
        (
            os.path.join("share", PACKAGE_NAME, "launch"),
            glob("launch/*.py"),
        ),
        # Gazebo worlds
        (
            os.path.join("share", PACKAGE_NAME, "worlds"),
            glob("worlds/*.world"),
        ),
    ],

    install_requires=["setuptools"],
    zip_safe=True,

    maintainer="Vulavakattu Ena Vamsi",
    maintainer_email="enavamsi99@gmail.comm",
    description=(
        "Colour-based visual servoing for TurtleBot3 — "
        "HSV segmentation, P-controller steering, LiDAR safety stop."
    ),
    license="Apache-2.0",

    extras_require={
        "test": ["pytest", "pytest-mock"],
    },

    entry_points={
        "console_scripts": [
            # ros2 run color_nav visual_servoing
            "visual_servoing = color_nav.visual_servoing:main",
        ],
    },
)
