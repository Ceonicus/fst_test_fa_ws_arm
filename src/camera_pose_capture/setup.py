from glob import glob
import os

from setuptools import setup


package_name = "camera_pose_capture"


setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README.md"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="FST FAIRINO User",
    maintainer_email="user@example.com",
    description="Safe RealSense camera pose capture workflow for FAIRINO MoveIt environments.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "capture_node = camera_pose_capture.capture_node:main",
        ],
    },
)
