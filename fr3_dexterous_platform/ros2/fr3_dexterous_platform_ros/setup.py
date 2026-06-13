from setuptools import find_packages, setup

package_name = "fr3_dexterous_platform_ros"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/fr3_collect.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ER_lqz",
    maintainer_email="user@example.com",
    description="ROS2 helper nodes for FR3 Dexterous Platform.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "fr3dex_episode_marker = fr3_dexterous_platform_ros.episode_marker_node:main",
            "fr3dex_topic_bridge = fr3_dexterous_platform_ros.topic_bridge_node:main",
            "fr3dex_recording_manifest = fr3_dexterous_platform_ros.recording_manifest_node:main",
        ],
    },
)
