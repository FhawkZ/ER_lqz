from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    task = LaunchConfiguration("task")
    recording_id = LaunchConfiguration("recording_id")
    output_dir = LaunchConfiguration("output_dir")
    return LaunchDescription(
        [
            DeclareLaunchArgument("task", default_value="pick the red cube and drop it in box"),
            DeclareLaunchArgument("recording_id", default_value="manual"),
            DeclareLaunchArgument("output_dir", default_value="/tmp/fr3dex_records"),
            Node(
                package="fr3_dexterous_platform_ros",
                executable="fr3dex_episode_marker",
                output="screen",
                parameters=[{"task": task, "recording_id": recording_id}],
            ),
            Node(package="fr3_dexterous_platform_ros", executable="fr3dex_topic_bridge", output="screen"),
            Node(
                package="fr3_dexterous_platform_ros",
                executable="fr3dex_recording_manifest",
                output="screen",
                parameters=[{"task": task, "recording_id": recording_id, "output_dir": output_dir}],
            ),
        ]
    )
