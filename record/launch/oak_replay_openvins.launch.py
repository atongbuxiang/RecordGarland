import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _default_config_path():
    record_prefix = get_package_prefix("record")
    workspace_candidates = [
        os.getcwd(),
        os.path.dirname(os.path.dirname(record_prefix)),
        os.path.dirname(record_prefix),
    ]
    candidates = [
        os.path.join(path, "src", "configs", "estimator_config.yaml")
        for path in workspace_candidates
    ]
    return next((path for path in candidates if os.path.isfile(path)), candidates[0])


def _launch_setup(context):
    config_path = LaunchConfiguration("config_path").perform(context) or _default_config_path()
    if not os.path.isfile(config_path):
        return [LogInfo(msg=f"ERROR: config_path file does not exist: {config_path}")]

    replay_node = Node(
        package="record",
        executable="oak_dataset_replay_openvins.py",
        name="oak_dataset_replay_openvins",
        output="screen",
        parameters=[
            {"dataset_dir": LaunchConfiguration("dataset_dir")},
            {"replay_rate": LaunchConfiguration("replay_rate")},
            {"pose_output": LaunchConfiguration("pose_output")},
            {"pose_topic": LaunchConfiguration("pose_topic")},
            {"start_delay_sec": LaunchConfiguration("start_delay_sec")},
        ],
    )

    openvins_node = Node(
        package="ov_msckf",
        executable="run_subscribe_msckf",
        condition=IfCondition(LaunchConfiguration("ov_enable")),
        namespace=LaunchConfiguration("namespace"),
        output="screen",
        parameters=[
            {"verbosity": LaunchConfiguration("verbosity")},
            {"use_stereo": True},
            {"max_cameras": 2},
            {"save_total_state": LaunchConfiguration("save_total_state")},
            {"topic_imu": "/imu"},
            {"topic_camera0": "/CAM_A/image"},
            {"topic_camera1": "/CAM_B/image"},
            {"config_path": config_path},
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        condition=IfCondition(LaunchConfiguration("rviz_enable")),
        arguments=[
            "-d" + os.path.join(get_package_share_directory("ov_msckf"), "launch", "display_ros2.rviz"),
            "--ros-args",
            "--log-level",
            "warn",
        ],
        output="screen",
    )

    return [openvins_node, replay_node, rviz_node]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("dataset_dir", default_value="./oak_datasets/demo"),
            DeclareLaunchArgument("config_path", default_value=""),
            DeclareLaunchArgument("replay_rate", default_value="1.0"),
            DeclareLaunchArgument("pose_output", default_value="video_frames_with_pose.json"),
            DeclareLaunchArgument("pose_topic", default_value="/ov_msckf/odomimu"),
            DeclareLaunchArgument("start_delay_sec", default_value="1.0"),
            DeclareLaunchArgument("namespace", default_value="ov_msckf"),
            DeclareLaunchArgument("ov_enable", default_value="true"),
            DeclareLaunchArgument("rviz_enable", default_value="true"),
            DeclareLaunchArgument("verbosity", default_value="INFO"),
            DeclareLaunchArgument("save_total_state", default_value="false"),
            OpaqueFunction(function=_launch_setup),
        ]
    )
