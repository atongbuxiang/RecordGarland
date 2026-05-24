import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context):
    workspace = os.getcwd()
    default_config_dir = os.path.join(workspace, "src", "configs")

    estimator_config_path = LaunchConfiguration("estimator_config_path").perform(context)
    if not estimator_config_path:
        estimator_config_path = os.path.join(default_config_dir, "estimator_config.yaml")

    imucam_config_path = LaunchConfiguration("imucam_config_path").perform(context)
    if not imucam_config_path:
        imucam_config_path = os.path.join(default_config_dir, "kalibr_imucam_chain.yaml")

    imu_config_path = LaunchConfiguration("imu_config_path").perform(context)
    if not imu_config_path:
        imu_config_path = os.path.join(default_config_dir, "kalibr_imu_chain.yaml")

    recorder = Node(
        package="record",
        executable="oak_dataset_recorder",
        name="oak_dataset_recorder",
        output="screen",
        parameters=[
            {"camera_names": ["CAM_A", "CAM_B", "CAM_C", "CAM_D"]},
            {"camera_topics": ["/CAM_A/image", "/CAM_B/image", "/CAM_C/image", "/CAM_D/image"]},
            {"imu_topic": LaunchConfiguration("imu_topic")},
            {"odom_topic": LaunchConfiguration("odom_topic")},
            {"output_dir": LaunchConfiguration("output_dir")},
            {"session_name": LaunchConfiguration("session_name")},
            {"sync_threshold_ms": LaunchConfiguration("sync_threshold_ms")},
            {"pose_lookahead_ms": LaunchConfiguration("pose_lookahead_ms")},
            {"pose_wait_timeout_ms": LaunchConfiguration("pose_wait_timeout_ms")},
            {"pose_buffer_sec": LaunchConfiguration("pose_buffer_sec")},
            {"video_fps": LaunchConfiguration("video_fps")},
            {"video_fourcc": LaunchConfiguration("video_fourcc")},
            {"video_extension": LaunchConfiguration("video_extension")},
            {"queue_warning_frames": LaunchConfiguration("queue_warning_frames")},
            {"estimator_config_path": estimator_config_path},
            {"imucam_config_path": imucam_config_path},
            {"imu_config_path": imu_config_path},
        ],
    )

    return [recorder]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("output_dir", default_value=os.path.expanduser("~/oak_datasets")),
            DeclareLaunchArgument("session_name", default_value=""),
            DeclareLaunchArgument("imu_topic", default_value="/imu"),
            DeclareLaunchArgument("odom_topic", default_value="/ov_msckf/odomimu"),
            DeclareLaunchArgument("sync_threshold_ms", default_value="10.0"),
            DeclareLaunchArgument("pose_lookahead_ms", default_value="20.0"),
            DeclareLaunchArgument("pose_wait_timeout_ms", default_value="150.0"),
            DeclareLaunchArgument("pose_buffer_sec", default_value="10.0"),
            DeclareLaunchArgument("video_fps", default_value="40.0"),
            DeclareLaunchArgument("video_fourcc", default_value="MJPG"),
            DeclareLaunchArgument("video_extension", default_value=".avi"),
            DeclareLaunchArgument("queue_warning_frames", default_value="1000"),
            DeclareLaunchArgument("estimator_config_path", default_value=""),
            DeclareLaunchArgument("imucam_config_path", default_value=""),
            DeclareLaunchArgument("imu_config_path", default_value=""),
            OpaqueFunction(function=_launch_setup),
        ]
    )
