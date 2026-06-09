import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def _default_config_path():
    candidates = [
        os.path.join(os.getcwd(), "src", "configs", "estimator_config.yaml"),
        os.path.join(os.path.dirname(os.getcwd()), "src", "configs", "estimator_config.yaml"),
    ]
    return next((path for path in candidates if os.path.isfile(path)), candidates[0])


def _launch_setup(context):
    oak_pkg_dir = get_package_share_directory("ros2_oak_ffc_sync")
    config_path = LaunchConfiguration("config_path").perform(context) or _default_config_path()
    oak_fw_uri = LaunchConfiguration("oak_fw_uri").perform(context)
    if not oak_fw_uri:
        oak_fw_uri = os.path.join(oak_pkg_dir, "params", "ffc", "fw_REBOOT_FIX_CLEAN_2.30.mvcmd")

    camera_param_uri = LaunchConfiguration("camera_param_uri").perform(context)
    if not camera_param_uri:
        camera_param_uri = "package://ros2_oak_ffc_sync/params/ffc"

    return [
        ComposableNodeContainer(
            name="oak_raw_record_container",
            namespace="",
            package="rclcpp_components",
            executable="component_container_mt",
            output="screen",
            composable_node_descriptions=[
                ComposableNode(
                    package="ros2_oak_ffc_sync",
                    plugin="OakFfcSyncNode",
                    name="oak_ffc_sync_publisher",
                    extra_arguments=[{"use_intra_process_comms": True}],
                    parameters=[
                        {"tf_prefix": LaunchConfiguration("tf_prefix")},
                        {"camera_param_uri": camera_param_uri},
                        {"mode": LaunchConfiguration("mode")},
                        {"camera_name": LaunchConfiguration("camera_name")},
                        {"camera_type": LaunchConfiguration("camera_type")},
                        {"resolution": LaunchConfiguration("resolution")},
                        {"fps": LaunchConfiguration("fps")},
                        {"oak_fw_uri": oak_fw_uri},
                        {"compressed": LaunchConfiguration("compressed")},
                        {"imu_hz": LaunchConfiguration("imu_hz")},
                        {"cam_board_sockets": ["CAM_A", "CAM_B", "CAM_C", "CAM_D"]},
                        {"sync_threshold": LaunchConfiguration("sync_threshold")},
                        {"isp_scale": LaunchConfiguration("isp_scale")},
                        {"record_enable": True},
                        {"record_output_dir": LaunchConfiguration("record_output_dir")},
                        {"record_session_name": LaunchConfiguration("record_session_name")},
                        {"record_imu_topic": "/imu"},
                        {"record_pose_topic": "/ov_msckf/odomimu"},
                        {"record_config_path": config_path},
                        {"record_video_fourcc": LaunchConfiguration("record_video_fourcc")},
                        {"record_video_extension": LaunchConfiguration("record_video_extension")},
                        {"record_wait_for_imu_and_pose": False},
                    ],
                )
            ],
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("config_path", default_value=""),
            DeclareLaunchArgument("tf_prefix", default_value="oak"),
            DeclareLaunchArgument("mode", default_value="depth"),
            DeclareLaunchArgument("camera_name", default_value="ar0234"),
            DeclareLaunchArgument("camera_type", default_value="color"),
            DeclareLaunchArgument("resolution", default_value="1200p"),
            DeclareLaunchArgument("fps", default_value="40"),
            DeclareLaunchArgument("camera_param_uri", default_value=""),
            DeclareLaunchArgument("oak_fw_uri", default_value=""),
            DeclareLaunchArgument("compressed", default_value="1"),
            DeclareLaunchArgument("imu_hz", default_value="200"),
            DeclareLaunchArgument("sync_threshold", default_value="20"),
            DeclareLaunchArgument("isp_scale", default_value="3"),
            DeclareLaunchArgument("record_output_dir", default_value="./oak_datasets"),
            DeclareLaunchArgument("record_session_name", default_value="demo"),
            DeclareLaunchArgument("record_video_fourcc", default_value="MJPG"),
            DeclareLaunchArgument("record_video_extension", default_value=".avi"),
            OpaqueFunction(function=_launch_setup),
        ]
    )
