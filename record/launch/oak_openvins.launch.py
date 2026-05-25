import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def _launch_setup(context):
    record_prefix = get_package_prefix("record")
    workspace_candidates = [
        os.getcwd(),
        os.path.dirname(os.path.dirname(record_prefix)),
        os.path.dirname(record_prefix),
    ]
    config_candidates = [
        os.path.join(path, "src", "configs", "estimator_config.yaml")
        for path in workspace_candidates
    ]
    default_config_path = next((path for path in config_candidates if os.path.isfile(path)), config_candidates[0])

    config_path = LaunchConfiguration("config_path").perform(context)
    if not config_path:
        config_path = default_config_path

    if LaunchConfiguration("ov_enable").perform(context).lower() not in ("true", "1", "yes", "on"):
        openvins_nodes = []
    else:
        openvins_nodes = [
            ComposableNode(
                package="ov_msckf",
                plugin="ov_msckf::SubscribeMsckfNode",
                name="run_subscribe_msckf",
                namespace=LaunchConfiguration("namespace"),
                extra_arguments=[{"use_intra_process_comms": True}],
                parameters=[
                    {"verbosity": LaunchConfiguration("verbosity")},
                    {"use_stereo": LaunchConfiguration("use_stereo")},
                    {"max_cameras": LaunchConfiguration("max_cameras")},
                    {"save_total_state": LaunchConfiguration("save_total_state")},
                    {"topic_imu": LaunchConfiguration("topic_imu")},
                    {"topic_camera0": LaunchConfiguration("topic_camera0")},
                    {"topic_camera1": LaunchConfiguration("topic_camera1")},
                    {"config_path": config_path},
                ],
            )
        ]

    oak_pkg_dir = get_package_share_directory("ros2_oak_ffc_sync")
    ov_pkg_dir = get_package_share_directory("ov_msckf")

    oak_fw_uri = LaunchConfiguration("oak_fw_uri").perform(context)
    if not oak_fw_uri:
        oak_fw_uri = os.path.join(oak_pkg_dir, "params", "ffc", "fw_REBOOT_FIX_CLEAN_2.30.mvcmd")

    camera_param_uri = LaunchConfiguration("camera_param_uri").perform(context)
    if not camera_param_uri:
        camera_param_uri = "package://ros2_oak_ffc_sync/params/ffc"

    container = ComposableNodeContainer(
        name="oak_openvins_container",
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
                    {"record_enable": LaunchConfiguration("record_enable")},
                    {"record_output_dir": LaunchConfiguration("record_output_dir")},
                    {"record_session_name": LaunchConfiguration("record_session_name")},
                    {"record_imu_topic": LaunchConfiguration("record_imu_topic")},
                    {"record_pose_topic": LaunchConfiguration("record_pose_topic")},
                    {"record_config_path": config_path},
                    {"record_video_fourcc": LaunchConfiguration("record_video_fourcc")},
                    {"record_video_extension": LaunchConfiguration("record_video_extension")},
                    {"record_wait_for_imu_and_pose": LaunchConfiguration("record_wait_for_imu_and_pose")},
                ],
            ),
            *openvins_nodes,
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        condition=IfCondition(LaunchConfiguration("rviz_enable")),
        arguments=[
            "-d" + os.path.join(ov_pkg_dir, "launch", "display_ros2.rviz"),
            "--ros-args",
            "--log-level",
            "warn",
        ],
        output="screen",
    )

    return [container, rviz_node]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value="ov_msckf"),
            DeclareLaunchArgument("ov_enable", default_value="true"),
            DeclareLaunchArgument("rviz_enable", default_value="true"),
            DeclareLaunchArgument("config_path", default_value=""),
            DeclareLaunchArgument("verbosity", default_value="INFO"),
            DeclareLaunchArgument("use_stereo", default_value="true"),
            DeclareLaunchArgument("max_cameras", default_value="2"),
            DeclareLaunchArgument("save_total_state", default_value="false"),
            DeclareLaunchArgument("topic_imu", default_value="/imu"),
            DeclareLaunchArgument("topic_camera0", default_value="/CAM_A/image"),
            DeclareLaunchArgument("topic_camera1", default_value="/CAM_B/image"),
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
            DeclareLaunchArgument("record_enable", default_value="true"),
            DeclareLaunchArgument("record_output_dir", default_value="./oak_datasets"),
            DeclareLaunchArgument("record_session_name", default_value="test"),
            DeclareLaunchArgument("record_imu_topic", default_value="/imu"),
            DeclareLaunchArgument("record_pose_topic", default_value="/ov_msckf/odomimu"),
            DeclareLaunchArgument("record_video_fourcc", default_value="MJPG"),
            DeclareLaunchArgument("record_video_extension", default_value=".avi"),
            DeclareLaunchArgument("record_wait_for_imu_and_pose", default_value="true"),
            OpaqueFunction(function=_launch_setup),
        ]
    )
