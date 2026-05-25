import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.actions import Shutdown
from launch.substitutions import TextSubstitution, LaunchConfiguration

def generate_launch_description():
    pkg_dir = get_package_share_directory('ros2_oak_ffc_sync')

    tf_prefix = LaunchConfiguration("tf_prefix", default="oak")
    camera_param_uri = LaunchConfiguration("camera_param_uri", default="package://ros2_oak_ffc_sync/params/ffc")
    mode = LaunchConfiguration("mode", default="depth")
    camera_name = LaunchConfiguration("camera_name", default="ar0234")  # ar0234 ov9782
    camera_type = LaunchConfiguration("camera_type", default="color")  # color mono
    resolution = LaunchConfiguration("resolution", default="1200p")
    fps = LaunchConfiguration("fps", default="40")
    oak_fw_uri = LaunchConfiguration("oak_fw_uri", default=os.path.join(pkg_dir, 'params/ffc', 'fw_REBOOT_FIX_CLEAN_2.30.mvcmd'))
    compressed = LaunchConfiguration("compressed", default="1")
    imu_hz = LaunchConfiguration("imu_hz", default="200")
    cam_board_sockets = LaunchConfiguration("cam_board_sockets", default="[CAM_A, CAM_B, CAM_C, CAM_D]")
    sync_threshold = LaunchConfiguration("sync_threshold", default="10")
    isp_scale = LaunchConfiguration("isp_scale", default="3")
    record_enable = LaunchConfiguration("record_enable", default="false")
    record_output_dir = LaunchConfiguration("record_output_dir", default="./oak_datasets")
    record_session_name = LaunchConfiguration("record_session_name", default="")
    record_imu_topic = LaunchConfiguration("record_imu_topic", default="/imu")
    record_pose_topic = LaunchConfiguration("record_pose_topic", default="/ov_msckf/odomimu")
    record_config_path = LaunchConfiguration("record_config_path", default="")
    record_video_fourcc = LaunchConfiguration("record_video_fourcc", default="MJPG")
    record_video_extension = LaunchConfiguration("record_video_extension", default=".avi")
    record_wait_for_imu_and_pose = LaunchConfiguration("record_wait_for_imu_and_pose", default="true")

    camera_model_arg = DeclareLaunchArgument(
        "camera_model", default_value=TextSubstitution(text="OAK-FFC-4P")
    )
    tf_prefix_arg = DeclareLaunchArgument(
        "tf_prefix", default_value=tf_prefix
    )
    mode_arg = DeclareLaunchArgument(
        "mode", default_value=mode
    )
    base_frame_arg = DeclareLaunchArgument(
        "base_frame", default_value=TextSubstitution(text="oak-d_frame")
    )
    parent_frame_arg = DeclareLaunchArgument(
        "parent_frame", default_value=TextSubstitution(text="oak-d-base-frame")
    )

    cam_pos_x_arg = DeclareLaunchArgument(
        "cam_pos_x", default_value=TextSubstitution(text="0.0")
    )

    cam_pos_y_arg = DeclareLaunchArgument(
        "cam_pos_y", default_value=TextSubstitution(text="0.0")
    )

    cam_pos_z_arg = DeclareLaunchArgument(
        "cam_pos_z", default_value=TextSubstitution(text="0.0")
    )

    cam_roll_arg = DeclareLaunchArgument(
        "cam_roll", default_value=TextSubstitution(text="0.0")
    )

    cam_pitch_arg = DeclareLaunchArgument(
        "cam_pitch", default_value=TextSubstitution(text="0.0")
    )

    cam_yaw_arg = DeclareLaunchArgument(
        "cam_yaw", default_value=TextSubstitution(text="0.0")
    )

    camera_name_arg = DeclareLaunchArgument(
        "camera_name", default_value=camera_name
    )

    camera_type_arg = DeclareLaunchArgument(
        "camera_type", default_value=camera_type
    )

    resolution_arg = DeclareLaunchArgument(
        "resolution", default_value=resolution
    )

    fps_arg = DeclareLaunchArgument(
        "fps", default_value=fps
    )

    camera_param_uri_arg = DeclareLaunchArgument(
        "camera_param_uri", default_value=camera_param_uri
    )

    oak_fw_uri_arg = DeclareLaunchArgument(
        "oak_fw_uri", default_value=oak_fw_uri
    )

    compressed_arg = DeclareLaunchArgument(
        "compressed", default_value=compressed
    )

    imu_hz_arg = DeclareLaunchArgument(
        "imu_hz", default_value=imu_hz
    )

    cam_board_sockets_arg = DeclareLaunchArgument(
        "cam_board_sockets", default_value=cam_board_sockets
    )

    sync_threshold_arg = DeclareLaunchArgument(
        "sync_threshold", default_value=sync_threshold
    )

    isp_scale_arg = DeclareLaunchArgument(
        "isp_scale", default_value=isp_scale
    )
    record_enable_arg = DeclareLaunchArgument("record_enable", default_value=record_enable)
    record_output_dir_arg = DeclareLaunchArgument("record_output_dir", default_value=record_output_dir)
    record_session_name_arg = DeclareLaunchArgument("record_session_name", default_value=record_session_name)
    record_imu_topic_arg = DeclareLaunchArgument("record_imu_topic", default_value=record_imu_topic)
    record_pose_topic_arg = DeclareLaunchArgument("record_pose_topic", default_value=record_pose_topic)
    record_config_path_arg = DeclareLaunchArgument("record_config_path", default_value=record_config_path)
    record_video_fourcc_arg = DeclareLaunchArgument("record_video_fourcc", default_value=record_video_fourcc)
    record_video_extension_arg = DeclareLaunchArgument("record_video_extension", default_value=record_video_extension)
    record_wait_for_imu_and_pose_arg = DeclareLaunchArgument(
        "record_wait_for_imu_and_pose", default_value=record_wait_for_imu_and_pose
    )

    oak_ffc_node = Node(
        package="ros2_oak_ffc_sync",
        executable="oak_ffc_sync",
        name="oak_ffc_sync_publisher",
        output="screen",
        parameters=[
            {"tf_prefix": tf_prefix},
            {"camera_param_uri": camera_param_uri},
            {"mode": mode},
            {"camera_name": camera_name},
            {"camera_type": camera_type},
            {"resolution": resolution},
            {"fps": fps},
            {"oak_fw_uri": oak_fw_uri},
            {"compressed": compressed},
            {"imu_hz": imu_hz},
            {"cam_board_sockets": cam_board_sockets},
            {"sync_threshold": sync_threshold},
            {"isp_scale": isp_scale},
            {"record_enable": record_enable},
            {"record_output_dir": record_output_dir},
            {"record_session_name": record_session_name},
            {"record_imu_topic": record_imu_topic},
            {"record_pose_topic": record_pose_topic},
            {"record_config_path": record_config_path},
            {"record_video_fourcc": record_video_fourcc},
            {"record_video_extension": record_video_extension},
            {"record_wait_for_imu_and_pose": record_wait_for_imu_and_pose},
        ],
        on_exit=Shutdown()
    )

    rviz_config = os.path.join(pkg_dir, 'rviz', 'oak_ffc_sync_publisher.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_config],
        output='screen'
    )

    return LaunchDescription([
        camera_model_arg,
        tf_prefix_arg,
        mode_arg,
        base_frame_arg,
        parent_frame_arg,
        cam_pos_x_arg,
        cam_pos_y_arg,
        cam_pos_z_arg,
        cam_roll_arg,
        cam_pitch_arg,
        cam_yaw_arg,
        camera_name_arg,
        camera_type_arg,
        resolution_arg,
        fps_arg,
        camera_param_uri_arg,
        oak_fw_uri_arg,
        compressed_arg,
        imu_hz_arg,
        cam_board_sockets_arg,
        sync_threshold_arg,
        isp_scale_arg,
        record_enable_arg,
        record_output_dir_arg,
        record_session_name_arg,
        record_imu_topic_arg,
        record_pose_topic_arg,
        record_config_path_arg,
        record_video_fourcc_arg,
        record_video_extension_arg,
        record_wait_for_imu_and_pose_arg,
        oak_ffc_node,
        rviz_node
    ])
