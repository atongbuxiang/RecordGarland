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
            {"isp_scale": isp_scale}
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
        oak_ffc_node,
        rviz_node
    ])
