import importlib.util
from pathlib import Path


def load_replay_module():
    module_path = Path(__file__).resolve().parents[1] / "scripts" / "oak_dataset_replay_openvins.py"
    spec = importlib.util.spec_from_file_location("oak_dataset_replay_openvins", module_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_add_poses_to_frame_groups_matches_nearest_pose_without_mutating_input():
    replay = load_replay_module()
    frames = {
        "pose_topic": "/ov_msckf/odomimu",
        "pose_format": "xyz_qxyzw",
        "camera_order": ["CAM_A", "CAM_B", "CAM_C", "CAM_D"],
        "frames": [
            {"i": 0, "t": 10.0, "t_cam": [10.0, 10.0, 10.0, 10.0], "video_i": [0, 0, 0, 0]},
            {"i": 1, "t": 10.1, "t_cam": [10.1, 10.1, 10.1, 10.1], "video_i": [1, 1, 1, 1]},
        ],
    }
    poses = [
        {"t": 9.99, "xyz_qxyzw": [1, 2, 3, 0, 0, 0, 1]},
        {"t": 10.12, "xyz_qxyzw": [4, 5, 6, 0.1, 0.2, 0.3, 0.9]},
    ]

    output = replay.add_poses_to_frame_groups(frames, poses)

    assert frames["frames"][0].get("pose") is None
    assert output["frames"][0]["pose"] == {
        "t": 9.99,
        "dt": -0.01,
        "xyz_qxyzw": [1, 2, 3, 0, 0, 0, 1],
    }
    assert output["frames"][1]["pose"] == {
        "t": 10.12,
        "dt": 0.02,
        "xyz_qxyzw": [4, 5, 6, 0.1, 0.2, 0.3, 0.9],
    }


def test_add_poses_to_frame_groups_allows_missing_poses():
    replay = load_replay_module()
    frames = {"frames": [{"i": 0, "t": 1.0, "video_i": [0, 0, 0, 0]}]}

    output = replay.add_poses_to_frame_groups(frames, [])

    assert output["frames"][0]["pose"] is None
