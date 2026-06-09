#!/usr/bin/env python3
import argparse
import bisect
import copy
import json
import math
import os
import time
from pathlib import Path

import cv2

try:
    import rclpy
    from builtin_interfaces.msg import Time
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Image, Imu
except ImportError:
    rclpy = None
    Node = object


CAMERA_TOPICS = {
    "CAM_A": "/CAM_A/image",
    "CAM_B": "/CAM_B/image",
}


def stamp_from_seconds(value):
    sec = math.floor(value)
    nanosec = int(round((value - sec) * 1e9))
    if nanosec >= 1000000000:
        sec += 1
        nanosec -= 1000000000
    stamp = Time()
    stamp.sec = int(sec)
    stamp.nanosec = int(nanosec)
    return stamp


def load_json(path):
    with open(path, "r", encoding="utf-8") as stream:
        return json.load(stream)


def write_json(path, data):
    with open(path, "w", encoding="utf-8") as stream:
        json.dump(data, stream, indent=2)
        stream.write("\n")


def nearest_pose(timestamp, poses, pose_times):
    if not poses:
        return None
    index = bisect.bisect_left(pose_times, timestamp)
    candidates = []
    if index < len(poses):
        candidates.append(poses[index])
    if index > 0:
        candidates.append(poses[index - 1])
    return min(candidates, key=lambda pose: abs(pose["t"] - timestamp))


def add_poses_to_frame_groups(frame_data, poses):
    output = copy.deepcopy(frame_data)
    pose_times = [pose["t"] for pose in poses]
    for frame in output.get("frames", []):
        pose = nearest_pose(frame["t"], poses, pose_times)
        if pose is None:
            frame["pose"] = None
            continue
        dt = pose["t"] - frame["t"]
        frame["pose"] = {
            "t": pose["t"],
            "dt": round(dt, 12),
            "xyz_qxyzw": pose["xyz_qxyzw"],
        }
    return output


class OakDatasetReplayNode(Node):
    def __init__(self):
        super().__init__("oak_dataset_replay_openvins")
        self.dataset_dir = Path(self.declare_parameter("dataset_dir", "./oak_datasets/demo").value).expanduser()
        self.replay_rate = float(self.declare_parameter("replay_rate", 1.0).value)
        self.pose_output = self.declare_parameter("pose_output", "video_frames_with_pose.json").value
        self.pose_topic = self.declare_parameter("pose_topic", "/ov_msckf/odomimu").value
        self.frame_id_prefix = self.declare_parameter("frame_id_prefix", "oak").value
        self.start_delay_sec = float(self.declare_parameter("start_delay_sec", 1.0).value)

        if self.replay_rate <= 0.0:
            raise ValueError("replay_rate must be positive")

        self.frame_data = load_json(self.dataset_dir / "video_frames.json")
        self.imu_data = load_json(self.dataset_dir / "imu.json")
        self.poses = []

        qos = QoSProfile(
            depth=10,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.image_publishers = {
            name: self.create_publisher(Image, topic, qos)
            for name, topic in CAMERA_TOPICS.items()
        }
        self.imu_publisher = self.create_publisher(Imu, "/imu", qos)
        self.pose_subscription = self.create_subscription(Odometry, self.pose_topic, self.pose_callback, 100)

    def pose_callback(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        pose = msg.pose.pose
        self.poses.append({
            "t": t,
            "xyz_qxyzw": [
                pose.position.x,
                pose.position.y,
                pose.position.z,
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ],
        })

    def make_image_msg(self, frame, stamp, camera_name):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        msg = Image()
        msg.header.stamp = stamp_from_seconds(stamp)
        msg.header.frame_id = f"{self.frame_id_prefix}_{camera_name}_optical_frame"
        msg.height = gray.shape[0]
        msg.width = gray.shape[1]
        msg.encoding = "mono8"
        msg.is_bigendian = False
        msg.step = gray.shape[1]
        msg.data = gray.tobytes()
        return msg

    def make_imu_msg(self, item):
        msg = Imu()
        stamp = item.get("t", item.get("stamp"))
        msg.header.stamp = stamp_from_seconds(float(stamp))
        msg.header.frame_id = f"{self.frame_id_prefix}_imu_frame"
        angular = item["angular_velocity"]
        accel = item["linear_acceleration"]
        orientation = item.get("orientation", [0.0, 0.0, 0.0, 1.0])
        msg.angular_velocity.x = float(angular[0])
        msg.angular_velocity.y = float(angular[1])
        msg.angular_velocity.z = float(angular[2])
        msg.linear_acceleration.x = float(accel[0])
        msg.linear_acceleration.y = float(accel[1])
        msg.linear_acceleration.z = float(accel[2])
        msg.orientation.x = float(orientation[0])
        msg.orientation.y = float(orientation[1])
        msg.orientation.z = float(orientation[2])
        msg.orientation.w = float(orientation[3])
        return msg

    def open_videos(self):
        videos = {}
        for name in CAMERA_TOPICS:
            path = self.dataset_dir / "videos" / f"{name}.avi"
            cap = cv2.VideoCapture(str(path))
            if not cap.isOpened():
                raise RuntimeError(f"failed to open video: {path}")
            videos[name] = cap
        return videos

    def publish_frame_group(self, videos, frame_group):
        camera_order = self.frame_data.get("camera_order", ["CAM_A", "CAM_B", "CAM_C", "CAM_D"])
        for camera_name in CAMERA_TOPICS:
            camera_index = camera_order.index(camera_name)
            frame_index = int(frame_group["video_i"][camera_index])
            videos[camera_name].set(cv2.CAP_PROP_POS_FRAMES, frame_index)
            ok, frame = videos[camera_name].read()
            if not ok:
                raise RuntimeError(f"failed to read {camera_name} frame {frame_index}")
            stamp = float(frame_group["t_cam"][camera_index])
            self.image_publishers[camera_name].publish(self.make_image_msg(frame, stamp, camera_name))

    def replay(self):
        time.sleep(self.start_delay_sec)
        imu_messages = self.imu_data.get("messages", [])
        frame_groups = self.frame_data.get("frames", [])
        events = []
        for item in imu_messages:
            stamp = float(item.get("t", item.get("stamp")))
            events.append((stamp, "imu", item))
        for frame in frame_groups:
            events.append((float(frame["t"]), "frame", frame))
        events.sort(key=lambda event: event[0])

        if not events:
            self.get_logger().warning("dataset has no replay events")
            self.finish()
            return

        videos = self.open_videos()
        try:
            last_stamp = events[0][0]
            for stamp, kind, payload in events:
                if not rclpy.ok():
                    break
                sleep_time = max(0.0, (stamp - last_stamp) / self.replay_rate)
                if sleep_time > 0.0:
                    time.sleep(sleep_time)
                if kind == "imu":
                    self.imu_publisher.publish(self.make_imu_msg(payload))
                else:
                    self.publish_frame_group(videos, payload)
                last_stamp = stamp
                rclpy.spin_once(self, timeout_sec=0.0)
        finally:
            for cap in videos.values():
                cap.release()
        time.sleep(0.5)
        rclpy.spin_once(self, timeout_sec=0.0)
        self.finish()

    def finish(self):
        self.poses.sort(key=lambda pose: pose["t"])
        output = add_poses_to_frame_groups(self.frame_data, self.poses)
        output_path = Path(self.pose_output)
        if not output_path.is_absolute():
            output_path = self.dataset_dir / output_path
        write_json(output_path, output)
        self.get_logger().info(f"wrote pose-annotated frames to {output_path}")


def main():
    if rclpy is None:
        parser = argparse.ArgumentParser()
        parser.add_argument("--dataset-dir", default="./oak_datasets/demo")
        parser.add_argument("--pose-output", default="video_frames_with_pose.json")
        args = parser.parse_args()
        frame_data = load_json(Path(args.dataset_dir) / "video_frames.json")
        write_json(Path(args.dataset_dir) / args.pose_output, add_poses_to_frame_groups(frame_data, []))
        return

    rclpy.init()
    node = OakDatasetReplayNode()
    try:
        node.replay()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
