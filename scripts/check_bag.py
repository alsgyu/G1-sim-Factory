"""Check a rosbag2 folder made by scripts/record_bag.sh before handing it over.

    bash scripts/g1_docker.sh python3 scripts/check_bag.py outputs/bags/g1bag_conveyor

Prints the storage, compression, size, wall-clock and simulated duration, and for every topic the message count,
its rate per simulated second (from the recorded /clock) and that rate against the simulator's nominal rate.
Exits 1 when metadata.yaml is missing, a topic has no messages, or a sensor stream has under 95 % of its messages.
"""
import sys
from pathlib import Path

import rosbag2_py
import yaml
from rclpy.serialization import deserialize_message
from rosgraph_msgs.msg import Clock

NOMINAL_HZ = {  # Isaac Sim and MuJoCo defaults (docs/ROS2_INTERFACE.md)
    "/camera/camera/color/image_raw": 30.0,
    "/camera/camera/aligned_depth_to_color/image_raw": 30.0,
    "/camera/camera/color/camera_info": 30.0,
    "/livox/lidar": 10.0,
    "/odom": 50.0,
    "/clock": 100.0,
}
EXPECTED = ["/camera/camera/color/image_raw", "/camera/camera/aligned_depth_to_color/image_raw", "/livox/lidar",
            "/camera/camera/color/camera_info", "/odom", "/tf", "/tf_static", "/clock"]


def sim_span(folder: Path, storage: str):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(folder), storage_id=storage),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    reader.set_filter(rosbag2_py.StorageFilter(topics=["/clock"]))
    first = last = None
    while reader.has_next():
        _, data, _ = reader.read_next()
        c = deserialize_message(data, Clock).clock
        t = c.sec+c.nanosec*1e-9
        first = t if first is None else first
        last = t
    return (first, last) if first is not None else None


def main():
    folder = Path(sys.argv[1])
    meta_path = folder/"metadata.yaml"
    if not meta_path.is_file():
        print(f"FAIL {meta_path} missing: the recorder was not stopped with Ctrl-C. "
              f"Repair: bash scripts/g1_docker.sh ros2 bag reindex -s sqlite3 {folder}")
        sys.exit(1)
    info = yaml.safe_load(meta_path.read_text())["rosbag2_bagfile_information"]
    storage = info["storage_identifier"]
    wall_s = info["duration"]["nanoseconds"]*1e-9
    size = sum(f.stat().st_size for f in folder.iterdir() if f.is_file())
    span = sim_span(folder, storage)
    sim_s = span[1]-span[0] if span else 0.0
    print(f"{folder}: storage {storage}, compression {info.get('compression_mode') or 'none'}, "
          f"{len(info['relative_file_paths'])} file(s), {size/1e9:.2f} GB")
    print(f"wall clock {wall_s:.1f} s, simulated {sim_s:.1f} s"
          + (f", real-time factor {sim_s/wall_s:.2f}" if wall_s > 0 and sim_s > 0 else ""))
    counts = {t["topic_metadata"]["name"]: (t["message_count"], t["topic_metadata"]["type"])
              for t in info["topics_with_message_count"]}
    problems = []
    print(f"{'topic':50s} {'type':32s} {'messages':>9s} {'Hz (sim)':>9s} {'of nominal':>10s}")
    for topic in EXPECTED+sorted(set(counts)-set(EXPECTED)):
        n, kind = counts.get(topic, (0, "-"))
        hz = n/sim_s if sim_s > 0 else 0.0
        nominal = NOMINAL_HZ.get(topic)
        share = f"{100*hz/nominal:.0f} %" if nominal and sim_s > 0 else "-"
        print(f"{topic:50s} {kind:32s} {n:9d} {hz:9.1f} {share:>10s}")
        if n == 0:
            problems.append(f"{topic} has no messages")
        elif nominal and sim_s > 0 and hz < 0.95*nominal:
            problems.append(f"{topic} has {100*hz/nominal:.0f} % of its {nominal:g} Hz messages")
    for p in problems:
        print("FAIL", p)
    print("OK: hand over the whole folder (metadata.yaml and every file next to it)" if not problems else
          "check the socket buffers (docs/UBUNTU_ISAAC.md section 5) and the disk, then record again")
    sys.exit(1 if problems else 0)


if __name__ == "__main__":
    main()
