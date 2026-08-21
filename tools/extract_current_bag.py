#!/usr/bin/env python3
"""Extract one static-scene PointCloud2 and TF messages from a ROS 2 sqlite bag.

Avoids DDS playback so it works in this offline workspace.  The bag was
recorded while the robot and scene were stationary, so one cloud and the
latest transforms in the same bag are a valid paired sample.
"""

import pickle
import sqlite3
import struct
from pathlib import Path

import numpy as np


class Cdr:
    def __init__(self, data):
        # ROS 2 messages use 4-byte CDR encapsulation header; all offsets here
        # are measured in the remaining CDR stream.
        self.data = memoryview(data)[4:]
        self.i = 0

    def align(self, size):
        self.i = (self.i + size - 1) & ~(size - 1)

    def u8(self):
        self.align(1)
        out = self.data[self.i]
        self.i += 1
        return out

    def u32(self):
        self.align(4)
        out = struct.unpack_from("<I", self.data, self.i)[0]
        self.i += 4
        return out

    def i32(self):
        self.align(4)
        out = struct.unpack_from("<i", self.data, self.i)[0]
        self.i += 4
        return out

    def f64(self):
        self.align(8)
        out = struct.unpack_from("<d", self.data, self.i)[0]
        self.i += 8
        return out

    def string(self):
        length = self.u32()
        if length == 0:
            return ""
        raw = bytes(self.data[self.i:self.i + length])
        self.i += length
        if raw[-1:] == b"\0":
            raw = raw[:-1]
        return raw.decode("utf-8")


def header(cdr):
    return {"sec": cdr.i32(), "nanosec": cdr.u32(), "frame_id": cdr.string()}


def transform(cdr):
    h = header(cdr)
    child = cdr.string()
    xyz = [cdr.f64(), cdr.f64(), cdr.f64()]
    xyzw = [cdr.f64(), cdr.f64(), cdr.f64(), cdr.f64()]
    return {"parent": h["frame_id"], "child": child, "xyz": xyz, "xyzw": xyzw}


def tf_message(data):
    cdr = Cdr(data)
    return [transform(cdr) for _ in range(cdr.u32())]


def pointcloud(data):
    cdr = Cdr(data)
    h = header(cdr)
    height, width = cdr.u32(), cdr.u32()
    fields = []
    for _ in range(cdr.u32()):
        fields.append({"name": cdr.string(), "offset": cdr.u32(),
                       "datatype": cdr.u8(), "count": cdr.u32()})
    big_endian = bool(cdr.u8())
    point_step, row_step = cdr.u32(), cdr.u32()
    length = cdr.u32()
    raw = bytes(cdr.data[cdr.i:cdr.i + length])
    cdr.i += length
    dense = bool(cdr.u8())
    if big_endian:
        raise ValueError("big-endian PointCloud2 not supported")
    field_map = {x["name"]: x for x in fields}
    required = [field_map[k] for k in ("x", "y", "z")]
    if any(x["datatype"] != 7 for x in required):  # FLOAT32
        raise ValueError(f"unexpected XYZ datatypes: {required}")
    count = height * width
    points = np.empty((count, 3), dtype=np.float32)
    for col, field in enumerate(required):
        points[:, col] = np.ndarray((count,), dtype="<f4", buffer=raw,
                                    offset=field["offset"], strides=(point_step,))
    points = points[np.all(np.isfinite(points), axis=1)]
    return {"stamp": h, "height": height, "width": width,
            "point_step": point_step, "row_step": row_step, "is_dense": dense,
            "points": points}


def joint_state(data):
    cdr = Cdr(data)
    h = header(cdr)
    names = [cdr.string() for _ in range(cdr.u32())]

    def doubles():
        count = cdr.u32()
        return [cdr.f64() for _ in range(count)]

    return {"stamp": h, "name": names, "position": doubles(),
            "velocity": doubles(), "effort": doubles()}


def main():
    bag = Path("robot_data/tf_analysis_current/tf_analysis_current_0.db3")
    out = Path("robot_data/current_bag")
    out.mkdir(parents=True, exist_ok=True)
    db = sqlite3.connect(bag)
    topics = {name: ident for ident, name in db.execute("SELECT id, name FROM topics")}
    rows = list(db.execute("SELECT topic_id, timestamp, data FROM messages ORDER BY timestamp"))

    static, dynamic, clouds, joints = [], [], [], []
    for topic_id, timestamp, data in rows:
        if topic_id == topics["/tf_static"]:
            static.extend(tf_message(data))
        elif topic_id == topics["/tf"]:
            dynamic.extend(tf_message(data))
        elif topic_id == topics["/sensor/camera/stereo/pointcloud/raw"]:
            clouds.append((timestamp, pointcloud(data)))
        elif topic_id == topics["/mc/joint_states"]:
            joints.append((timestamp, joint_state(data)))
    # Static scene: use middle cloud to avoid recorder start/stop boundaries.
    timestamp, cloud = clouds[len(clouds) // 2]
    # Deduplicate by child frame; last dynamic message is the current pose.
    dynamic_latest = {item["child"]: item for item in dynamic}
    static_latest = {item["child"]: item for item in static}
    with (out / "cloud.pkl").open("wb") as stream:
        pickle.dump(cloud, stream)
    with (out / "tf_static.pkl").open("wb") as stream:
        pickle.dump(list(static_latest.values()), stream)
    with (out / "tf_dynamic.pkl").open("wb") as stream:
        pickle.dump(list(dynamic_latest.values()), stream)
    closest_joint = min(joints, key=lambda item: abs(item[0] - timestamp))[1]
    with (out / "joint_state.pkl").open("wb") as stream:
        pickle.dump(closest_joint, stream)
    print(f"cloud_timestamp_ns={timestamp}")
    print(f"cloud_stamp={cloud['stamp']['sec']}.{cloud['stamp']['nanosec']:09d}")
    print(f"frame={cloud['stamp']['frame_id']} shape={cloud['height']}x{cloud['width']} valid_points={len(cloud['points'])}")
    print(f"tf_static={len(static_latest)} tf_dynamic={len(dynamic_latest)}")
    positions = dict(zip(closest_joint["name"], closest_joint["position"]))
    print("nearest_joint_stamp="
          f"{closest_joint['stamp']['sec']}.{closest_joint['stamp']['nanosec']:09d}")
    print("head/waist joints:", {name: positions.get(name) for name in
          ("head_pitch_joint", "head_yaw_joint", "waist_pitch_joint", "waist_yaw_joint")})
    print("dynamic frames:", ", ".join(sorted(dynamic_latest)))


if __name__ == "__main__":
    main()
