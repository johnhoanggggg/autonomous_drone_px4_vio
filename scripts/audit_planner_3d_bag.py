#!/usr/bin/env python3
"""Read-only replay of accepted 3D routes and follower chords against recorded OctoMaps."""
import argparse
import json
from pathlib import Path
import subprocess


def audit(bag):
    import rosbag2_py
    from ament_index_python.packages import get_package_prefix
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from collections import Counter

    binary = Path(get_package_prefix("px4_vio_bridge")) / "lib/px4_vio_bridge/octomap_path_audit"
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id="mcap"),
                rosbag2_py.ConverterOptions("", ""))
    selected = {"/rtabmap/octomap", "/rtabmap/octomap_metadata", "/planner3d/path_snapshot",
                "/planner3d/follower/status", "/planner3d/reference_snapshot"}
    types = {item.name: get_message(item.type) for item in reader.get_all_topics_and_types()
             if item.name in selected}
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(selected)))
    pending_maps, metadata, checks = {}, {}, []
    seen_maps, seen_metadata = set(), set()
    counts, states, errors = Counter(), Counter(), []
    test_clear_generations = set()
    process = subprocess.Popen([str(binary)], stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True)

    def send(record):
        process.stdin.write(json.dumps(record, separators=(",", ":")) + "\n")
        process.stdin.flush()
        line = process.stdout.readline()
        if not line:
            raise RuntimeError("OctoMap audit process exited")
        result = json.loads(line)
        if not result["valid"]:
            errors.append({"kind": record["kind"], "generation": record["generation"], **result})
        counts["segments"] += result.get("segments", 0)
        counts["unverified_reference_segments"] += result.get("unverified_reference_segments", 0)

    def flush():
        for stamp in sorted(set(pending_maps) & set(metadata)):
            msg = pending_maps.pop(stamp)
            meta = metadata[stamp]
            send({"kind": "map", "generation": meta["generation"], "stamp_ns": stamp,
                  "id": msg.id, "binary": msg.binary, "resolution": msg.resolution,
                  "data": list(msg.data)})
            meta["loaded"] = True
            counts["maps"] += 1
        for check in checks[:]:
            meta = metadata.get(check["stamp_ns"], {})
            if meta.get("loaded"):
                send(check)
                counts[check["kind"]] += 1
                checks.remove(check)

    try:
        while reader.has_next():
            topic, data, _ = reader.read_next()
            msg = deserialize_message(data, types[topic])
            if topic == "/rtabmap/octomap":
                stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
                if stamp in seen_maps:
                    errors.append({"error": "duplicate map stamp", "stamp_ns": stamp})
                    continue
                seen_maps.add(stamp)
                pending_maps[stamp] = msg
            else:
                value = json.loads(msg.data)
                if topic == "/rtabmap/octomap_metadata":
                    stamp = value["stamp_ns"]
                    if stamp in seen_metadata:
                        errors.append({"error": "duplicate metadata stamp", "stamp_ns": stamp})
                        continue
                    seen_metadata.add(stamp)
                    metadata[stamp] = value
                    if value.get("test_start_clear", {}).get("enabled", False):
                        test_clear_generations.add(value["generation"])
                elif topic == "/planner3d/reference_snapshot" and len(value["points"]) >= 2:
                    checks.append({"kind": "references", "generation": value["map_generation"],
                                   "stamp_ns": value["map_stamp_ns"], "points": value["points"],
                                   "radius": value["lethal_radius"], "executable": value["executable"]})
                elif topic == "/planner3d/path_snapshot" and len(value["points"]) >= 2:
                    if not value.get("executable", True):
                        errors.append({"error": "reference route published as executable path"})
                    checks.append({"kind": "paths", "generation": value["map_generation"],
                                   "stamp_ns": value["map_stamp_ns"], "points": value["points"],
                                   "radius": value["lethal_radius"]})
                elif topic == "/planner3d/follower/status":
                    states[value["state"]] += 1
                    if value["state"] == "GOAL_REACHED" and not value.get("goal_exact", True):
                        errors.append({"error": "partial path reported requested goal reached"})
                    if value["state"] in ("FOLLOWING", "GOAL_REACHED", "SAFE_APPROACH_REACHED",
                                          "EXPLORATION_ENDPOINT_REACHED"):
                        if value["path_map_generation"] != value["map_generation"]:
                            errors.append({"error": "follower path generation mismatch"})
                        if value.get("pose_map_generation", value["map_generation"]) != value["map_generation"]:
                            errors.append({"error": "follower pose generation mismatch"})
                        for field in ("carrot", "lookahead"):
                            checks.append({"kind": "chords", "generation": value["map_generation"],
                                           "stamp_ns": value["map_stamp_ns"],
                                           "points": [value["pose"], value[field]],
                                           "radius": value["lethal_radius"]})
            flush()
        if checks:
            errors.append({"error": "missing recorded maps", "checks": len(checks)})
    finally:
        process.stdin.close()
        process.wait(timeout=10)
        process.stdout.close()
    return {"valid": not errors and counts["paths"] > 0 and counts["chords"] > 0,
            "accepted_path_check": "occupied_and_unknown",
            "reference_check": "occupied_only; not executable",
            "uses_artificial_start_clear": bool(test_clear_generations),
            "artificial_start_generations": sorted(test_clear_generations),
            "physical_clearance_valid": not test_clear_generations and not errors
                and counts["paths"] > 0 and counts["chords"] > 0,
            "map_cache_generations": 16, "counts": dict(counts), "follower_states": dict(states), "errors": errors}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    if args.out.exists():
        parser.error("output already exists")
    result = audit(args.bag)
    args.out.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))
    raise SystemExit(0 if result["valid"] else 1)


if __name__ == "__main__":
    main()
