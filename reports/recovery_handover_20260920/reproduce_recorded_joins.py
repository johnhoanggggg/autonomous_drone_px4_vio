#!/usr/bin/env python3
"""Build exact-source-stamp, ROS-free PathCommandLimiter join cases."""
import json, math, subprocess
from pathlib import Path

R = Path(__file__).parent
D = json.loads((R.parent / "tunnel_flights_20260920_1200" / "120016.json").read_text())

def stamp(m):
    h = m["header"]["stamp"]
    return h["sec"] * 1_000_000_000 + h["nanosec"]

def yaw(m):
    q = m["pose"]["orientation"]
    return math.atan2(2 * (q["w"] * q["z"] + q["x"] * q["y"]),
                      1 - 2 * (q["y"] * q["y"] + q["z"] * q["z"]))

def pair_maps():
    body = {stamp(m): m for _, m in D["/rtabmap/body_pose"]}
    correction = {stamp(m): m for _, m in D["/rtabmap/odom_correction"]}
    return body, correction

BODY, CORR = pair_maps()

def preceding(topic, t):
    values = [x for x in D[topic] if x[0] <= t]
    if not values:
        raise RuntimeError(f"no preceding {topic} sample at {t}")
    return values[-1]

def exact_pair_before(t):
    values = [(ot, BODY[stamp(m)], CORR[stamp(m)])
              for ot, m in D["/rtabmap/body_pose"]
              if stamp(m) in CORR and ot <= t]
    if not values:
        raise RuntimeError(f"no exact body/correction pair before {t}")
    return values[-1]

def rotate_minus(v, angle):
    c, s = math.cos(angle), math.sin(angle)
    return (c * v[0] + s * v[1], -s * v[0] + c * v[1])

def map_to_ned(point_map, map_pose, correction, local):
    delta = (point_map[0] - map_pose[0], point_map[1] - map_pose[1])
    vio = rotate_minus(delta, yaw(correction))
    return (local[0] + vio[1], local[1] + vio[0])

def map_vector_to_ned(vector_map, correction):
    vio = rotate_minus(vector_map, yaw(correction))
    return (vio[1], vio[0])

def reexpress(point_map, from_correction, to_correction):
    # p_new = R(yaw_to-yaw_from) * (p_old-t_from) + t_to
    f = from_correction["pose"]["position"]
    t = to_correction["pose"]["position"]
    delta = (point_map[0] - f["x"], point_map[1] - f["y"])
    angle = yaw(to_correction) - yaw(from_correction)
    c, s = math.cos(angle), math.sin(angle)
    return (c * delta[0] - s * delta[1] + t["x"],
            s * delta[0] + c * delta[1] + t["y"])

def path_input_records():
    # Paths 1--6 predate the first exact body/correction pair and are never
    # accepted. Accepted path_input_id is path record index minus six.
    records = []
    for index, (receipt, msg) in enumerate(D["/planner/path"], 1):
        source = stamp(msg)
        if source not in BODY or source not in CORR:
            continue
        records.append((index - 6, receipt, msg, BODY[source], CORR[source]))
    return {item[0]: item for item in records}

def case(install_t, accepted_id, seed_limit, seed_lead_override=None):
    path_id, path_receipt, path_msg, path_body, path_corr = path_input_records()[accepted_id]
    # The adapter's current coherent map pose/correction and PX4 reference are
    # the latest preceding samples at the handover callback.
    _, current_body, current_corr = exact_pair_before(install_t)
    _, local_msg = preceding("/fmu/out/vehicle_local_position_v1", install_t)
    local = (local_msg["x"], local_msg["y"])
    map_pose = (current_body["pose"]["position"]["x"], current_body["pose"]["position"]["y"])
    points_map = [(p["pose"]["position"]["x"], p["pose"]["position"]["y"])
                  for p in path_msg["poses"]]
    points_map = [reexpress(p, path_corr, current_corr) for p in points_map]
    points_ned = [map_to_ned(p, map_pose, current_corr, local) for p in points_map]
    _, sp = preceding("/fmu/in/trajectory_setpoint", install_t)
    seed = sp["position"][:2]
    velocity = sp["velocity"][:2]
    if seed_lead_override is not None:
        delta = (seed[0] - local[0], seed[1] - local[1])
        norm = math.hypot(*delta)
        seed = [local[0] + delta[0] * seed_lead_override / norm,
                local[1] + delta[1] * seed_lead_override / norm]
    _, displacement = preceding("/planner/follower/vio_displacement", install_t)
    vector = displacement["vector"]
    # update_route_command() consumes vio_displacement_ directly through
    # vio_enu_displacement_to_ned(): ENU (x,y) -> NED (y,x). The correction
    # belongs to map/path geometry; applying it here would rotate this input a
    # second time.
    desired_delta = (vector["y"], vector["x"])
    desired = [local[0] + desired_delta[0], local[1] + desired_delta[1]]
    body = [local[0], local[1]]
    return {
        "id": accepted_id,
        "install_time": install_t,
        "path_receipt": path_receipt,
        "path_source_stamp": stamp(path_msg),
        "body_pair_receipt": preceding("/rtabmap/body_pose", install_t)[0],
        "seed_sample_receipt": preceding("/fmu/in/trajectory_setpoint", install_t)[0],
        "body": body,
        "seed": seed,
        "velocity": velocity,
        "path": points_ned,
        "desired": desired,
        "desired_offset": [desired[0] - body[0], desired[1] - body[1]],
        "follower_displacement_vio": [vector["x"], vector["y"]],
        "seed_limit": seed_limit,
        "dt": 0.05,
        "speed_cap": 0.06,
    }

def main():
    install = [(7.1143, 8), (7.6505, 10), (8.2134, 11),
               (8.8038, 12), (9.3569, 13), (9.9568, 14)]
    cases = [case(t, i, 0.05) for t, i in install]
    payload = {"cases": cases}
    binary = R / "reproduce_json_join"
    output = subprocess.check_output([str(binary)], input=json.dumps(payload).encode())
    result = json.loads(output)
    report = {"input": payload, "output": result}
    (R / "reproduce_recorded_joins.json").write_text(json.dumps(report, indent=2) + "\n")
    for row in result:
        print(row["id"], "seed", f'{row["body_lead"]:.6f}',
              "post", f'{row.get("post_lead", float("nan")):.6f}',
              "deltaV", f'{row.get("velocity_delta", float("nan")):.6f}',
              "within", row.get("post_within_seed_limit"))

    # Exact same recorded cases, with the active handover admission lead
    # reduced to 47 mm. This is the requested one-tick-headroom control.
    control = {"cases": [case(t, i, 0.05, 0.047) for t, i in install]}
    control_out = json.loads(subprocess.check_output(
        [str(binary)], input=json.dumps(control).encode()))
    report["control_47mm"] = control_out
    (R / "reproduce_recorded_joins.json").write_text(json.dumps(report, indent=2) + "\n")
    print("control_47mm")
    for row in control_out:
        print(row["id"], "seed", f'{row["body_lead"]:.6f}',
              "post", f'{row.get("post_lead", float("nan")):.6f}',
              "deltaV", f'{row.get("velocity_delta", float("nan")):.6f}',
              "within", row.get("post_within_seed_limit"))

if __name__ == "__main__":
    main()
