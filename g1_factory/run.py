"""Run one G1 episode; simulation time is authoritative for all metrics."""
from __future__ import annotations

import argparse
from contextlib import nullcontext
import csv
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import time

import mujoco
import numpy as np

from .locomotion import G1Locomotion, projected_gravity
from .navigation import WaypointFollower, RelativeActionFollower, cross_track_error, yaw_from_quat
from .scene import build_scene, temperature_readings

ROOT = Path(__file__).resolve().parents[1]


def environment_contacts(model, data):
    """Only robot vs factory furniture/wall contacts; ignore feet vs floor."""
    names = []
    for contact in data.contact:
        if contact.dist > 0:
            continue
        a, b = int(contact.geom1), int(contact.geom2)
        na = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, a) or ""
        nb = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, b) or ""
        fa, fb = na.startswith("factory_"), nb.startswith("factory_")
        if fa != fb:
            env = na if fa else nb
            if env != "factory_floor":
                names.append(env)
    return sorted(set(names))


def parser():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--config", type=Path, default=ROOT/"configs/factory.yaml")
    p.add_argument("--upstream", type=Path, default=ROOT/"external/unitree_rl_gym")
    p.add_argument("--lab", default="lab_a")
    p.add_argument("--headless", action="store_true", help="No window; no GPU/GL required unless recording or VLN")
    p.add_argument("--duration", type=float, default=120)
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--output", type=Path)
    p.add_argument("--record", action="store_true", help="Save ego RGB PNG + metric depth NPY every 0.5 simulation seconds")
    p.add_argument("--navila-url", help="Optional NaVILA /infer server; replaces fixed waypoints as action source")
    p.add_argument("--instruction", help="Required for NaVILA; preferably English per the upstream examples")
    p.add_argument("--goal-local", nargs=2, type=float, metavar=("X", "Y"), help="Required NaVILA evaluation goal, metres relative to this bay; never provided to the model")
    return p


def main(argv=None):
    args = parser().parse_args(argv)
    if args.duration <= 0 or not np.isfinite(args.duration):
        raise SystemExit("--duration must be finite and positive")
    if args.navila_url and not args.instruction:
        raise SystemExit("--instruction is required with --navila-url")
    if args.navila_url and args.goal_local is None:
        raise SystemExit("--goal-local X Y is required for VLN evaluation; loop endpoint would equal spawn")
    np.random.seed(args.seed)
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
    out = args.output or ROOT / "outputs" / f"{args.lab}_{stamp}"
    out.mkdir(parents=True, exist_ok=False)
    meta = build_scene(args.config, args.upstream, out/"scene.xml")
    if args.lab not in meta["bays"]:
        raise SystemExit(f"Unknown lab: {args.lab}; choose {list(meta['bays'])}")
    bay = meta["bays"][args.lab]
    evaluation_goal = np.asarray(bay["waypoints"][-1])
    if args.navila_url:
        goal = np.asarray(args.goal_local)
        inner = np.asarray(meta["config"]["layout"]["bay_size"])/2 - 0.6
        if not np.isfinite(goal).all() or (np.abs(goal) > inner).any():
            raise SystemExit("VLN goal must be a finite in-bay coordinate with boundary clearance")
        evaluation_goal = goal + np.asarray(bay["origin"])
        if np.linalg.norm(evaluation_goal-np.asarray(bay["spawn"][:2])) <= 0.5:
            raise SystemExit("VLN goal must be farther than 0.5 m from spawn")
    model = mujoco.MjModel.from_xml_path(str(meta["scene_path"]))
    data = mujoco.MjData(model)
    walk = G1Locomotion(model, data, args.upstream)
    walk.reset(bay["spawn"][:2], bay["spawn"][2])
    follower = WaypointFollower(bay["waypoints"])
    reference = np.vstack([bay["spawn"][:2], bay["waypoints"]])
    renderer, client, current_action = None, None, None
    frames, action_log, readings, trace = [], [], [], []
    if args.record or args.navila_url:
        renderer = mujoco.Renderer(model, height=240, width=320)
        (out/"frames").mkdir()
    if args.navila_url:
        from .navila import NaVILAClient
        client = NaVILAClient(args.navila_url)
    viewer_context = nullcontext(None)
    if not args.headless:
        from mujoco import viewer as mj_viewer
        viewer_context = mj_viewer.launch_passive(model, data)
    status, failure, collision_steps = "timeout", None, 0
    minimum_height, travelled = float("inf"), 0.0
    last_xy = data.qpos[:2].copy()
    command = np.zeros(3, dtype=np.float32)
    previous_collision = False
    collision_events, collision_names = 0, set()
    tick = 0
    nav_action_start = 0.0
    start_wall = time.monotonic()
    try:
        with viewer_context as viewer:
            if viewer:
                viewer.cam.lookat[:] = [*bay["origin"], 0.7]
                viewer.cam.distance = 12
                viewer.cam.elevation = -50
            while data.time < args.duration:
                if viewer and not viewer.is_running():
                    status = "viewer_closed"
                    break
                tick_wall = time.monotonic()
                xy, yaw = data.qpos[:2].copy(), yaw_from_quat(data.qpos[3:7])
                if tick % walk.decimation == 0:
                    completed_status = None
                    if renderer and tick % 250 == 0:
                        renderer.update_scene(data, camera=meta["camera_names"]["ego"])
                        rgb = renderer.render().copy()
                        frames.append(rgb)
                        frames = frames[-8:]
                        if args.record:
                            from PIL import Image
                            stem = f"{data.time:09.3f}"
                            Image.fromarray(rgb).save(out/"frames"/f"{stem}.png")
                            renderer.enable_depth_rendering()
                            depth = renderer.render().copy()
                            renderer.disable_depth_rendering()
                            np.save(out/"frames"/f"{stem}_depth_m.npy", depth)
                    if data.time < 1.0:
                        command[:] = 0  # allow the released policy to settle
                    elif client:
                        if current_action is None or current_action.done:
                            # Synchronous inference pauses physics: latency is logged separately.
                            renderer.update_scene(data, camera=meta["camera_names"]["ego"])
                            frames.append(renderer.render().copy())
                            frames = frames[-8:]
                            begin = time.monotonic()
                            action = client.infer(args.instruction, frames)
                            action_log.append({"sim_time_s": data.time, "kind": action.kind, "text": client.last_text,
                                               "value": action.value, "latency_s": time.monotonic()-begin})
                            if action.kind == "stop":
                                completed_status = "model_stop"
                                command[:] = 0
                            else:
                                current_action = RelativeActionFollower(action, xy, yaw)
                            nav_action_start = data.time
                        if not completed_status:
                            command = current_action.command(xy, yaw)
                        if data.time-nav_action_start > 20:
                            status = "action_timeout"
                            break
                    else:
                        command = follower.command(xy, yaw)
                        if follower.done:
                            completed_status = "path_complete"
                    contacts = environment_contacts(model, data)
                    hit = bool(contacts)
                    collision_steps += int(hit)
                    collision_events += int(hit and not previous_collision)
                    previous_collision = hit
                    collision_names.update(contacts)
                    height = float(data.qpos[2])
                    minimum_height = min(minimum_height, height)
                    travelled += float(np.linalg.norm(xy-last_xy))
                    last_xy = xy
                    error = cross_track_error(xy, reference)
                    trace.append([data.time, *xy, height, yaw, *command, error, int(hit), follower.index])
                    if tick % 500 == 0:
                        readings.append({"sim_time_s": float(data.time), "sensors": temperature_readings(meta, data.time)})
                    if data.time > 0.5 and (height < 0.45 or projected_gravity(data.qpos[3:7])[2] > -0.5):
                        status = "fallen"
                        break
                    if hit:
                        status = "collision"
                        break
                    if completed_status:
                        status = completed_status
                        break
                walk.step(command)
                tick += 1
                # Catch transient contacts/falls at the physics rate, including
                # events between the 50 Hz trajectory samples.
                step_contacts = environment_contacts(model, data)
                step_fallen = data.time > 0.5 and (data.qpos[2] < 0.45 or projected_gravity(data.qpos[3:7])[2] > -0.5)
                if step_contacts or step_fallen:
                    collision_steps += int(bool(step_contacts))
                    collision_events += int(bool(step_contacts) and not previous_collision)
                    collision_names.update(step_contacts)
                    status = "fallen" if step_fallen else "collision"
                    xy = data.qpos[:2].copy()
                    height = float(data.qpos[2])
                    minimum_height = min(minimum_height, height)
                    travelled += float(np.linalg.norm(xy-last_xy))
                    trace.append([data.time, *xy, height, yaw_from_quat(data.qpos[3:7]), *command,
                                  cross_track_error(xy, reference), int(bool(step_contacts)), follower.index])
                    break
                if viewer and tick % 10 == 0:
                    viewer.sync()
                    pause = .02 - (time.monotonic()-tick_wall)
                    if pause > 0:
                        time.sleep(pause)
    except Exception as exc:
        status, failure = "error", f"{type(exc).__name__}: {exc}"
    finally:
        if renderer:
            renderer.close()
    goal_error = float(np.linalg.norm(data.qpos[:2]-evaluation_goal))
    errors = [row[8] for row in trace]
    # VLN stop is evaluated against a configured oracle goal, not against its text.
    reached = (status == "path_complete") if not client else (status == "model_stop" and goal_error <= 0.5)
    success = reached and collision_steps == 0 and status not in ("fallen", "error")
    try:
        import subprocess
        if not (args.upstream/".git").exists():
            raise FileNotFoundError("Assets were downloaded without a Git checkout")
        upstream_commit = subprocess.check_output(["git", "-C", str(args.upstream), "rev-parse", "HEAD"], text=True).strip()
    except Exception:
        manifest_path = args.upstream.parent/"assets_manifest.json"
        upstream_commit = json.loads(manifest_path.read_text()).get("commit", "unknown") if manifest_path.exists() else "unknown"
    summary = {"success": success, "status": status, "error": failure, "lab": args.lab,
               "mode": "navila" if client else "fixed_path", "sim_time_s": float(data.time),
               "wall_time_s": time.monotonic()-start_wall, "goal_error_m": goal_error,
               "cross_track_rmse_m": float(np.sqrt(np.mean(np.square(errors)))) if errors and not client else None,
               "cross_track_max_m": max(errors) if errors and not client else None,
               "travelled_m": travelled, "waypoints_reached": follower.index if not client else None,
               "waypoints_total": len(follower.points), "collision_events": collision_events,
               "collision_samples": collision_steps, "safety_check_hz": 500, "collision_geoms": sorted(collision_names),
               "min_base_height_m": minimum_height if np.isfinite(minimum_height) else None,
               "seed": args.seed, "mujoco": mujoco.__version__, "upstream_commit": upstream_commit,
               "evaluation_goal_world_xy": evaluation_goal.tolist(),
               "config_sha256": hashlib.sha256(args.config.read_bytes()).hexdigest(),
               "policy_sha256": hashlib.sha256(walk.policy_path.read_bytes()).hexdigest(),
               "pose_source": "simulator ground truth", "temperature_source": "synthetic spatial field",
               "inference_clock": "physics paused during synchronous VLN inference" if client else None}
    (out/"summary.json").write_text(json.dumps(summary, indent=2)+"\n")
    (out/"temperature.json").write_text(json.dumps(readings, indent=2)+"\n")
    (out/"actions.json").write_text(json.dumps(action_log, indent=2)+"\n")
    (out/"config.yaml").write_bytes(args.config.read_bytes())
    with (out/"trajectory.csv").open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["sim_time_s", "x_m", "y_m", "base_z_m", "yaw_rad", "cmd_vx", "cmd_vy", "cmd_wz", "cross_track_m", "collision", "waypoint_index"])
        writer.writerows(trace)
    print(json.dumps(summary, indent=2))
    print(f"Results: {out}")
    return 0 if success else 1


if __name__ == "__main__":
    raise SystemExit(main())
