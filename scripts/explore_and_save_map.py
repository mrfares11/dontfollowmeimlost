#!/usr/bin/env python3
"""
explore_and_save_map.py

Runs explore_lite, watches its console output, and when exploration finishes it
saves the current SLAM map into a maps/ folder inside the project directory.

Default output:
    <project_dir>/maps/saved_map.yaml
    <project_dir>/maps/saved_map.pgm

If saved_map.yaml or saved_map.pgm already exists, this script automatically
uses the next available numbered name:

    saved_map.yaml
    saved_map_1.yaml
    saved_map_2.yaml
    saved_map_3.yaml
    ...

This prevents old maps from being overwritten.
"""

import argparse
import os
import signal
import subprocess
import sys
import time
from pathlib import Path


FINISH_PATTERNS = (
    "All frontiers traversed/tried out, stopping.",
    "Exploration stopped.",
)


def terminate_process_tree(proc: subprocess.Popen, timeout_s: float = 3.0) -> None:
    """Terminate the explore_lite process group cleanly."""
    if proc.poll() is not None:
        return

    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        proc.wait(timeout=timeout_s)
    except Exception:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except Exception:
            pass


def map_files_exist(map_base_path: Path) -> bool:
    """
    Return True if either output file already exists.

    map_base_path has no extension. For example:
        /home/hadi/amr_project/maps/saved_map

    This checks:
        /home/hadi/amr_project/maps/saved_map.yaml
        /home/hadi/amr_project/maps/saved_map.pgm
    """
    return (
        map_base_path.with_suffix(".yaml").exists()
        or map_base_path.with_suffix(".pgm").exists()
    )


def next_available_map_base_path(maps_dir: Path, map_name: str) -> Path:
    """
    Find the next free map base path without overwriting existing maps.

    If map_name is 'saved_map', the sequence is:
        saved_map
        saved_map_1
        saved_map_2
        saved_map_3
        ...
    """
    candidate = maps_dir / map_name

    if not map_files_exist(candidate):
        return candidate

    idx = 1
    while True:
        candidate = maps_dir / f"{map_name}_{idx}"
        if not map_files_exist(candidate):
            return candidate
        idx += 1


def save_map(map_base_path: Path, map_topic: str, timeout_s: float) -> int:
    """Call Nav2 map_saver_cli to save /map as .yaml + .pgm."""
    map_base_path.parent.mkdir(parents=True, exist_ok=True)

    cmd = [
        "ros2", "run", "nav2_map_server", "map_saver_cli",
        "-f", str(map_base_path),
        "-t", map_topic,
        "--fmt", "pgm",
        "--mode", "trinary",
        "--ros-args",
        "-p", "map_subscribe_transient_local:=true",
        "-p", f"save_map_timeout:={timeout_s}",
    ]

    print("", flush=True)
    print("[explore_and_save_map] Saving map with command:", flush=True)
    print("[explore_and_save_map] " + " ".join(cmd), flush=True)

    result = subprocess.run(cmd)

    if result.returncode == 0:
        print("", flush=True)
        print("[explore_and_save_map] Map saved successfully:", flush=True)
        print(f"[explore_and_save_map]   {map_base_path}.yaml", flush=True)
        print(f"[explore_and_save_map]   {map_base_path}.pgm", flush=True)
    else:
        print("", flush=True)
        print(
            f"[explore_and_save_map] ERROR: map_saver_cli failed with return code {result.returncode}",
            flush=True,
        )

    return result.returncode


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--explore-params",
        default="/home/hadi/amr_project/param/explore_params.yaml",
        help="Path to explore_lite params YAML. Relative paths are resolved from the current working directory.",
    )
    parser.add_argument(
        "--maps-dir",
        default="maps",
        help="Directory where the map is saved. Relative paths are resolved from the current working directory.",
    )
    parser.add_argument(
        "--map-name",
        default="saved_map",
        help="Base name for saved map files, without .yaml/.pgm extension.",
    )
    parser.add_argument(
        "--map-topic",
        default="/map",
        help="OccupancyGrid topic to save.",
    )
    parser.add_argument(
        "--save-delay",
        type=float,
        default=2.0,
        help="Seconds to wait after exploration finishes before saving the map.",
    )
    parser.add_argument(
        "--save-timeout",
        type=float,
        default=10.0,
        help="Timeout passed to nav2_map_server map_saver_cli.",
    )
    parser.add_argument(
        "--use-sim-time",
        default="true",
        choices=("true", "false"),
        help="use_sim_time parameter passed to explore_lite.",
    )
    parser.add_argument(
        "--keep-explore-alive",
        action="store_true",
        help="Keep explore_lite running after the map is saved. By default it is stopped after saving.",
    )
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="Overwrite the requested map name instead of auto-numbering. Default is false.",
    )
    args = parser.parse_args()

    work_dir = Path.cwd()

    explore_params = Path(args.explore_params)
    if not explore_params.is_absolute():
        explore_params = work_dir / explore_params

    maps_dir = Path(args.maps_dir)
    if not maps_dir.is_absolute():
        maps_dir = work_dir / maps_dir

    maps_dir.mkdir(parents=True, exist_ok=True)

    requested_map_base_path = maps_dir / args.map_name

    if args.overwrite:
        map_base_path = requested_map_base_path
    else:
        map_base_path = next_available_map_base_path(maps_dir, args.map_name)

    if not explore_params.exists():
        print(
            f"[explore_and_save_map] ERROR: explore params file does not exist: {explore_params}",
            flush=True,
        )
        return 2

    explore_cmd = [
        "stdbuf", "-oL", "-eL",
        "ros2", "run", "explore_lite", "explore",
        "--ros-args",
        "--params-file", str(explore_params),
        "-p", f"use_sim_time:={args.use_sim_time}",
    ]

    print(f"[explore_and_save_map] Working directory: {work_dir}", flush=True)
    print(f"[explore_and_save_map] Maps directory:    {maps_dir}", flush=True)
    print(f"[explore_and_save_map] Requested base:     {requested_map_base_path}", flush=True)

    if map_base_path != requested_map_base_path:
        print(
            "[explore_and_save_map] Requested map already exists. Using next available name:",
            flush=True,
        )

    print(f"[explore_and_save_map] Map output base:   {map_base_path}", flush=True)
    print("[explore_and_save_map] Starting explore_lite...", flush=True)
    print("[explore_and_save_map] " + " ".join(explore_cmd), flush=True)

    saved = False
    proc = subprocess.Popen(
        explore_cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
        preexec_fn=os.setsid,
    )

    try:
        assert proc.stdout is not None
        for line in proc.stdout:
            print(line, end="", flush=True)

            if saved:
                continue

            if any(pattern in line for pattern in FINISH_PATTERNS):
                saved = True
                print("", flush=True)
                print("[explore_and_save_map] Exploration finish message detected.", flush=True)
                print(
                    f"[explore_and_save_map] Waiting {args.save_delay:.1f}s before saving map...",
                    flush=True,
                )
                time.sleep(args.save_delay)

                # Re-check the available name at save time too, in case another
                # process created a map while exploration was running.
                if not args.overwrite:
                    map_base_path = next_available_map_base_path(maps_dir, args.map_name)
                    print(
                        f"[explore_and_save_map] Final map output base: {map_base_path}",
                        flush=True,
                    )

                ret = save_map(map_base_path, args.map_topic, args.save_timeout)

                if not args.keep_explore_alive:
                    print("[explore_and_save_map] Stopping explore_lite after map save.", flush=True)
                    terminate_process_tree(proc)

                return ret

        return proc.wait()

    except KeyboardInterrupt:
        print("\n[explore_and_save_map] Ctrl+C received, stopping explore_lite.", flush=True)
        terminate_process_tree(proc)
        return 130

    finally:
        terminate_process_tree(proc)


if __name__ == "__main__":
    sys.exit(main())
