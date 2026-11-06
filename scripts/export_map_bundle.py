#!/usr/bin/env python3
"""Bundle a map package for an external agent: what a robot would have of the map, never the simulator's truth.

    bash scripts/g1_docker.sh python3 scripts/export_map_bundle.py maps/campus

Writes outputs/handoff/<name>_<version>/ and the same as a .zip next to it:
  map.yaml, map.pgm, semantic_map.json, annotations.json, map.png, topology.png   copied from the package
  meta.json        the map's frame and SLAM facts (no simulator pose, no map scores)
  README.md        frame, places with names and aliases, doorways, anchors, how to send instructions
  manifest.json    name, version (SHA-256 of map.yaml, map.pgm, semantic_map.json), SHA-256 of every file
  SHA256SUMS       for `sha256sum -c SHA256SUMS` on the receiving side
Run scripts/build_map_package.py first; a new map or new annotations give a new version.
"""
from __future__ import annotations

import argparse
from pathlib import Path
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from g1_factory import map_package as P  # noqa: E402


def git_commit() -> str:
    try:
        # the container sees the mounted repository as someone else's: trust it for this one read
        out = subprocess.run(["git", "-c", "safe.directory=*", "-C", str(ROOT), "rev-parse", "--short", "HEAD"],
                             capture_output=True, text=True)
        return out.stdout.strip() if out.returncode == 0 else ""
    except OSError:
        return ""


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("package", help="map package folder, e.g. maps/campus")
    ap.add_argument("--out", help="output folder (default outputs/handoff/<name>_<version>)")
    args = ap.parse_args()
    package = Path(args.package)
    staging = ROOT/"outputs"/"handoff"/f".{package.name}_staging"
    shutil.rmtree(staging, ignore_errors=True)
    manifest = P.export_bundle(package, staging, git_commit())
    out = Path(args.out) if args.out else ROOT/"outputs"/"handoff"/f"{manifest['name']}_{manifest['version']}"
    if out.exists():
        shutil.rmtree(staging)
        sys.exit(f"{out} exists: this version was exported already")
    out.parent.mkdir(parents=True, exist_ok=True)
    staging.rename(out)
    archive = shutil.make_archive(str(out), "zip", out.parent, out.name)
    c = manifest["counts"]
    rel = lambda p: Path(p).resolve().relative_to(ROOT) if Path(p).resolve().is_relative_to(ROOT) else p
    print(f"{manifest['name']} version {manifest['version']}: {c['zones']} rooms, {c['places']} places, "
          f"{c['edges']} doorways, {c['anchors']} anchors")
    print(f"  {rel(out)}/ ({len(list(out.iterdir()))} files)")
    print(f"  {rel(archive)}  <- send this")


if __name__ == "__main__":
    main()
