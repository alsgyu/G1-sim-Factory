#!/usr/bin/env python3
"""Encode the frames recorded by the Isaac Sim node (--video NAME) into MP4 videos.

    python3 scripts/make_videos.py outputs/media/conveyor          # every task segment under it
    python3 scripts/make_videos.py outputs/media/conveyor/01_ab12cd34 --speed 2

Per segment folder (fpv/, tpv/, frames.json), writes
    fpv.mp4            first person: the robot's head camera (640x480)
    tpv.mp4            third person: the chase camera behind and above the robot
    side_by_side.mp4   both, third person on the left, at the third-person height
Frames are taken every camera period of simulated time (30 Hz), so --speed 1 plays at simulated real time.
H.264 (yuv420p) plays in browsers, QuickTime, Slack and PowerPoint. Needs ffmpeg on the PATH
(Ubuntu: sudo apt install ffmpeg; macOS: brew install ffmpeg) or the imageio-ffmpeg package; standard library only.
A segment without frames.json (the simulator stopped mid-task) is encoded from the frame files at 30 fps.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import shutil
import subprocess
import sys


def find_ffmpeg() -> str:
    exe = shutil.which("ffmpeg")
    if exe:
        return exe
    try:
        import imageio_ffmpeg
        return imageio_ffmpeg.get_ffmpeg_exe()
    except ImportError:
        sys.exit("ffmpeg not found: sudo apt install ffmpeg (Ubuntu), brew install ffmpeg (macOS) "
                 "or python3 -m pip install imageio-ffmpeg")


def segments(path: Path) -> list[Path]:
    if (path/"fpv").is_dir() and (path/"tpv").is_dir():
        return [path]
    found = sorted(p for p in path.iterdir() if p.is_dir() and (p/"fpv").is_dir() and (p/"tpv").is_dir())
    if not found:
        sys.exit(f"{path}: no segment folders with fpv/ and tpv/")
    return found


def image_size(path: Path) -> tuple[int, int] | None:
    """(width, height) from a PNG or baseline/progressive JPEG header."""
    data = path.read_bytes()[:1 << 16]
    if data[:8] == b"\x89PNG\r\n\x1a\n":
        return int.from_bytes(data[16:20], "big"), int.from_bytes(data[20:24], "big")
    k = 2
    while k+9 < len(data) and data[:2] == b"\xff\xd8":
        if data[k] != 0xFF:
            return None
        marker, length = data[k+1], int.from_bytes(data[k+2:k+4], "big")
        if marker in (0xC0, 0xC1, 0xC2):
            return int.from_bytes(data[k+7:k+9], "big"), int.from_bytes(data[k+5:k+7], "big")
        k += 2+length
    return None


def frame_info(seg: Path) -> dict:
    meta = seg/"frames.json"
    info = json.loads(meta.read_text()) if meta.is_file() else {}
    ext = info.get("ext") or next((e for e in (".jpg", ".png") if (seg/"fpv"/f"000000{e}").is_file()), ".jpg")
    count = min(sum(1 for _ in (seg/view).glob(f"*{ext}")) for view in ("fpv", "tpv"))
    first = seg/"tpv"/f"000000{ext}"
    tpv_size = info.get("tpv_size") or (image_size(first) if first.is_file() else None) or [1280, 720]
    return {"fps": float(info.get("fps") or 30.0), "ext": ext, "frames": count, "state": info.get("state"),
            "instruction": info.get("instruction", ""), "complete": meta.is_file(), "tpv_height": int(tpv_size[1])}


def commands(ffmpeg: str, seg: Path, info: dict, speed: float, crf: int) -> dict[str, list[str]]:
    """ffmpeg command per output file (exposed for tests)."""
    rate = f"{info['fps']*speed:g}"
    frames = str(info["frames"])
    pattern = {view: str(seg/view/f"%06d{info['ext']}") for view in ("fpv", "tpv")}
    encode = ["-c:v", "libx264", "-preset", "medium", "-crf", str(crf), "-pix_fmt", "yuv420p",
              "-color_range", "tv", "-colorspace", "bt709", "-color_primaries", "bt709", "-color_trc", "bt709",
              "-movflags", "+faststart"]
    # Even size for yuv420p; JPEG frames are full-range, players expect limited-range BT.709.
    even = "pad=ceil(iw/2)*2:ceil(ih/2)*2,scale=out_range=tv:out_color_matrix=bt709,format=yuv420p"
    head = [ffmpeg, "-hide_banner", "-loglevel", "error", "-y"]
    out = {}
    for view in ("fpv", "tpv"):
        out[f"{view}.mp4"] = head+["-framerate", rate, "-i", pattern[view], "-frames:v", frames, "-vf", even,
                                   *encode, str(seg/f"{view}.mp4")]
    height = info["tpv_height"]
    out["side_by_side.mp4"] = head+["-framerate", rate, "-i", pattern["tpv"], "-framerate", rate, "-i", pattern["fpv"],
                                    "-filter_complex", f"[1:v]scale=-2:{height}[f];[0:v][f]hstack=inputs=2,{even}",
                                    "-frames:v", frames, *encode, str(seg/"side_by_side.mp4")]
    return out


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("path", type=Path, help="outputs/media/<NAME> or one segment folder under it")
    p.add_argument("--speed", type=float, default=1.0, help="playback speed against simulated time (2 = twice)")
    p.add_argument("--crf", type=int, default=20, help="H.264 quality: lower is better and larger (18-28)")
    p.add_argument("--only", choices=("fpv.mp4", "tpv.mp4", "side_by_side.mp4"), action="append",
                   help="write only these files (repeatable)")
    p.add_argument("--delete-frames", action="store_true", help="remove fpv/ and tpv/ after every file encoded")
    args = p.parse_args(argv)
    ffmpeg = find_ffmpeg()
    failed = 0
    for seg in segments(args.path):
        info = frame_info(seg)
        if info["frames"] == 0:
            print(f"{seg}: no frames, skipped")
            continue
        note = "" if info["complete"] else " (no frames.json: the simulator stopped mid-task)"
        print(f"{seg.name}: {info['frames']} frames = {info['frames']/info['fps']:.1f} s simulated, "
              f"state {info['state'] or '-'}{note}")
        if info["instruction"]:
            print(f"  instruction: {info['instruction']}")
        ok = True
        for name, cmd in commands(ffmpeg, seg, info, args.speed, args.crf).items():
            if args.only and name not in args.only:
                continue
            result = subprocess.run(cmd, capture_output=True, text=True)
            if result.returncode != 0:
                ok, failed = False, failed+1
                print(f"  FAIL {name}: {result.stderr.strip()[-400:]}")
            else:
                print(f"  {seg/name} ({(seg/name).stat().st_size/1e6:.1f} MB)")
        if ok and args.delete_frames:
            for view in ("fpv", "tpv"):
                shutil.rmtree(seg/view)
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
