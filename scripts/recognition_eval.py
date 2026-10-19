#!/usr/bin/env python3
"""Score zone and object recognition on captured frames (Isaac Sim: isaac_sim_node --capture).

    bash scripts/g1_docker.sh python3 scripts/recognition_eval.py outputs/recognition/run1 \
        --vlm-model qwen3-vl:8b-instruct

For every frame the VLM answers two questions with only the head-camera image:
  1. zone   - which area of the campus the robot is in (9 zones or "aisle/corridor"),
  2. objects - every visible object of the known classes, with its name plate and box.
Ground truth comes from the semantic map: the zone from the robot pose, the visible objects by projecting
each object's 3-D box into the camera and checking it against depth (g1_factory.recognition).
Writes <capture>/eval_<model>/results.json, results.md and annotated example frames.
"""
from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import json
from pathlib import Path
import sys

import numpy as np
from PIL import Image, ImageDraw

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from g1_factory.recognition import AISLE, INTRINSICS, match_detections, visible_objects  # noqa: E402
from g1_factory.vlm import VLMClient, extract_json  # noqa: E402

ZONE_SYSTEM = """You are the perception module of a humanoid robot in a factory campus (a warehouse, an office \
building and a pump room). Using ONLY the head-camera image, decide which area the robot is standing in. \
Signs, name plates, equipment and furniture are the evidence. Reply with one JSON object and nothing else: \
{"evidence": "<what you see that tells you>", "area": "<one area id from the list>"}"""

OBJECT_SYSTEM = """You are the perception module of a humanoid robot in a factory campus. List every object of \
the given classes that is clearly visible in the head-camera image (ignore objects you cannot identify). Signs, \
name plates and floor markings are not objects; list each physical object once. For \
each give its class (exactly one name from the list), the text of its name plate if you can read it (else ""), \
and its bounding box [x1, y1, x2, y2] normalized to 0-1000 (x right, y down). Reply with one JSON object and \
nothing else: {"objects": [{"class": "...", "plate": "...", "bbox_2d": [x1, y1, x2, y2]}]}"""


def ask_zone(client, image, zones):
    options = "\n".join(f"- {z['id']}: {z['label']}" for z in zones)
    text = f"Areas:\n{options}\n- {AISLE}: an aisle, corridor or passage between areas\nWhich area is the robot in?"
    call = client.chat(ZONE_SYSTEM, text, [image], max_tokens=200)
    try:
        area = str(extract_json(call.text).get("area", "")).strip()
    except ValueError:
        area = ""
    return area, call


def ask_objects(client, image, classes, size):
    text = "Object classes:\n" + "\n".join(f"- {c}" for c in classes) + "\nList the visible objects."
    call = client.chat(OBJECT_SYSTEM, text, [image], max_tokens=700)
    w, h = size
    out = []
    try:
        items = extract_json(call.text).get("objects", [])
    except ValueError:
        items = []
    for item in items if isinstance(items, list) else []:
        try:
            x1, y1, x2, y2 = [float(v) for v in item.get("bbox_2d")]
        except (TypeError, ValueError):
            continue
        cls = str(item.get("class", "")).strip()
        if cls not in classes or x2 <= x1 or y2 <= y1:
            continue
        out.append({"class": cls, "plate": str(item.get("plate", "")),
                    "bbox_px": [x1*w/1000, y1*h/1000, x2*w/1000, y2*h/1000]})
    return out, call


def draw_example(image, truth, predictions, zone_truth, zone_pred, path):
    img = image.copy()
    d = ImageDraw.Draw(img)
    for t in truth:
        if t["status"] == "visible":
            d.rectangle(t["bbox_px"], outline=(0, 200, 80), width=3)
            d.text((t["bbox_px"][0]+3, t["bbox_px"][1]+3), t["label"] or t["class"], fill=(0, 200, 80))
    for p in predictions:
        d.rectangle(p["bbox_px"], outline=(255, 120, 0), width=2)
        d.text((p["bbox_px"][0]+3, p["bbox_px"][3]-12), p["class"] + (f" [{p['plate']}]" if p["plate"] else ""),
               fill=(255, 120, 0))
    d.rectangle([0, 0, img.width, 22], fill=(0, 0, 0))
    d.text((6, 5), f"zone truth: {zone_truth}   VLM: {zone_pred}   (green = ground truth, orange = VLM)",
           fill=(255, 255, 255))
    img.save(path, quality=88)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("capture", type=Path)
    parser.add_argument("--vlm-url", default="http://127.0.0.1:11434/v1")
    parser.add_argument("--vlm-model", default="qwen3-vl:8b-instruct")
    parser.add_argument("--limit", type=int, default=0, help="score only the first N frames")
    parser.add_argument("--examples", type=int, default=12, help="annotated example frames to save")
    args = parser.parse_args()
    index = json.loads((args.capture/"index.json").read_text())
    semantic = json.loads((args.capture/"semantic_map.json").read_text())
    objects = {o["id"]: o for o in semantic["objects"]}
    classes = sorted({o["class"] for o in semantic["objects"]})
    zone_ids = {z["id"] for z in semantic["zones"]} | {AISLE}
    client = VLMClient(args.vlm_url, args.vlm_model, timeout=300)
    out = args.capture/f"eval_{args.vlm_model.replace(':', '_').replace('/', '_')}"
    (out/"examples").mkdir(parents=True, exist_ok=True)

    captures = index["captures"][:args.limit or None]
    confusion = defaultdict(Counter)
    per_class = defaultdict(lambda: Counter())
    loc_errors, plates, latencies, rows = [], [], [], []
    for k, cap in enumerate(captures):
        jpg = args.capture/"frames"/f"{cap['frame']}.jpg"
        image = (Image.open(jpg).convert("RGB") if jpg.exists() else
                 Image.fromarray(np.load(args.capture/"frames"/f"{cap['frame']}_rgb.npy")))
        depth = np.load(args.capture/"frames"/f"{cap['frame']}_depth.npy").astype(np.float32)
        cam = np.array(cap["map_from_optical"])
        truth = visible_objects(semantic["objects"], depth, cam, INTRINSICS)
        area, zcall = ask_zone(client, np.asarray(image), semantic["zones"])
        preds, ocall = ask_objects(client, np.asarray(image), classes, image.size)
        latencies += [zcall.latency_s, ocall.latency_s]
        m = match_detections(preds, truth, objects, depth, cam)
        zone_ok = area == cap["zone"]
        confusion[cap["zone"]][area if area in zone_ids else "invalid"] += 1
        for t in truth:
            if t["status"] == "visible":
                per_class[t["class"]]["gt"] += 1
        for oid in m["tp"]:
            per_class[objects[oid]["class"]]["tp"] += 1
        for p in preds:
            per_class[p["class"]]["pred"] += 1
        for oid in m["ignored"]:
            per_class[objects[oid]["class"]]["pred"] -= 1  # matched a marginal object: not a false positive
        loc_errors += m["loc_errors"]
        plates += m["plates"]
        rows.append({"frame": cap["frame"], "place": cap["place"], "zone": cap["zone"], "zone_pred": area,
                     "zone_correct": zone_ok, "visible": [t["id"] for t in truth if t["status"] == "visible"],
                     "predicted": preds, "tp": m["tp"], "fn": m["fn"], "fp": m["fp"],
                     "wrong_class": m["wrong_class"], "plates": m["plates"]})
        if k < args.examples:
            draw_example(image, truth, preds, cap["zone"], area, out/"examples"/f"{cap['frame']}.jpg")
        print(f"[{k+1}/{len(captures)}] {cap['frame']}: zone {cap['zone']} -> {area} "
              f"{'OK' if zone_ok else 'x'} | objects tp {len(m['tp'])} fn {len(m['fn'])} fp {m['fp']}", flush=True)

    n = len(rows)
    zone_acc = sum(r["zone_correct"] for r in rows)/max(n, 1)
    per_zone = {z: {"n": sum(confusion[z].values()), "correct": confusion[z][z]} for z in sorted(confusion)}
    tp = sum(c["tp"] for c in per_class.values())
    gt = sum(c["gt"] for c in per_class.values())
    pred = sum(max(0, c["pred"]) for c in per_class.values())
    precision, recall = tp/max(pred, 1), tp/max(gt, 1)
    f1 = 2*precision*recall/max(precision+recall, 1e-9)
    plate_acc = sum(p["correct"] for p in plates)/max(len(plates), 1)
    summary = {"model": args.vlm_model, "frames": n, "zone_accuracy": round(zone_acc, 3), "per_zone": per_zone,
               "confusion": {z: dict(c) for z, c in confusion.items()},
               "objects": {"precision": round(precision, 3), "recall": round(recall, 3), "f1": round(f1, 3),
                           "ground_truth_visible": gt, "predicted": pred, "true_positive": tp},
               "per_class": {c: {"gt": v["gt"], "tp": v["tp"], "pred": max(0, v["pred"]),
                                 "recall": round(v["tp"]/v["gt"], 3) if v["gt"] else None,
                                 "precision": round(v["tp"]/v["pred"], 3) if v["pred"] > 0 else None}
                             for c, v in sorted(per_class.items())},
               "plate_accuracy": round(plate_acc, 3), "plates_scored": len(plates),
               "localization_error_m": {"mean": round(float(np.mean(loc_errors)), 3) if loc_errors else None,
                                        "median": round(float(np.median(loc_errors)), 3) if loc_errors else None},
               "vlm_latency_s_mean": round(float(np.mean(latencies)), 2) if latencies else None}
    (out/"results.json").write_text(json.dumps({"summary": summary, "frames": rows}, indent=1, ensure_ascii=False))
    lines = [f"# Recognition accuracy ({args.vlm_model}, {n} frames)", "",
             "| metric | value |", "|---|---|",
             f"| zone accuracy | {zone_acc:.1%} |",
             f"| object precision / recall / F1 | {precision:.1%} / {recall:.1%} / {f1:.1%} |",
             f"| name-plate reading (matched objects with a plate) | {plate_acc:.1%} ({len(plates)}) |",
             f"| 3-D localization error (median) | {summary['localization_error_m']['median']} m |",
             f"| VLM latency per question | {summary['vlm_latency_s_mean']} s |", "",
             "| zone (truth) | frames | correct |", "|---|---:|---:|"]
    lines += [f"| {z} | {v['n']} | {v['correct']} |" for z, v in per_zone.items()]
    lines += ["", "| object class | visible | found | predicted | recall | precision |", "|---|---:|---:|---:|---:|---:|"]
    lines += [f"| {c} | {v['gt']} | {v['tp']} | {v['pred']} | {v['recall']} | {v['precision']} |"
              for c, v in summary["per_class"].items()]
    (out/"results.md").write_text("\n".join(lines)+"\n")
    print("\n".join(lines[:9]))


if __name__ == "__main__":
    main()
