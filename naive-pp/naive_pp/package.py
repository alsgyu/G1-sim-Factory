"""The robot's map package: SLAM occupancy grid + a person's annotations + the automatic place graph.

    maps/<name>/map.yaml, map.pgm   occupancy grid of the robot's own SLAM map (map frame = the robot's start pose)
    maps/<name>/annotations.json    rooms, corridors and anchors a person drew on that map (tools/map_annotator.html)
    maps/<name>/semantic_map.json   rooms, places, doorways and anchors (scripts/build_map_package.py); built here
                                    from annotations.json when missing

Everything is in the package's map frame, the frame the localizer puts the robot in. Simulation-only files
(evaluation.json, map_world.*, semantic_truth.json) are never read here.
"""
from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path

import numpy as np

from naive_pp import REPO  # noqa: F401  (puts g1_factory on the path)
from g1_factory import map_package as P
from g1_factory import slam_map as S

FREE, OCCUPIED, UNKNOWN = S.FREE, S.OCCUPIED, S.UNKNOWN


@dataclass
class MapPackage:
    folder: Path
    grid: np.ndarray        # int8 FREE / OCCUPIED / UNKNOWN, row 0 = lowest y
    origin: tuple           # (x, y) of cell (0, 0)'s corner
    resolution: float
    semantic: dict          # semantic_map.json
    labels: np.ndarray      # int32 index into places for every free cell, -1 elsewhere
    places: list            # semantic["places"], the order labels refer to

    @property
    def shape(self):
        return self.grid.shape

    def world_to_cell(self, xy):
        xy = np.asarray(xy, float)
        col = np.floor((xy[..., 0]-self.origin[0])/self.resolution).astype(int)
        row = np.floor((xy[..., 1]-self.origin[1])/self.resolution).astype(int)
        return row, col

    def cell_to_world(self, row, col):
        return np.stack([self.origin[0]+(np.asarray(col)+0.5)*self.resolution,
                         self.origin[1]+(np.asarray(row)+0.5)*self.resolution], axis=-1)

    def inside(self, row, col):
        h, w = self.grid.shape
        return (0 <= row) & (row < h) & (0 <= col) & (col < w)

    def place_index(self, pid: str) -> int:
        for k, p in enumerate(self.places):
            if p["id"] == pid:
                return k
        raise KeyError(pid)

    def place_at(self, xy, search_m: float = 1.0) -> str | None:
        """Place of the cell under xy, or of the nearest labelled cell within search_m (robot just off free space)."""
        r, c = (int(v) for v in self.world_to_cell(np.asarray(xy, float)))
        h, w = self.grid.shape
        n = int(round(search_m/self.resolution))
        r0, r1, c0, c1 = max(0, r-n), min(h, r+n+1), max(0, c-n), min(w, c+n+1)
        if r0 >= r1 or c0 >= c1:
            return None
        window = self.labels[r0:r1, c0:c1]
        rows, cols = np.nonzero(window >= 0)
        if len(rows) == 0:
            return None
        k = int(np.argmin((rows+r0-r)**2+(cols+c0-c)**2))
        return self.places[int(window[rows[k], cols[k]])]["id"]


def zones_of(places: list) -> list:
    """Annotated zones (annotation format) of a semantic map's places, in their original order."""
    return [{"id": p["id"], "label": p["label"], "aliases": p.get("aliases", []),
             "kind": "room" if p["kind"] == "zone" else "corridor", "rects": p["rects"]}
            for p in places if p["kind"] in ("zone", "aisle") and p.get("rects")]


def load_package(folder) -> MapPackage:
    folder = Path(folder)
    grid, origin, res = S.read_map_yaml(folder/"map.yaml")
    path = folder/"semantic_map.json"
    if path.is_file():
        semantic = json.loads(path.read_text(encoding="utf-8"))
    elif (folder/"annotations.json").is_file():
        semantic = P.build_semantic(grid, origin, res, P.load_annotations(folder/"annotations.json"))
    else:
        raise FileNotFoundError(f"{folder}: needs semantic_map.json or annotations.json (a person's annotations)")
    # The per-cell place labels are not stored in semantic_map.json: label the free space again exactly as
    # build_semantic did (same zones, same order), so every free cell knows its room or corridor.
    labels, places = P.label_free_space(grid, origin, res, zones_of(semantic["places"]))
    if [p["id"] for p in places] != [p["id"] for p in semantic["places"]]:
        raise ValueError(f"{folder}: semantic_map.json does not match map.yaml; rebuild it "
                         "(python3 scripts/build_map_package.py <folder>)")
    return MapPackage(folder, grid, tuple(float(v) for v in origin), float(res), semantic, labels,
                      semantic["places"])
