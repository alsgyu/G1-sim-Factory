"""Report metrics: map, localisation, zone judgement and VLN (numpy only; scripts/eval_*.py and tests use these).

Definitions (docs/METRICS.md has the full protocol):
  trajectory   ATE of the SLAM LiDAR positions against the simulator's truth, (a) aligned at the start pose (how the
               map frame is fixed in practice) and (b) after a best-fit 2-D rigid alignment (standard ATE, Umeyama
               without scale); RPE: position and heading error of 10 m path segments, % and deg/m (drift).
  doorways     the automatic place graph against the simulator's place graph. Places are matched by where their
               free cells lie (not by id); a doorway (edge) is detected when the matched pair of places is connected
               in the truth. Precision, recall, F1; position error of each detected doorway.
  first fix    a fresh localiser search from a rough guess: correct (error within tolerance), refused, or wrong.
  zones        the zone the robot would say it is in (localised pose + annotated room bounds) vs the truth zone.
  rates        with Wilson 95 % intervals.
"""
from __future__ import annotations

import math

import numpy as np


# ------------------------------------------------------------------ basic statistics
def stats(values) -> dict:
    v = np.asarray([x for x in values if x is not None and np.isfinite(x)], float)
    if len(v) == 0:
        return {"n": 0}
    return {"n": int(len(v)), "mean": round(float(v.mean()), 4), "rmse": round(float(np.sqrt((v**2).mean())), 4),
            "median": round(float(np.median(v)), 4), "p95": round(float(np.percentile(v, 95)), 4),
            "min": round(float(v.min()), 4), "max": round(float(v.max()), 4), "std": round(float(v.std(ddof=1)) if len(v) > 1 else 0.0, 4)}


def wilson(k: int, n: int, z: float = 1.96) -> tuple:
    """Wilson score interval of a proportion k/n (95 % by default); (nan, nan) for n = 0."""
    if n == 0:
        return (float("nan"), float("nan"))
    p = k/n
    d = 1+z*z/n
    c = (p+z*z/(2*n))/d
    h = z*math.sqrt(p*(1-p)/n+z*z/(4*n*n))/d
    return (round(max(0.0, c-h), 4), round(min(1.0, c+h), 4))


def rate(k: int, n: int) -> dict:
    lo, hi = wilson(k, n)
    return {"k": int(k), "n": int(n), "rate": round(k/n, 4) if n else None, "ci95": [lo, hi]}


def wrap(a):
    return (np.asarray(a, float)+np.pi) % (2*np.pi)-np.pi


# ------------------------------------------------------------------ trajectories
def fit_se2(src, dst):
    """(R 2x2, t) minimising |R src + t - dst| (rigid, no scale; Umeyama/Kabsch in 2-D)."""
    src, dst = np.asarray(src, float), np.asarray(dst, float)
    ms, md = src.mean(axis=0), dst.mean(axis=0)
    H = (src-ms).T@(dst-md)
    U, _, Vt = np.linalg.svd(H)
    D = np.diag([1.0, np.sign(np.linalg.det(Vt.T@U.T))])
    R = Vt.T@D@U.T
    return R, md-R@ms


def ate(est_xy, truth_xy, align: bool = False) -> dict:
    """Absolute trajectory error of time-matched positions; align=True first fits a rigid 2-D transform."""
    est, truth = np.asarray(est_xy, float), np.asarray(truth_xy, float)
    if align:
        R, t = fit_se2(est, truth)
        est = est@R.T+t
    d = np.linalg.norm(est-truth, axis=1)
    out = stats(d)
    out["final"] = round(float(d[-1]), 4)
    return out


def rpe(est, truth, segment_m: float = 10.0, step: int = 5) -> dict:
    """Relative pose error over path segments of `segment_m` (truth path length): est/truth (N, 3) x, y, yaw.

    For each start i, j is the first sample segment_m further along the true path; the motion i -> j in the
    start's own frame is compared. Returns position error in % of the segment and heading error in deg per metre.
    """
    est, truth = np.asarray(est, float), np.asarray(truth, float)
    s = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(truth[:, :2], axis=0), axis=1))]
    pct, yaw = [], []
    for i in range(0, len(truth), step):
        j = int(np.searchsorted(s, s[i]+segment_m))
        if j >= len(truth):
            break
        rel = []
        for p in (est, truth):
            c, sn = math.cos(p[i, 2]), math.sin(p[i, 2])
            dx, dy = p[j, 0]-p[i, 0], p[j, 1]-p[i, 1]
            rel.append((c*dx+sn*dy, -sn*dx+c*dy, float(wrap(p[j, 2]-p[i, 2]))))
        length = s[j]-s[i]
        pct.append(100*math.hypot(rel[0][0]-rel[1][0], rel[0][1]-rel[1][1])/length)
        yaw.append(math.degrees(abs(float(wrap(rel[0][2]-rel[1][2]))))/length)
    return {"segment_m": segment_m, "segments": len(pct), "position_pct": stats(pct), "heading_deg_per_m": stats(yaw)}


# ------------------------------------------------------------------ place graph
def truth_place_fn(truth: dict):
    """Vectorised truth place of points, as the simulator defines places (semantic_world.place_of): the zone whose
    bounds hold the point, else the aisle segment (building) holding it, else the bridge between buildings."""
    zones = [(z["id"], z["bounds"]) for z in truth["zones"]]
    buildings = list(truth.get("aisle_segments", {}).items())

    def place(x, y):
        out = np.full(len(x), "bridge", dtype=object)
        done = np.zeros(len(x), bool)
        for pid, b in zones+buildings:
            hit = ~done & (x >= b[0]) & (x <= b[2]) & (y >= b[1]) & (y <= b[3])
            out[hit] = pid
            done |= hit
        return out
    return place


def match_places(labels, places, origin, res, truth_place, min_share=0.5) -> dict:
    """Predicted place id -> truth place id: the truth place holding most of the predicted place's free cells.

    labels: per-cell place index (g1_factory.map_package.label_free_space); truth_place: f(x, y) -> truth ids
    (truth_place_fn). None when no truth place holds at least min_share of the cells.
    """
    out = {}
    for k, p in enumerate(places):
        rows, cols = np.nonzero(labels == k)
        if len(rows) == 0:
            out[p["id"]] = None
            continue
        ids, counts = np.unique(truth_place(origin[0]+(cols+0.5)*res, origin[1]+(rows+0.5)*res), return_counts=True)
        j = int(np.argmax(counts))
        out[p["id"]] = str(ids[j]) if counts[j] >= min_share*len(rows) else None
    return out


def doorway_metrics(pred_edges, truth_edges, place_map, door_width_m=6.0) -> dict:
    """Detection and position error of doorways (edges between places).

    pred_edges/truth_edges: [{a, b, door, (width_m)}]; place_map: predicted id -> truth id (match_places).
    A predicted edge is a true positive when its mapped pair is a truth edge (each truth edge counted once, the
    closest doorway wins). Contacts up to door_width_m wide are "door" (an opening in a wall: doors 1.7-1.8 m,
    corridor mouths 3-4 m here), wider ones "open" (a zone marked on the floor without walls, contact tens of m).
    """
    truth = {frozenset((e["a"], e["b"])): e for e in truth_edges}
    candidates = {}
    unmatched = []
    for e in pred_edges:
        a, b = place_map.get(e["a"]), place_map.get(e["b"])
        key = frozenset((a, b)) if a and b and a != b else None
        if key in truth:
            d = math.hypot(e["door"][0]-truth[key]["door"][0], e["door"][1]-truth[key]["door"][1])
            if key not in candidates or d < candidates[key][1]:
                if key in candidates:
                    unmatched.append(candidates[key][0])
                candidates[key] = (e, d)
            else:
                unmatched.append(e)
        else:
            unmatched.append(e)
    tp = len(candidates)
    fp, fn = len(unmatched), len(truth)-tp
    precision = tp/(tp+fp) if tp+fp else None
    recall = tp/len(truth) if truth else None
    f1 = 2*precision*recall/(precision+recall) if precision and recall else 0.0
    detected = []
    for key, (e, d) in sorted(candidates.items(), key=lambda kv: sorted(kv[0])):
        kind = "door" if e.get("width_m", 0.0) <= door_width_m else "open"
        detected.append({"truth": sorted(key), "pred": [e["a"], e["b"]], "error_m": round(d, 3), "kind": kind,
                         "width_m": e.get("width_m")})
    return {"truth": len(truth), "predicted": len(pred_edges), "tp": tp, "fp": fp, "fn": fn,
            "precision": round(precision, 4) if precision is not None else None,
            "recall": round(recall, 4) if recall is not None else None, "f1": round(f1, 4),
            "recall_ci95": list(wilson(tp, len(truth))), "error_m": stats([d["error_m"] for d in detected]),
            "error_m_door": stats([d["error_m"] for d in detected if d["kind"] == "door"]),
            "error_m_open": stats([d["error_m"] for d in detected if d["kind"] == "open"]),
            "detected": detected, "missed": [sorted(k) for k in truth if k not in candidates],
            "false": [[e["a"], e["b"]] for e in unmatched]}


def iou(a, b) -> float:
    ix = max(0.0, min(a[2], b[2])-max(a[0], b[0]))
    iy = max(0.0, min(a[3], b[3])-max(a[1], b[1]))
    inter = ix*iy
    union = (a[2]-a[0])*(a[3]-a[1])+(b[2]-b[0])*(b[3]-b[1])-inter
    return inter/union if union > 0 else 0.0


def room_iou(pred_zones, truth_zones) -> dict:
    """Annotated room bounds vs truth zone bounds, each truth zone matched to the best-overlapping room."""
    out = {}
    for t in truth_zones:
        best = max((iou(z["bounds"], t["bounds"]) for z in pred_zones), default=0.0)
        out[t["id"]] = round(best, 4)
    return {"per_zone": out, "mean": stats(out.values()).get("mean")}


# ------------------------------------------------------------------ zones and fixes
def confusion(pairs) -> dict:
    """[(truth, predicted)] -> {"accuracy": rate, "per_zone": {truth: rate}, "matrix": {truth: {pred: n}}}."""
    matrix, ok = {}, 0
    for t, p in pairs:
        matrix.setdefault(t, {})
        matrix[t][p] = matrix[t].get(p, 0)+1
        ok += t == p
    per = {t: rate(row.get(t, 0), sum(row.values())) for t, row in sorted(matrix.items())}
    return {"accuracy": rate(ok, len(pairs)), "per_zone": per, "matrix": matrix}


def fix_outcome(localized: bool, err_m: float | None, err_deg: float | None, tol_m=0.5, tol_deg=5.0) -> str:
    if not localized:
        return "refused"
    return "correct" if err_m is not None and err_m <= tol_m and err_deg <= tol_deg else "wrong"
