import math

import numpy as np
import pytest

from g1_factory import metrics as X
from g1_factory import slam_map as S


def test_wilson_interval_matches_known_values():
    lo, hi = X.wilson(8, 10)
    assert lo == pytest.approx(0.4902, abs=1e-3) and hi == pytest.approx(0.9433, abs=1e-3)
    assert X.wilson(0, 0) != X.wilson(0, 0)  # nan for no trials
    assert X.wilson(10, 10)[1] == 1.0
    assert X.rate(3, 4)["rate"] == 0.75


def test_ate_best_fit_removes_a_rigid_offset_but_start_alignment_does_not():
    rng = np.random.default_rng(0)
    truth = np.c_[np.linspace(0, 30, 200), 5*np.sin(np.linspace(0, 3, 200))]
    c, s = math.cos(0.2), math.sin(0.2)
    est = truth@np.array([[c, s], [-s, c]])+[3.0, -1.0]+rng.normal(0, 0.01, truth.shape)
    assert X.ate(est, truth)["rmse"] > 1.0
    assert X.ate(est, truth, align=True)["rmse"] < 0.02


def test_rpe_measures_drift_per_segment():
    n = 400
    x = np.linspace(0, 100, n)
    truth = np.c_[x, np.zeros(n), np.zeros(n)]
    est = np.c_[x*1.02, np.zeros(n), np.zeros(n)]  # 2 % too long: 0.2 m per 10 m segment
    r = X.rpe(est, truth, 10.0)
    assert r["position_pct"]["mean"] == pytest.approx(2.0, abs=0.1)
    assert r["heading_deg_per_m"]["max"] == pytest.approx(0.0, abs=1e-9)
    bent = np.c_[x, np.zeros(n), np.radians(0.5)*x]  # heading drifts 0.5 deg per metre
    assert X.rpe(bent, truth, 10.0)["heading_deg_per_m"]["mean"] == pytest.approx(0.5, abs=0.02)


def test_places_are_matched_by_where_their_cells_lie_not_by_name():
    truth = {"zones": [{"id": "room_a", "bounds": [0, 0, 5, 5]}],
             "aisle_segments": {"hall": [0, 0, 20, 10]}}
    place = X.truth_place_fn(truth)
    assert list(place(np.array([2.0, 12.0, 30.0]), np.array([2.0, 2.0, 2.0]))) == ["room_a", "hall", "bridge"]
    labels = np.full((100, 200), -1)
    labels[:50, :50] = 0     # predicted "kitchen" covers room_a
    labels[60:, :] = 1       # predicted "corridor" lies in the hall
    places = [{"id": "kitchen"}, {"id": "corridor"}, {"id": "empty"}]
    m = X.match_places(labels, places, (0.0, 0.0), 0.1, place)
    assert m == {"kitchen": "room_a", "corridor": "hall", "empty": None}


def test_doorway_metrics_count_hits_misses_and_false_doorways():
    truth = [{"a": "room_a", "b": "hall", "door": [5.0, 2.0]}, {"a": "room_b", "b": "hall", "door": [10.0, 5.0]}]
    pred = [{"a": "kitchen", "b": "corridor", "door": [5.3, 2.4], "width_m": 1.8},   # room_a-hall, 0.5 m off
            {"a": "kitchen", "b": "store", "door": [1.0, 1.0], "width_m": 2.0}]      # not in the truth
    place_map = {"kitchen": "room_a", "corridor": "hall", "store": "room_c"}
    r = X.doorway_metrics(pred, truth, place_map)
    assert (r["tp"], r["fp"], r["fn"]) == (1, 1, 1)
    assert r["precision"] == 0.5 and r["recall"] == 0.5 and r["f1"] == 0.5
    assert r["error_m"]["mean"] == pytest.approx(0.5)
    assert r["detected"][0]["kind"] == "door" and r["missed"] == [["hall", "room_b"]]


def test_automatic_topology_against_the_ideal_annotator_finds_every_doorway():
    from g1_factory.map_package import build_semantic, label_free_space
    g = np.full((100, 200), S.FREE, np.int8)   # 20 x 10 m: two rooms over a corridor, a door each
    g[[0, -1], :] = S.OCCUPIED
    g[:, [0, -1]] = S.OCCUPIED
    g[50, :] = S.OCCUPIED
    g[50, 40:52] = S.FREE
    g[50, 140:152] = S.FREE
    g[50:, 100] = S.OCCUPIED
    truth = {"zones": [{"id": "west", "bounds": [0, 5, 10, 10]}, {"id": "east", "bounds": [10, 5, 20, 10]}],
             "aisle_segments": {"corridor": [0, 0, 20, 5]},
             "edges": [{"a": "west", "b": "corridor", "door": [4.6, 5.0]}, {"a": "east", "b": "corridor", "door": [14.6, 5.0]}]}
    ann = {"zones": [{"id": "w", "label": "West", "kind": "room", "rects": [[0.3, 5.3, 9.7, 9.7]]},
                     {"id": "e", "label": "East", "kind": "room", "rects": [[10.3, 5.3, 19.7, 9.7]]},
                     {"id": "c", "label": "Corridor", "kind": "corridor", "rects": [[0.3, 0.3, 19.7, 4.7]]}]}
    from g1_factory.map_package import load_annotations
    a = load_annotations(ann)
    labels, places = label_free_space(g, (0.0, 0.0), 0.1, a["zones"])
    sem = build_semantic(g, (0.0, 0.0), 0.1, ann)
    pm = X.match_places(labels, places, (0.0, 0.0), 0.1, X.truth_place_fn(truth))
    r = X.doorway_metrics(sem["edges"], truth["edges"], pm)
    assert r["recall"] == 1.0 and r["precision"] == 1.0
    assert r["error_m"]["max"] < 0.3


def test_zone_confusion_and_fix_outcomes():
    c = X.confusion([("a", "a"), ("a", "b"), ("b", "b"), ("b", "b")])
    assert c["accuracy"]["k"] == 3 and c["per_zone"]["a"]["rate"] == 0.5 and c["matrix"]["a"]["b"] == 1
    assert X.fix_outcome(False, None, None) == "refused"
    assert X.fix_outcome(True, 0.2, 1.0) == "correct"
    assert X.fix_outcome(True, 3.0, 1.0) == "wrong"
    assert X.fix_outcome(True, 0.2, 9.0) == "wrong"
