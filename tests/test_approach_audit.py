"""Exercise actual act() with stub ROS I/O; this is not a ROS/DDS integration test."""
import ast
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from g1_factory.approach_validation import MODES, alternatives
from g1_factory.planning import CostMap, OccupancyGrid, PlanningError
from g1_factory.roadmap import region_at
from g1_factory.vln_navigator import already_in_front, approach_pose, behind_partition, outside_region


def load_act():
    path = Path(__file__).resolve().parents[1]/'g1_factory/ros2/vln_navigator_node.py'
    cls = next(n for n in ast.parse(path.read_text()).body
               if isinstance(n, ast.ClassDef) and n.name == 'VLNNavigatorNode')
    fn = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'act')
    scope = dict(np=np, json=json, alternatives=alternatives, behind_partition=behind_partition,
                 approach_pose=approach_pose, already_in_front=already_in_front,
                 outside_region=outside_region, region_at=region_at, PlanningError=PlanningError,
                 M=SimpleNamespace(image_to_numpy=lambda m: m.data, depth_to_metres=lambda m: m.data,
                                   to_seconds=lambda t: t))
    exec(compile(ast.Module(body=[fn], type_ignores=[]), str(path), 'exec'), scope)
    return scope['act']


class StubNode:
    run_act = staticmethod(load_act())

    def __init__(self, mode, scene, path):
        self.eval_mode, self.scene, self.audit_dir = mode, scene, path
        self.standoff, self.pose = 1.4, [1., 1., 0.]
        rgb = SimpleNamespace(data=np.zeros((3, 3, 3), np.uint8), header=SimpleNamespace(stamp=1.))
        self.frame, self.info = (rgb, None, np.eye(4)), SimpleNamespace(k=[1, 0, 1, 0, 1, 1, 0, 0, 1])
        self.navigator = SimpleNamespace(history=[], task=None)
        self.state, self.sent, self.done = 'deciding', [], False
        grid = OccupancyGrid((0, 0), .1, np.zeros((100, 120), np.int8))
        if scene == 'thin':
            grid.data[:, 59:61] = 100
        elif scene == 'thick':
            grid.data[:, 50:70] = 100
        self.costmap = CostMap(grid)

    def now(self):
        return 1.

    def act(self, decision):
        return self.run_act(self, decision)

    def localize(self, bbox):
        if self.scene == 'no_depth':
            return None
        values = ([2., 1., 0.], [1.9, 1., 0.]) if self.scene == 'already_close' else ([6., 1., 0.], [5.8, 1., 0.])
        return tuple(np.array(v) for v in values)

    def report(self, *args, **kwargs):
        pass

    def segment(self, goal):
        if self.scene == 'unreachable':
            raise PlanningError('controlled unreachable path')
        return np.array([self.pose[:2], goal[:2]])

    def send_path(self, points, yaw, phase):
        self.sent.append((points.tolist(), yaw, phase))

    def finish_step(self, timestamp):
        self.done, self.state = True, 'idle'


def decision():
    return SimpleNamespace(action='approach', bbox=(0, 0, 2, 2), prompt_image=None, thought='',
                           latency_s=0., request_bytes=0, note='', raw='', plate=None, waypoint=None)


@pytest.mark.parametrize('scene', ['thin', 'thick', 'free', 'already_close', 'no_depth', 'unreachable'])
@pytest.mark.parametrize('mode', ['disabled', *MODES])
def test_selected_mode_and_audit_outcomes(scene, mode, tmp_path):
    node = StubNode(mode, scene, tmp_path)
    node.act(decision())
    if mode == 'disabled':
        assert not list(tmp_path.iterdir())
        return
    packets = list(tmp_path.glob('*.npz'))
    assert len(packets) == 1
    with np.load(packets[0]) as packet:
        assert {'rgb', 'depth_m', 'map_from_camera', 'intrinsics', 'bbox', 'rgb_stamp', 'depth_stamp'} <= set(packet.files)
    events = [json.loads(line) for line in (tmp_path/'events.jsonl').read_text().splitlines()]
    assert all(e['id'] == packets[0].stem and e['mode'] == mode for e in events)
    if scene == 'no_depth':
        assert [e['outcome'] for e in events] == ['no_depth']
    else:
        assert events[0]['outcome'] == 'candidates'
        held = events[0]['options'][mode]['hold']
        expected = ('hold' if held else 'already_close' if scene == 'already_close' else
                    'unreachable' if scene == 'unreachable' else 'path_sent')
        assert events[-1]['outcome'] == expected
        assert bool(node.sent) == (expected == 'path_sent')
    if mode == 'full':
        reference = StubNode('disabled', scene, tmp_path)
        reference.act(decision())
        assert (node.sent, node.state, node.done) == (reference.sent, reference.state, reference.done)
