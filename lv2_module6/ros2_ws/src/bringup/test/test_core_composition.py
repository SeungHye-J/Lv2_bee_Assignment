#!/usr/bin/env python3
"""core_composition 계약 (11-16 메모리) — 컨테이너로 묶어도 따로 띄울 때와 노드 이름·파라미터가 같은지.
launch 동작(action)만 만들어 조건을 평가 — 노드는 띄우지 않음."""
import importlib.util
from pathlib import Path
import unittest

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.utilities import perform_substitutions
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.utilities import evaluate_parameters

ROOT = Path(__file__).resolve().parents[1]
COMPOSED = {'arm_safety_monitor', 'manual_control', 'body_filter', 'signal_decoder', 'coordinator', 'mission_manager',
            'grasp_scorer'}


def load(path):
    spec = importlib.util.spec_from_file_location(path.stem.replace('.', '_'), path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class CoreCompositionTests(unittest.TestCase):
    def setUp(self):
        self.mod = load(ROOT / 'launch/bringup.launch.py')

    def context(self, **args):
        ctx = LaunchContext()
        ld = self.mod.generate_launch_description()
        for e in ld.entities:
            if isinstance(e, DeclareLaunchArgument):
                ctx.launch_configurations[e.name] = perform_substitutions(ctx, e.default_value)
        # setup_map이 정하는 것 (지도 없이 · 인지 켬)
        ctx.launch_configurations.update({'use_perception': 'true', 'use_hand_relative': 'false', 'map_mode': 'NONE',
                                          'use_slam': 'false', 'use_ros_system': 'false', **args})
        return ctx, ld

    def active(self, **args):
        """{노드 이름: (어디 — 'process' | 컨테이너 이름, 파라미터 dict)}"""
        ctx, ld = self.context(**args)
        out = {}
        for e in ld.entities:
            if isinstance(e, ComposableNodeContainer):
                if e.condition is not None and not e.condition.evaluate(ctx):
                    continue
                where = e._Node__node_name
                where = where if isinstance(where, str) else perform_substitutions(ctx, where)
                for c in e._ComposableNodeContainer__composable_node_descriptions:
                    if c.condition() is not None and not c.condition().evaluate(ctx):
                        continue
                    name = perform_substitutions(ctx, c.node_name)
                    self.assertNotIn(name, out)
                    out[name] = (where, self.params(ctx, c.parameters))
            elif isinstance(e, Node):
                if e.condition is not None and not e.condition.evaluate(ctx):
                    continue
                exe = e.node_executable
                name = exe if isinstance(exe, str) else perform_substitutions(ctx, exe)
                self.assertNotIn(name, out)
                out[name] = ('process', self.params(ctx, e._Node__parameters))
        return out

    @staticmethod
    def params(ctx, parameters):
        merged = {}
        for p in evaluate_parameters(ctx, parameters or []):
            merged.update(p if isinstance(p, dict) else {'file': str(p)})
        return merged

    def test_same_nodes_and_params(self):
        for args in ({}, {'start_pose': '[0.7, 0.9, 0.0]'}, {'use_mission': 'false'}, {'use_perception': 'false'},
                     {'use_grasp_scorer': 'false', 'use_mission_manager': 'false'}):
            sep, comp = self.active(core_composition='false', **args), self.active(core_composition='true', **args)
            self.assertEqual(set(sep), set(comp), args)
            for name in sep:
                self.assertEqual(sep[name][1], comp[name][1], (name, args))
                self.assertEqual(comp[name][0] != 'process', name in COMPOSED, (name, args))
                self.assertEqual(sep[name][0], 'process', (name, args))

    def test_containers(self):
        comp = self.active(core_composition='true')
        self.assertEqual({n for n, (w, _) in comp.items() if w == 'control_nodes'},
                         {'arm_safety_monitor', 'manual_control', 'body_filter', 'signal_decoder'})
        self.assertEqual({n for n, (w, _) in comp.items() if w == 'mission_nodes'},
                         {'coordinator', 'mission_manager', 'grasp_scorer'})
        self.assertEqual(comp['state_manager'][0], 'process')  # 재시작 담당은 따로
        self.assertEqual(comp['perception_node'][0], 'process')  # 무거움 — 따로


if __name__ == '__main__':
    unittest.main()
