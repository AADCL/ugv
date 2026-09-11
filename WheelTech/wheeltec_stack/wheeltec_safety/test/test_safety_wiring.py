#!/usr/bin/env python3

import ast
from pathlib import Path
import unittest
import xml.etree.ElementTree as element_tree


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SCRIPT_PATH = PACKAGE_ROOT / "scripts" / "cmd_vel_safety_gate.py"


class SafetyWiringTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tree = ast.parse(
            SCRIPT_PATH.read_text(encoding="utf-8"), filename=str(SCRIPT_PATH)
        )
        gate = next(
            node
            for node in cls.tree.body
            if isinstance(node, ast.ClassDef) and node.name == "CmdVelSafetyGate"
        )
        cls.methods = {
            node.name: node
            for node in gate.body
            if isinstance(node, ast.FunctionDef)
        }

    def test_launch_does_not_expose_velocity_topic_overrides(self):
        launch = element_tree.parse(
            str(PACKAGE_ROOT / "launch" / "include" / "cmd_vel_safety.launch.xml")
        ).getroot()
        argument_names = {node.attrib["name"] for node in launch.findall("arg")}
        self.assertNotIn("input_topic", argument_names)
        self.assertNotIn("output_topic", argument_names)
        node = launch.find("node")
        self.assertEqual(node.attrib.get("name"), "wheeltec_safety")

        config = (PACKAGE_ROOT / "config" / "cmd_vel_safety.yaml").read_text(
            encoding="utf-8"
        )
        self.assertNotIn("\ninput_topic:", "\n" + config)
        self.assertNotIn("\noutput_topic:", "\n" + config)

    def test_all_perception_commits_advance_safety_generation(self):
        callbacks = (
            "cloud_callback",
            "raw_cloud_callback",
            "costmap_callback",
            "costmap_update_callback",
        )
        for callback in callbacks:
            method = self.methods[callback]
            advances = [
                node
                for node in ast.walk(method)
                if isinstance(node, ast.AugAssign)
                and isinstance(node.target, ast.Attribute)
                and node.target.attr == "safety_input_generation"
            ]
            self.assertEqual(len(advances), 1, callback)
            serialized_commits = [
                node
                for node in ast.walk(method)
                if isinstance(node, ast.With)
                and any(
                    isinstance(item.context_expr, ast.Attribute)
                    and item.context_expr.attr == "output_lock"
                    for item in node.items
                )
                and advances[0] in tuple(ast.walk(node))
            ]
            self.assertEqual(len(serialized_commits), 1, callback)

    def test_final_publish_is_guarded_by_all_generations(self):
        run_cycle = self.methods["_run_cycle_attempt"]
        calls = [node for node in ast.walk(run_cycle) if isinstance(node, ast.Call)]
        guards = [
            node
            for node in calls
            if isinstance(node.func, ast.Name)
            and node.func.id == "snapshot_is_current"
        ]
        self.assertEqual(len(guards), 1)
        self.assertEqual(len(guards[0].args), 6)
        publishes = [
            node
            for node in calls
            if isinstance(node.func, ast.Attribute)
            and node.func.attr == "publish"
        ]
        self.assertTrue(publishes)
        self.assertLess(guards[0].lineno, min(node.lineno for node in publishes))

    def test_goal_and_command_callbacks_enforce_sources(self):
        init_nodes = tuple(ast.walk(self.methods["__init__"]))
        init_names = {
            node.id for node in init_nodes if isinstance(node, ast.Name)
        }
        init_attributes = {
            node.attr for node in init_nodes if isinstance(node, ast.Attribute)
        }
        self.assertIn("MoveBaseActionGoal", init_names)
        self.assertIn("move_base_goal_callback", init_attributes)
        self.assertIn("get_name", init_attributes)
        self.assertIn("remap_name", init_attributes)

        source = SCRIPT_PATH.read_text(encoding="utf-8")
        self.assertNotIn('get_param("~input_topic"', source)
        self.assertNotIn('get_param("~output_topic"', source)
        command_names = {
            node.id
            for node in ast.walk(self.methods["command_callback"])
            if isinstance(node, ast.Name)
        }
        command_strings = {
            node.value
            for node in ast.walk(self.methods["command_callback"])
            if isinstance(node, ast.Constant)
            and isinstance(node.value, str)
        }
        self.assertIn("command_caller_is_expected", command_names)
        self.assertIn("unexpected_command_publisher", command_strings)


if __name__ == "__main__":
    unittest.main()
