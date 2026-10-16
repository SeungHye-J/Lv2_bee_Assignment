import base64
import unittest
from unittest import mock

import server


GOOD = [
    {"step": 1, "skill": "navigate", "target": "3", "relation": "front", "offset_m": 0.2},
    {"step": 2, "skill": "pick", "target": "3", "relation": "", "offset_m": 0.0},
]
OBJS = [{"id": 3, "class_name": "puck"}, {"id": 7, "class_name": "goal_zone"}]


class TestValidate(unittest.TestCase):
    def test_ok(self):
        self.assertEqual(server.validate(GOOD, OBJS), "")

    def test_empty(self):
        self.assertEqual(server.validate([], OBJS), "empty")

    def test_bad_order(self):
        bad = [dict(GOOD[0], step=2), dict(GOOD[1], step=1)]
        self.assertIn("순서", server.validate(bad, OBJS))

    def test_bad_skill(self):
        self.assertIn("skill", server.validate([dict(GOOD[0], skill="dance")], OBJS))

    def test_unknown_id(self):
        self.assertIn("not tracked", server.validate([dict(GOOD[0], target="9")], OBJS))

    def test_say_text_target_ok(self):
        say = [{"step": 1, "skill": "say", "target": "퍽을 옮겼어요", "relation": "", "offset_m": 0.0}]
        self.assertEqual(server.validate(say, OBJS), "")


class TestCheckImages(unittest.TestCase):
    def test_ok(self):
        self.assertEqual(server.check_images([{"data_b64": base64.b64encode(b"x" * 100).decode()}]), "")

    def test_bad_base64(self):
        self.assertEqual(server.check_images([{"data_b64": "!!!"}]), "이미지 base64 오류")

    def test_oversize(self):
        big = base64.b64encode(b"x" * 600_000).decode()
        self.assertIn("500KB", server.check_images([{"data_b64": big}]))

    def test_empty_list_ok(self):
        self.assertEqual(server.check_images([]), "")


class TestLogPlan(unittest.TestCase):
    def test_per_request_dir(self):
        import base64
        import json
        import os
        import tempfile
        from pathlib import Path

        tmp = Path(tempfile.mkdtemp())
        data = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")
        with open(os.path.join(data, "pole_cam.jpg"), "rb") as f:
            img = base64.b64encode(f.read()).decode()
        req = {"command": "빨간 퍽", "objects": [{"id": 3}],
               "images": [{"data_b64": img}, {"data_b64": img}]}
        with mock.patch.object(server, "HERE", tmp):
            server.log_plan(req, 200, {"ok": True, "goals": []})
        dirs = list((tmp / "log").iterdir())
        self.assertEqual(len(dirs), 1)
        d = dirs[0]
        self.assertEqual(
            json.loads((d / "request.json").read_text())["images"],
            [{"file": "request-image1.png", "bytes": len((d / "request-image1.png").read_bytes())},
             {"file": "request-image2.png", "bytes": len((d / "request-image2.png").read_bytes())}])
        self.assertEqual(json.loads((d / "response.json").read_text())["code"], 200)
        for i in (1, 2):
            self.assertTrue((d / f"request-image{i}.png").read_bytes().startswith(b"\x89PNG"))

    def test_no_images_no_files(self):
        import json
        import tempfile
        from pathlib import Path

        tmp = Path(tempfile.mkdtemp())
        with mock.patch.object(server, "HERE", tmp):
            server.log_plan({"command": "x"}, 400, {"ok": False})
        (d,) = list((tmp / "log").iterdir())
        self.assertEqual(json.loads((d / "request.json").read_text())["images"], [])
        self.assertFalse(list(d.glob("*.png")))


class TestDispatch(unittest.TestCase):
    def test_template(self):
        with mock.patch.dict(server.CONFIG, {"ai": {"backend": "template"}}):
            goals, err = server.dispatch({"objects": OBJS})
            self.assertIsNone(err)
            self.assertEqual(goals[0]["skill"], "navigate")

    def test_unknown_backend_falls_back(self):
        with mock.patch.dict(server.CONFIG, {"ai": {"backend": "뭐"}}):
            goals, err = server.dispatch({"objects": OBJS})
            self.assertIsNone(err)
            self.assertTrue(goals)

    def test_ollama_reads_ai_section(self):
        seen = {}

        def fake(req, **kw):
            seen.update(kw)
            return None, "x"

        with mock.patch.dict(server.CONFIG,
                              {"ai": {"backend": "ollama", "model": "qwen3", "timeout_s": 30}}), \
            mock.patch("backends.openai_compat.plan", fake):
            server.dispatch({"objects": []})
        self.assertEqual(seen, {"endpoint": "http://localhost:11434/v1",
                                "model": "qwen3", "timeout_s": 30})


if __name__ == "__main__":
    unittest.main()
