import base64
import json
import unittest
from unittest import mock

from backends import ollama, openai_compat, opencode_cli, template


GOOD = [{"step": 1, "skill": "navigate", "target": "3", "relation": "front", "offset_m": 0.2}]


class TestTemplate(unittest.TestCase):
    def test_ok(self):
        goals, err = template.plan({"objects": [{"id": 3, "class_name": "puck"},
                                                 {"id": 7, "class_name": "goal_zone"}]})
        self.assertIsNone(err)
        self.assertEqual(len(goals), 4)

    def test_no_objects(self):
        self.assertEqual(template.plan({"objects": []})[1], "puck·goal_zone 없음 — 계획 보류")


class TestOpencodeParser(unittest.TestCase):
    def test_single_json(self):
        self.assertEqual(opencode_cli._goals_from_text(json.dumps({"goals": GOOD}))[0], GOOD)

    def test_wrapped(self):
        inner = json.dumps({"goals": GOOD})
        self.assertEqual(opencode_cli._goals_from_text(json.dumps({"response": inner}))[0], GOOD)

    def test_embedded_array(self):
        self.assertEqual(opencode_cli._goals_from_text("계획:\n" + json.dumps(GOOD))[0], GOOD)

    def test_jsonl_stream(self):
        stream = "\n".join([
            json.dumps({"type": "step_start", "sessionID": "ses_x"}),
            json.dumps({"type": "text", "part": {"type": "text", "text": "계획: " + json.dumps(GOOD)}}),
            json.dumps({"type": "step_finish"}),
        ])
        self.assertEqual(opencode_cli._goals_from_text(stream)[0], GOOD)

    def test_quota_error_text(self):
        err = json.dumps({"type": "error", "error": {"type": "provider.quota", "message": "429"}})
        self.assertEqual(opencode_cli._goals_from_text(err), (None, "응답에 goals 없음"))

    def test_empty(self):
        self.assertEqual(opencode_cli._goals_from_text("  "), (None, "빈 출력"))


class FakeProc:
    def __init__(self, stdout="", returncode=0, stderr=""):
        self.stdout, self.returncode, self.stderr = stdout, returncode, stderr


class TestOpencodePlan(unittest.TestCase):
    def test_cmd_and_file(self):
        import os
        seen = {}
        with open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "data",
                               "pole_cam.jpg"), "rb") as f:
            import base64 as b64mod
            img = b64mod.b64encode(f.read()).decode()

        def fake_run(cmd, **kw):
            seen["cmd"] = cmd
            return FakeProc(stdout=json.dumps({"goals": GOOD}))

        with mock.patch("shutil.which", return_value="/usr/bin/opencode"), \
            mock.patch("subprocess.run", fake_run):
            goals, err = opencode_cli.plan(
                {"command": "x", "objects": [], "images": [{"data_b64": img}]})
        self.assertEqual((goals, err), (GOOD, None))
        cmd = seen["cmd"]
        self.assertEqual(cmd[:4], ["opencode", "run", "--format", "json"])
        self.assertNotIn("--model", cmd)  # default면 생략
        fi = [i for i, c in enumerate(cmd) if c == "--file"]
        self.assertEqual(len(fi), 1)
        self.assertTrue(cmd[fi[0] + 1].endswith(".jpg"))
        self.assertIn("도구 사용 없이", cmd[-1])

    def test_model_forwarded(self):
        def fake_run(cmd, **kw):
            self.assertEqual(cmd[4:6], ["--model", "opencode-go/muse-spark-1.3-contributor"])
            return FakeProc(stdout=json.dumps(GOOD))

        with mock.patch("shutil.which", return_value="/usr/bin/opencode"), \
            mock.patch("subprocess.run", fake_run):
            self.assertEqual(opencode_cli.plan(
                {"objects": []}, model="opencode-go/muse-spark-1.3-contributor")[0], GOOD)

    def test_no_cli(self):
        with mock.patch("shutil.which", return_value=None):
            self.assertEqual(opencode_cli.plan({"objects": []}), (None, "opencode CLI 없음"))

    def test_bad_base64_skipped(self):
        seen = {}

        def fake_run(cmd, **kw):
            seen["cmd"] = cmd
            return FakeProc(stdout=json.dumps(GOOD))

        with mock.patch("shutil.which", return_value="/usr/bin/opencode"), \
            mock.patch("subprocess.run", fake_run):
            goals, err = opencode_cli.plan(
                {"objects": [], "images": [{"data_b64": "!!!"}]})
        self.assertEqual((goals, err), (GOOD, None))  # 깨진 이미지는 버리고 진행
        self.assertNotIn("--file", seen["cmd"])


class TestOpenaiCompat(unittest.TestCase):
    def test_no_key_remote(self):
        with mock.patch.dict("os.environ", {}, clear=False):
            import os
            os.environ.pop("OPENAI_API_KEY", None)
            self.assertEqual(openai_compat.plan({"objects": []}), (None, "OPENAI_API_KEY 없음"))

    def test_no_key_local_tries_http(self):
        import os
        os.environ.pop("OPENAI_API_KEY", None)
        goals, err = openai_compat.plan({"objects": []}, endpoint="http://127.0.0.1:9/v1",
                                        model="x", timeout_s=2)
        self.assertIsNone(goals)
        self.assertTrue(err.startswith("http 실패"))


class TestOllama(unittest.TestCase):
    def test_defaults(self):
        seen = {}

        def fake(req, **kw):
            seen.update(kw)
            return None, "x"

        with mock.patch("backends.openai_compat.plan", fake):
            ollama.plan({"objects": []})
        self.assertEqual(seen, {"endpoint": "http://localhost:11434/v1",
                                "model": "llama4", "timeout_s": 120})

    def test_explicit_passthrough(self):
        seen = {}

        def fake(req, **kw):
            seen.update(kw)
            return None, "x"

        with mock.patch("backends.openai_compat.plan", fake):
            ollama.plan({"objects": []}, model="m", endpoint="http://x:1/v1", timeout_s=5)
        self.assertEqual(seen, {"endpoint": "http://x:1/v1", "model": "m", "timeout_s": 5})


if __name__ == "__main__":
    unittest.main()
