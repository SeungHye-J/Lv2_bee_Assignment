#!/usr/bin/env python3
"""호스트 VLM HTTP (표준 라이브러리만) — 컨테이너 vlm_task_planner가 호출.

  python3 server.py [--port 8000] [--host 127.0.0.1] [--config config.json]

  POST /plan {"command": "...", "objects": [{"id": 1, "class_name": "puck"}],
              "images": [{"data_b64": "<jpeg base64>"}],
              "replan": {"plan_id": 1, "failed_step": 2, "reason": "grasp_failed_all"},
              "history": [{"role": "user", "text": "..."}, {"role": "plan", "text": "..."}]}
    → {"ok": true, "goals": [{step, skill, target, relation, offset_m}]}
    → 못 짜면 {"ok": false, "error": "..."} (컨테이너는 템플릿으로 폴백)
  GET /health → {"ok": true, "backend": "..."}

백엔드는 backends/: template(기본·오프라인) · claude_cli · opencode_cli · ollama(로컬) · openai_compat(외부사).
출력은 coordinator 검증(허용 동작·대상 id·step 순서)과 같은 규칙으로 검사한다.
"""

import argparse
import base64
import json
import sys
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE / "backends"))

from backends import template  # noqa: E402

ALLOWED = {"navigate", "pick", "place", "say"}
CONFIG = {}


def validate(goals, objects):
    ids = {str(o.get("id")) for o in objects if isinstance(o, dict)}
    if not isinstance(goals, list) or not goals:
        return "empty"
    for i, g in enumerate(goals, 1):
        if not isinstance(g, dict):
            return f"step {i}: 객체 아님"
        if g.get("step") != i:
            return f"step {i}: 순서 어긋남({g.get('step')})"
        if g.get("skill") not in ALLOWED:
            return f"step {i}: skill {g.get('skill')}"
        t = str(g.get("target", ""))
        if g.get("skill") in ("navigate", "pick", "place") and t.isdigit() and t not in ids:
            return f"step {i}: target {t} not tracked"
        for im in ("images",):
            _ = im  # images는 검증 대상 아님 (용량 큰 base64 스킵)
    return ""


def check_images(images):
    total = 0
    for im in images[:2]:
        d = im.get("data_b64", "") if isinstance(im, dict) else ""
        if not d:
            continue
        try:
            raw = base64.b64decode(d, validate=True)
        except Exception:  # noqa: BLE001
            return "이미지 base64 오류"
        if isinstance(im, dict) and im.get("format", "jpeg") == "rgb8":
            w, h = im.get("width"), im.get("height")
            if not isinstance(w, int) or not isinstance(h, int):
                return "rgb8는 width·height(int) 필요"
            if len(raw) != w * h * 3:
                return "rgb8 크기 불일치"
        elif len(raw) > 500_000:
            return "이미지 500KB 초과 — 320px JPEG로 줄여라"
        total += len(raw)
    if total == 0 and images:
        return "이미지 비어 있음"
    return ""


def ai_conf():
    ai = CONFIG.get("ai") or {}
    return ai if isinstance(ai, dict) else {}


def dispatch(req):
    ai = ai_conf()
    backend = ai.get("backend", "template")
    if backend == "claude_cli":
        from backends import claude_cli
        return claude_cli.plan(req, model=ai.get("model", "default"),
                               timeout_s=ai.get("timeout_s", 60))
    if backend == "opencode_cli":
        from backends import opencode_cli
        return opencode_cli.plan(req, model=ai.get("model", "default"),
                                 timeout_s=ai.get("timeout_s", 60))
    if backend == "ollama":
        from backends import ollama
        return ollama.plan(req, model=ai.get("model"),
                           endpoint=ai.get("endpoint"),
                           timeout_s=ai.get("timeout_s"))
    if backend == "openai_compat":
        from backends import openai_compat
        return openai_compat.plan(req, endpoint=ai.get("endpoint", ""),
                                  model=ai.get("model", "default"),
                                  timeout_s=ai.get("timeout_s", 60))
    return template.plan(req)


def log_plan(req, code, resp):
    """요청 건당 log/plan-YYYYMMDD-HHMMSS-<rand>/ 에 request.json·response.json·request-imageN.png.
    이미지 원문은 PNG 파일로, request.json에는 파일명+용량만."""
    try:
        import uuid

        from images import to_png

        images = req.get("images", []) if isinstance(req, dict) else []
        pngs = []
        for im in images[:2]:
            d = im.get("data_b64", "") if isinstance(im, dict) else ""
            try:
                raw = base64.b64decode(d, validate=True)
            except Exception:  # noqa: BLE001
                raw = b""
            png = to_png(raw, im.get("format", "jpeg") if isinstance(im, dict) else "jpeg",
                         im.get("width") if isinstance(im, dict) else None,
                         im.get("height") if isinstance(im, dict) else None) if raw else None
            pngs.append(png)
        d = HERE / "log" / f"plan-{datetime.now():%Y%m%d-%H%M%S}-{uuid.uuid4().hex[:6]}"
        d.mkdir(parents=True, exist_ok=True)
        req_out = dict(req) if isinstance(req, dict) else {}
        req_out["images"] = [
            {"file": f"request-image{i + 1}.png" if png else None,
             "bytes": len(png) if png else 0}
            for i, png in enumerate(pngs)
        ]
        (d / "request.json").write_text(json.dumps(req_out, ensure_ascii=False, indent=2))
        (d / "response.json").write_text(json.dumps({"code": code, "resp": resp},
                                                    ensure_ascii=False, indent=2))
        for i, png in enumerate(pngs):
            if png:
                (d / f"request-image{i + 1}.png").write_bytes(png)
    except Exception:  # noqa: BLE001 — 로깅 실패가 서빙을 막지 않음
        pass


class Handler(BaseHTTPRequestHandler):
    def _send(self, code, obj):
        body = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path == "/health":
            self._send(200, {"ok": True, "backend": ai_conf().get("backend", "template")})
        else:
            self._send(404, {"ok": False, "error": "not found"})

    def do_POST(self):
        if self.path != "/plan":
            self._send(404, {"ok": False, "error": "not found"})
            return
        req = {}

        def done(code, resp):
            log_plan(req, code, resp)
            self._send(code, resp)

        try:
            n = int(self.headers.get("Content-Length", 0))
        except ValueError:
            n = 0
        if n > 4_000_000:
            done(413, {"ok": False, "error": "요청 4MB 초과"})
            return
        try:
            req = json.loads(self.rfile.read(n) or b"{}")
        except json.JSONDecodeError:
            done(400, {"ok": False, "error": "JSON 형식 오류"})
            return
        objects = req.get("objects", [])
        if not isinstance(objects, list):
            done(400, {"ok": False, "error": "objects 배열 필요"})
            return
        if "history" in req and not isinstance(req.get("history"), list):
            done(400, {"ok": False, "error": "history 배열 필요"})
            return
        images = req.get("images", [])
        if images:
            err = check_images(images)
            if err:
                done(400, {"ok": False, "error": err})
                return
        goals, err = dispatch(req)
        if err or goals is None:
            done(200, {"ok": False, "error": err or "계획 실패"})
            return
        why = validate(goals, objects)
        if why:
            done(200, {"ok": False, "error": f"검증 실패: {why}"})
            return
        done(200, {"ok": True, "goals": goals})

    def log_message(self, fmt, *args):  # 시끄러운 기본 로그 억제
        pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=None)  # 주면 config api.port 덮어씀
    ap.add_argument("--host", default=None)  # 주면 config api.host 덮어씀
    ap.add_argument("--config", default=str(HERE / "config.json"))
    args = ap.parse_args()
    try:
        CONFIG.update(json.loads(Path(args.config).read_text()))
    except (OSError, json.JSONDecodeError) as e:
        print(f"config 읽기 실패({args.config}): {e} — template 기본값", flush=True)
        CONFIG.update({"backend": "template"})
    api = CONFIG.get("http") or {}
    if not isinstance(api, dict):
        api = {}
    try:
        port = int(api.get("port", 8000))
    except (TypeError, ValueError):
        port = 8000
    host = api.get("host") or "127.0.0.1"
    if args.port is not None:
        port = args.port
    if args.host is not None:
        host = args.host
    srv = ThreadingHTTPServer((host, port), Handler)
    print(f"ai-service {host}:{port} backend={ai_conf().get('backend')} (GET /health, POST /plan)", flush=True)
    srv.serve_forever()


if __name__ == "__main__":
    main()
