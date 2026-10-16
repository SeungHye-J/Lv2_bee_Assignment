"""OpenAI 호환 HTTP 백엔드 (표준 라이브러리만) — 외부 회사 API·로컬 ollama용.

POST {endpoint}/chat/completions {"model": ..., "messages": [{"role": "user",
"content": [{"type": "text", "text": <프롬프트>},
{"type": "image_url", "image_url": {"url": "data:image/jpeg;base64,..."}} ...]}]},
temperature=0, response_format=json_object.
키는 환경변수(OPENAI_API_KEY 등)에서 읽고 이미지에 절대 굽지 않는다.
단 localhost endpoints(ollama 등 로컬)는 키 없이 호출한다.
"""

import json
import os
import urllib.request
from urllib.parse import urlparse

from images import prepare
from prompt import build


def _is_local(endpoint):
    try:
        host = urlparse(endpoint).hostname or ""
    except ValueError:
        return False
    return host in ("localhost", "127.0.0.1", "::1")


def plan(req, endpoint="https://api.openai.com/v1", model="gpt-4o-mini", timeout_s=60):
    key = os.environ.get("OPENAI_API_KEY", "")
    if not key and not _is_local(endpoint):
        return None, "OPENAI_API_KEY 없음"
    content = [{"type": "text", "text": build(req)}]
    for im in prepare(req):  # 원본·마킹 최대 4장
        content.append({"type": "image_url",
                        "image_url": {"url": "data:image/jpeg;base64," + im["data_b64"]}})
    body = json.dumps({
        "model": model,
        "temperature": 0,
        "response_format": {"type": "json_object"},
        "messages": [{"role": "user", "content": content}],
    }).encode()
    headers = {"Content-Type": "application/json"}
    if key:
        headers["Authorization"] = "Bearer " + key
    r = urllib.request.Request(endpoint.rstrip("/") + "/chat/completions", data=body,
                               headers=headers)
    try:
        with urllib.request.urlopen(r, timeout=timeout_s) as res:
            out = json.loads(res.read())
    except Exception as e:  # noqa: BLE001 — 네트워크 실패는 문자열로 전달
        return None, f"http 실패: {e}"[:200]
    try:
        text = out["choices"][0]["message"]["content"]
        goals = json.loads(text)
        goals = goals.get("goals", goals) if isinstance(goals, dict) else goals
    except (KeyError, json.JSONDecodeError, IndexError):
        return None, "응답 goals 파싱 실패"
    if not isinstance(goals, list) or not goals:
        return None, "응답에 goals 없음"
    return goals, None
