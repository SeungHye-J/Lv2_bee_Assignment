"""Claude CLI 백엔드 — `claude -p "<프롬프트>" --output-format json` 호출.

이미지는 임시 JPEG 파일로 내려 `--attach` ... 형태가 아니라,
CLI가 stdin 텍스트만 받는 구성이면 프롬프트에 이미지 생략을 명시한다.
지금은 골격: CLI 없으면 에러 반환 (컨테이너는 템플릿 폴백).
"""

import json
import shutil
import subprocess

from prompt import build


def plan(req, model="default", timeout_s=60):
    if shutil.which("claude") is None:
        return None, "claude CLI 없음"
    prompt = build(req)
    try:
        p = subprocess.run(["claude", "-p", prompt, "--output-format", "json"],
                           capture_output=True, text=True, timeout=timeout_s)
    except subprocess.TimeoutExpired:
        return None, "claude timeout"
    if p.returncode != 0:
        return None, f"claude exit={p.returncode}: {p.stderr[:200]}"
    try:
        out = json.loads(p.stdout)
    except json.JSONDecodeError:
        return None, "claude 출력 JSON 파싱 실패"
    goals = out.get("goals", out if isinstance(out, list) else None)
    if not goals:
        return None, "claude 응답에 goals 없음"
    return goals, None
