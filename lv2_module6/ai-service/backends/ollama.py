"""로컬 ollama 백엔드 — OpenAI 호환 API(`/v1/chat/completions`) 경유, 키 불필요.

실제는 openai_compat에 위임한다. ollama가 꺼져 있거나 모델이 없으면
그대로 에러 반환 (컨테이너는 템플릿 폴백).
기본값은 여기 DEFAULTS 한 곳에만 — server.py는 config.json 값을 그대로 넘긴다.
"""

from backends import openai_compat

DEFAULTS = {
    "model": "llama4",
    "endpoint": "http://localhost:11434/v1",
    "timeout_s": 120,
}


def plan(req, model=None, endpoint=None, timeout_s=None):
    return openai_compat.plan(req, endpoint=endpoint or DEFAULTS["endpoint"],
                              model=model or DEFAULTS["model"],
                              timeout_s=timeout_s or DEFAULTS["timeout_s"])
