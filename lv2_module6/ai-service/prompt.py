"""VLM 프롬프트 조립 — 명령 + 물체 + 이미지 수 + 재계획 사유 + 대화 기록 → 텍스트 한 덩이.

컨테이너가 보낸 JSON을 사람·모델이 읽는 지시문으로 바꾼다.
서버는 무상태: history는 요청에 실려 오고 저장하지 않는다 (최근 20개·개당 500자까지만).
출력 스키마는 coordinator 검증(03-plan.md:67)과 동일하게 고정한다.
"""

ALLOWED = "navigate · pick · place · say"
HISTORY_N = 20
HISTORY_CHARS = 500


def build(req):
    lines = []
    lines.append(f"명령: {req.get('command', '')}")
    objs = req.get("objects", [])
    lines.append(f"보이는 물체({len(objs)}개, 좌표=map 기준 m):")
    for o in objs:
        if isinstance(o, dict):
            pos = o.get("position", [])
            size = o.get("size", [])
            extra = ""
            if len(pos) == 3:
                extra += f" pos=({pos[0]:.2f},{pos[1]:.2f},{pos[2]:.2f})"
            if len(size) == 3:
                extra += f" size=({size[0]:.2f},{size[1]:.2f},{size[2]:.2f})"
            if isinstance(o.get("confidence"), (int, float)):
                extra += f" conf={o['confidence']:.2f}"
            px = o.get("px", {})
            if isinstance(px, dict):
                for cam in ("pole_cam", "gripper_cam"):
                    if cam in px and isinstance(px[cam], (list, tuple)) and len(px[cam]) == 2:
                        extra += f" {cam.split('_')[0]}px=({px[cam][0]},{px[cam][1]})"
            lines.append(f"  - id={o.get('id')} class={o.get('class_name')}{extra}")
    n_img = len(req.get("images", []))
    lines.append(f"첨부 이미지: {n_img}장 — 카메라별 원본·마킹(번호=id) 순으로 전달 "
                 "(예: pole 원본·pole 마킹·gripper 원본·gripper 마킹, 마킹 없는 카메라는 원본만)")
    rp = req.get("replan") or {}
    if rp:
        lines.append(f"재계획: plan={rp.get('plan_id')} step={rp.get('failed_step')} 사유={rp.get('reason')}")
        lines.append("같은 실패를 반복하지 말고 접근 방향·순서를 바꿔라.")
    hist = req.get("history", [])
    if isinstance(hist, list) and hist:
        lines.append(f"이전 대화(최근 {HISTORY_N}개까지):")
        for h in hist[-HISTORY_N:]:
            if isinstance(h, dict):
                text = str(h.get("text", ""))[:HISTORY_CHARS]
                lines.append(f"  - {h.get('role', '?')}: {text}")
    lines.append(f"허용 동작: {ALLOWED}. JSON 배열로만 답하라: "
                 '[{"step": 1, "skill": "...", "target": "<물체id|말할내용>", "relation": "front|on|", "offset_m": 0.0}]')
    return "\n".join(lines)
