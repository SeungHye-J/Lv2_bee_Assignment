"""BEE 집기 시연 환경 (standalone) — scene_sim.usda를 열지 않고 stage를 직접 구성: 바닥 · 시작 상자 1개 · 그 위 퍽 2개 · 로봇.

  isaac/scripts/demo/pick_load_demo.sh   (이 스크립트 + ROS까지 — isaac_sim.sh 경유)
  장면은 bee_isaac.scenes.manipulator_scene · 배치는 bee_isaac.constants
  에셋(beebot · shelf · puck)은 isaac/usda 파일을 참조로 씀 — 로봇 안 ActionGraph(/clock · 카메라)와 opencr_bridge.py(가상 OpenCR)는 그대로
"""
import argparse
import os

from bee_isaac.scenes.manipulator_layout import layout  # 표준 라이브러리만 — Isaac 전에 import 가능

parser = argparse.ArgumentParser()
parser.add_argument("--headless", action="store_true")
parser.add_argument("--no-play", action="store_true", help="Play를 누르지 않음")
parser.add_argument("--puck", action="append", metavar="색:모양:상태:태그",
                    help="퍽 하나, 2번 반복: red|green : box|cyl : stand|lie : 0~11. 뒤쪽·빈 항목은 --seed로 정함 (예: red:cyl:stand:3, green::lie)")
parser.add_argument("--seed", type=int, help="--puck에서 비운 항목과 퍽 방향을 정함. 없으면 방향은 scene_sim 값")
args = parser.parse_args()
try:
    pucks = layout(args.puck, args.seed)
except ValueError as e:
    parser.error(str(e))  # Isaac을 띄우기 전에 끝냄

from isaacsim import SimulationApp  # noqa: E402 — 다른 omni import보다 먼저

app = SimulationApp({"headless": args.headless, "width": 1280, "height": 720})

import omni.kit.app  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.utils.extensions import enable_extension  # noqa: E402

for ext in ("isaacsim.ros2.bridge", "isaacsim.sensors.experimental.physics", "isaacsim.core.nodes"):
    enable_extension(ext)
app.update()

from bee_isaac.constants import BOX_X  # noqa: E402
from bee_isaac.scenes.manipulator_scene import VIEW_CAMERA, build  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
RESET_FILE = "/tmp/beebot_isaac_reset"  # isaac_sim.sh와 같은 경로

omni.usd.get_context().new_stage()
for _ in range(10):
    app.update()
stage = omni.usd.get_context().get_stage()
build(stage, pucks)
for _ in range(60):  # 페이로드 · 그래프가 올라올 때까지
    app.update()
if not args.headless:
    from omni.kit.viewport.utility import get_active_viewport  # noqa: E402

    get_active_viewport().camera_path = VIEW_CAMERA

bridge = os.path.join(HERE, "opencr_bridge.py")  # pty를 열고, 첫 물리 스텝에서 로봇에 붙어 /tmp/opencr를 만듦
exec(compile(open(bridge).read(), bridge, "exec"), {"__name__": "__main__", "__file__": bridge})

timeline = omni.timeline.get_timeline_interface()
if not args.no_play:
    timeline.play()
print(f"[manipulator_env] 준비 — 상자 가운데 x {BOX_X:.4f} · 퍽 {[(p.name, p.posture, p.pos[:2], p.rot[2]) for p in pucks]}", flush=True)
while app.is_running():
    app.update()
    if os.path.exists(RESET_FILE):  # reset.sh: Stop(처음 자세로) → Play — ROS 재시작은 isaac_sim.sh
        timeline.stop()
        for _ in range(30):
            app.update()
        timeline.play()
        os.remove(RESET_FILE)
app.close()
