"""원본 미로 메시(STL)에서 벽 상자를 뽑아 우리 코스 모델을 만든다 — 코스가 바뀔 때만 다시 실행.

  python3 tools/build_walls.py            # (course/ 에서) 원본 STL을 받아 → twin_closed_maze/{walls.json, model.sdf, model.config}

왜: 원본 model.sdf(original/)의 충돌 상자에는 세로 벽(바깥 테두리·안쪽 4개)이 빠져 있어서 그대로 쓰면 로봇이 벽을 통과한다.
    메시를 위에서 본 모양으로 기둥 사이에 벽판이 있는지 판정해 기둥 + 벽판 상자로 다시 만든다.
STL(29MB)은 저장소에 넣지 않는다 — 여기서만 쓰고 .cache/에 둔다 (gitignore).
출처: SeongminJaden/waffle-lyrical-slam @ 7fc1ef4 (Apache-2.0) — original/NOTICE · source.json
필요: pip install numpy opencv-python
"""
import json
import struct
import urllib.request
import xml.etree.ElementTree as ET
from pathlib import Path

import cv2
import numpy as np

COURSE = Path(__file__).resolve().parents[1] / 'twin_closed_maze'
STL_URL = ('https://raw.githubusercontent.com/SeongminJaden/waffle-lyrical-slam/7fc1ef486f3fc2513b06ca9bc76cd00f4df4fb03/'
           'src/waffle_slam_course/models/twin_closed_maze/meshes/maze.stl')
CACHE = Path(__file__).resolve().parent / '.cache' / 'maze.stl'
PX_PER_M = 200
POST = 0.06
PANEL = 0.02   # 원본 판넬 두께는 3mm — 렌더·충돌에서 놓치지 않게 2cm로


def stl_triangles_xy():
    if not CACHE.exists():
        CACHE.parent.mkdir(parents=True, exist_ok=True)
        print(f'STL 받는 중 ({STL_URL})')
        with urllib.request.urlopen(STL_URL, timeout=60) as r:
            CACHE.write_bytes(r.read())
    data = CACHE.read_bytes()
    count = struct.unpack('<I', data[80:84])[0]
    tris = np.frombuffer(data, np.dtype([('n', '<f4', (3,)), ('v', '<f4', (3, 3)), ('a', '<u2')]), count, 84)['v']
    return tris[:, :, :2]


def main():
    source = json.loads((COURSE / 'original' / 'source.json').read_text())
    sdf = ET.parse(COURSE / 'original' / 'model.sdf')
    posts = np.array([[float(v) for v in c.findtext('pose').split()[:2]]
                      for c in sdf.iter('collision') if c.get('name', '').startswith('post_')])
    w, h, wall_h = source['size_m']
    half = (w / 2 + 0.1, h / 2 + 0.1)
    shape = (int(2 * half[1] * PX_PER_M), int(2 * half[0] * PX_PER_M))

    def to_px(xy):
        return np.stack([(xy[..., 0] + half[0]) * PX_PER_M, (half[1] - xy[..., 1]) * PX_PER_M], -1).astype(np.int32)

    mask = np.zeros(shape, np.uint8)
    cv2.fillPoly(mask, list(to_px(stl_triangles_xy())), 255)
    mask = cv2.dilate(mask, np.ones((3, 3), np.uint8))

    boxes = [{'name': f'post_{i}', 'x': round(x, 4), 'y': round(y, 4), 'sx': POST, 'sy': POST}
             for i, (x, y) in enumerate(posts)]
    for i, (x, y) in enumerate(posts):
        for axis in (0, 1):  # +x · +y 방향으로 가장 가까운 기둥과의 사이
            same = (np.abs(posts[:, 1 - axis] - (y, x)[axis]) < 1e-3) & (posts[:, axis] > (x, y)[axis] + 1e-3)
            if not same.any():
                continue
            j = np.flatnonzero(same)[np.argmin(posts[same, axis])]
            a, b = posts[i], posts[j]
            uv = to_px(a + np.linspace(0.15, 0.85, 15)[:, None] * (b - a))
            if (mask[uv[:, 1], uv[:, 0]] > 0).mean() > 0.8:
                c, length = (a + b) / 2, abs((b - a)[axis])
                boxes.append({'name': f'panel_{len(boxes)}', 'x': round(c[0], 4), 'y': round(c[1], 4),
                              'sx': round(length if axis == 0 else PANEL, 4), 'sy': round(PANEL if axis == 0 else length, 4)})

    walls = {'name': 'twin_closed_maze', 'size_m': [w, h], 'wall_height_m': wall_h, 'spawn_xy_m': source['spawn_xy_m'],
             'source': 'SeongminJaden/waffle-lyrical-slam@7fc1ef4 maze.stl (Apache-2.0)', 'boxes': boxes}
    (COURSE / 'walls.json').write_text(json.dumps(walls, indent=1))

    items = []
    for b in boxes:
        geom = f"<geometry><box><size>{b['sx']} {b['sy']} {wall_h}</size></box></geometry>"
        items.append(f"<collision name=\"{b['name']}\"><pose>{b['x']} {b['y']} {wall_h / 2} 0 0 0</pose>{geom}</collision>"
                     f"<visual name=\"{b['name']}_v\"><pose>{b['x']} {b['y']} {wall_h / 2} 0 0 0</pose>{geom}"
                     "<material><ambient>0.65 0.72 0.8 1</ambient><diffuse>0.65 0.72 0.8 1</diffuse></material></visual>")
    (COURSE / 'model.sdf').write_text(
        "<?xml version='1.0'?>\n<!-- tools/build_walls.py로 생성 — 손으로 고치지 말 것. 원본: original/ -->\n"
        "<sdf version=\"1.8\"><model name=\"twin_closed_maze\"><static>true</static><link name=\"maze\">\n"
        + '\n'.join(items) + "\n</link></model></sdf>\n")
    (COURSE / 'model.config').write_text(
        '<?xml version="1.0"?>\n<model><name>Twin Closed Maze (beebot)</name><version>1.0</version>\n'
        '<sdf version="1.8">model.sdf</sdf>\n<description>Twin Closed Maze B — 기둥 + 벽판 상자 (세로 벽 포함). '
        '원본: SeongminJaden/waffle-lyrical-slam (Apache-2.0)</description></model>\n')
    print(f'기둥 {len(posts)} · 벽판 {len(boxes) - len(posts)} → {COURSE}')


if __name__ == '__main__':
    main()
