"""수동 조작 창 — 시뮬레이터 [수동 조작] 버튼으로 연다. 여는 순간 수동 모드(/manual/enable true), 닫으면 바퀴 정지 + 자동 모드로.

  [바퀴 조작판]                │ [로봇 3D (지금 상태)]          │ [팔 3D — 관절 마커를 잡아 위아래로 끌기]
  앞 · 뒤 · 멈춤                │  드래그 = 회전 · 휠 = 줌         │  빈 곳 드래그 = 회전 · 휠 = 줌
  좌측 바퀴 앞/뒤 · 우측 바퀴 앞/뒤 │                                │  놓으면 /manual/arm 으로 보냄
  전체 속도 ▲ · ▼               │                                │  [지금 자세로] · 관절 목표/실제 값
  비상정지 · 상태               │                                │

바퀴: 좌·우 바퀴 방향(-1·0·+1) × 전체 속도[m/s] → 차동 구동 (v, ω)로 바꿔 /manual/cmd_vel (0.2초마다, 멈추면 0 한 번)
      → control/manual_control → twist_mux(자동보다 우선) → 안전 체인 → diff_drive_controller → 바퀴
팔:   목표(밝게) · 지금(어둡게)을 겹쳐 그린다. 관절 한계(URDF)로 자른다.
지금 상태: 플랜트가 돌면(mock_hardware:=false) 플랜트 값, 아니면 /joint_states.
"""
import math
import tkinter as tk

import cv2
import numpy as np

import robot_view
from bridge import WHEEL_SEPARATION, read_msg, send_arm, send_bool, send_twist

VIEW = (420, 360)
SPEED_STEP, SPEED_MAX, SPEED_MIN = 0.02, 0.26, 0.02
ARM_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'gripper_left_joint']


def to_photo(img):
    import base64
    ok, buf = cv2.imencode('.png', img)
    return tk.PhotoImage(data=base64.b64encode(buf.tobytes())) if ok else None


class ManualWindow(tk.Toplevel):
    def __init__(self, master, sim, on_close=None):
        super().__init__(master)
        self.sim, self.on_close = sim, on_close
        self.title('수동 조작 — beebot')
        self.protocol('WM_DELETE_WINDOW', self.close)
        self.model = None
        self.wheel = [0, 0]           # 좌·우 바퀴 방향
        self.speed = 0.10             # 전체 바퀴 속도 [m/s]
        self.sent_zero = True
        self.target = {}              # 팔 목표 (관절 → 값)
        self.selected = None
        self.drag_from = None
        self.robot_cam = robot_view.OrbitView(target=(0, 0, 0.2), dist=1.3)
        self.arm_cam = robot_view.OrbitView(target=(0.05, 0, 0.35), dist=0.75, yaw=-2.0, pitch=0.35)
        send_bool('/manual/enable', True)

        # --- 바퀴 조작판
        left = tk.Frame(self)
        left.pack(side=tk.LEFT, padx=8, pady=8, anchor=tk.N)
        tk.Label(left, text='바퀴', font=('', 13, 'bold')).pack(anchor=tk.W)
        pad = tk.Frame(left)
        pad.pack(pady=4)
        buttons = [
            ('좌측 바퀴 앞', (1, None), 0, 0), ('앞', (1, 1), 0, 1), ('우측 바퀴 앞', (None, 1), 0, 2),
            ('', None, 1, 0), ('멈춤', (0, 0), 1, 1), ('', None, 1, 2),
            ('좌측 바퀴 뒤', (-1, None), 2, 0), ('뒤', (-1, -1), 2, 1), ('우측 바퀴 뒤', (None, -1), 2, 2)]
        for text, d, r, c in buttons:
            if d is None:
                continue
            tk.Button(pad, text=text, width=11, height=2, command=lambda d=d: self.set_wheels(*d)).grid(row=r, column=c, padx=2, pady=2)
        sp = tk.Frame(left)
        sp.pack(pady=4)
        tk.Button(sp, text='전체 속도 ▼', command=lambda: self.change_speed(-SPEED_STEP)).pack(side=tk.LEFT)
        self.speed_label = tk.Label(sp, width=10)
        self.speed_label.pack(side=tk.LEFT)
        tk.Button(sp, text='전체 속도 ▲', command=lambda: self.change_speed(SPEED_STEP)).pack(side=tk.LEFT)
        self.estop = tk.BooleanVar(value=False)
        tk.Checkbutton(left, text='비상정지 (/e_stop)', variable=self.estop, fg='#c62828',
                       command=lambda: send_bool('/e_stop', self.estop.get())).pack(anchor=tk.W, pady=4)
        self.info = tk.Label(left, justify=tk.LEFT, font=('Menlo', 10), anchor=tk.W)
        self.info.pack(anchor=tk.W, pady=6)
        tk.Label(left, text='창을 닫으면 바퀴 정지 + 자동 모드', fg='#666').pack(anchor=tk.W)

        # --- 로봇 3D
        mid = tk.Frame(self)
        mid.pack(side=tk.LEFT, padx=4, pady=8, anchor=tk.N)
        tk.Label(mid, text='로봇 (지금 상태) — 드래그 회전 · 휠 줌').pack(anchor=tk.W)
        self.robot_label = tk.Label(mid, bg='black')
        self.robot_label.pack()
        self.robot_label.bind('<ButtonPress-1>', lambda e: setattr(self, 'robot_drag', (e.x, e.y)))
        self.robot_label.bind('<B1-Motion>', self.on_robot_drag)
        self.robot_label.bind('<MouseWheel>', lambda e: self.robot_cam.zoom(1 if e.delta > 0 else -1))
        self.robot_label.bind('<Button-4>', lambda e: self.robot_cam.zoom(1))
        self.robot_label.bind('<Button-5>', lambda e: self.robot_cam.zoom(-1))

        # --- 팔 3D
        right = tk.Frame(self)
        right.pack(side=tk.LEFT, padx=4, pady=8, anchor=tk.N)
        tk.Label(right, text='매니퓰레이터 — 노란 관절 마커를 잡아 위아래로 끌기 · 빈 곳 드래그 회전').pack(anchor=tk.W)
        self.arm_label = tk.Label(right, bg='black', cursor='hand2')
        self.arm_label.pack()
        self.arm_label.bind('<ButtonPress-1>', self.on_arm_press)
        self.arm_label.bind('<B1-Motion>', self.on_arm_drag)
        self.arm_label.bind('<ButtonRelease-1>', self.on_arm_release)
        self.arm_label.bind('<MouseWheel>', lambda e: self.arm_cam.zoom(1 if e.delta > 0 else -1))
        self.arm_label.bind('<Button-4>', lambda e: self.arm_cam.zoom(1))
        self.arm_label.bind('<Button-5>', lambda e: self.arm_cam.zoom(-1))
        row = tk.Frame(right)
        row.pack(fill=tk.X)
        tk.Button(row, text='지금 자세로', command=self.reset_target).pack(side=tk.LEFT)
        tk.Button(row, text='그리퍼 열기', command=lambda: self.set_gripper(0.019)).pack(side=tk.LEFT)
        tk.Button(row, text='그리퍼 닫기', command=lambda: self.set_gripper(-0.010)).pack(side=tk.LEFT)
        self.joint_text = tk.Label(right, justify=tk.LEFT, font=('Menlo', 10), anchor=tk.W)
        self.joint_text.pack(anchor=tk.W)
        self.photos = []
        self.ticks = 0
        self.tick()

    # ---------- 상태
    def actual(self):
        """지금 관절값 · 바퀴 속도 [rad/s]"""
        if self.sim.plant.alive:
            q = self.sim.plant.pose()[3]
            with self.sim.plant.lock:
                wheel = list(self.sim.plant.wheel_vel)
            return q, wheel
        js, _ = read_msg('joint_states')
        if not js:
            return {}, [0.0, 0.0]
        q = {n: p for n, p in zip(js['name'], js['position']) if isinstance(p, float) and math.isfinite(p)}
        vel = dict(zip(js['name'], js['velocity'])) if js.get('velocity') else {}
        return q, [vel.get('wheel_left_joint', 0.0), vel.get('wheel_right_joint', 0.0)]

    # ---------- 바퀴
    def set_wheels(self, left, right):
        if left is not None:
            self.wheel[0] = left
        if right is not None:
            self.wheel[1] = right

    def change_speed(self, delta):
        self.speed = round(min(SPEED_MAX, max(SPEED_MIN, self.speed + delta)), 2)

    def twist(self):
        vl, vr = self.wheel[0] * self.speed, self.wheel[1] * self.speed
        return (vl + vr) / 2, (vr - vl) / WHEEL_SEPARATION

    # ---------- 팔
    def reset_target(self):
        q, _ = self.actual()
        self.target = {j: q.get(j, 0.0) for j in ARM_JOINTS}
        send_arm(self.target)

    def set_gripper(self, value):
        if not self.target:
            self.reset_target()
        self.target['gripper_left_joint'] = value
        send_arm(self.target)

    def on_arm_press(self, e):
        self.drag_from = (e.x, e.y)
        self.selected = None
        if not self.model or not self.target:
            return
        frames = self.model.joint_frames(self.target)
        best, best_d = None, 16
        for name in ARM_JOINTS:
            if name in frames:
                uv, z = self.arm_cam.project([frames[name][0]], VIEW)
                d = math.dist(uv[0], (e.x, e.y))
                if z[0] > 0 and d < best_d:
                    best, best_d = name, d
        self.selected = best
        self.drag_value = self.target.get(best, 0.0) if best else None

    def on_arm_drag(self, e):
        dx, dy = e.x - self.drag_from[0], e.y - self.drag_from[1]
        if self.selected:
            _, kind, _, lo, hi = self.model.joints[self.selected]
            scale = 0.0002 if kind == 'prismatic' else 0.01  # [m/px] · [rad/px]
            self.target[self.selected] = min(hi, max(lo, self.drag_value - dy * scale))
        else:
            self.arm_cam.orbit(dx, dy)
            self.drag_from = (e.x, e.y)

    def on_arm_release(self, _):
        if self.selected:
            send_arm(self.target)
        self.selected = None

    def on_robot_drag(self, e):
        x0, y0 = self.robot_drag
        self.robot_cam.orbit(e.x - x0, e.y - y0)
        self.robot_drag = (e.x, e.y)

    # ---------- 그리기 · 보내기
    def tick(self):
        if not self.winfo_exists():
            return
        if self.model is None:
            desc, _ = read_msg('robot_description')
            if desc:
                self.model = robot_view.RobotModel(desc['data'])
                self.arm_links = self.model.descendants('link1')
        q, wheel_vel = self.actual()
        if q and not self.target:
            self.target = {j: q.get(j, 0.0) for j in ARM_JOINTS}

        # 바퀴: 움직이는 동안 0.2초마다, 멈추면 0 한 번
        self.ticks += 1
        v, w = self.twist()
        if self.ticks % 2 == 0 and ((v, w) != (0.0, 0.0) or not self.sent_zero):
            send_twist(v, w)
            self.sent_zero = (v, w) == (0.0, 0.0)

        photos = []
        if self.model:
            img = self.robot_cam.render(self.model.faces(q), VIEW)
            photos.append(to_photo(img))
            self.robot_label.config(image=photos[-1])
            arm = self.arm_cam.render(self.model.faces(q, self.arm_links, tint=(70, 70, 80)), VIEW)  # 지금 (어둡게)
            goal = self.arm_cam.render(self.model.faces(self.target, self.arm_links), VIEW, grid=False)  # 목표
            mask = goal.any(axis=2) & (goal != (40, 40, 46)).any(axis=2)
            arm[mask] = cv2.addWeighted(arm, 0.3, goal, 0.7, 0)[mask]
            for name, (origin, _) in self.model.joint_frames(self.target).items():
                if name not in ARM_JOINTS:
                    continue
                uv, z = self.arm_cam.project([origin], VIEW)
                if z[0] > 0:
                    c = tuple(uv[0].astype(int))
                    sel = name == self.selected
                    cv2.circle(arm, c, 9 if sel else 7, (0, 80, 255) if sel else (0, 220, 255), 2, cv2.LINE_AA)
                    cv2.putText(arm, name.replace('_joint', ''), (c[0] + 9, c[1] - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.38,
                                (220, 220, 220), 1, cv2.LINE_AA)
            photos.append(to_photo(arm))
            self.arm_label.config(image=photos[-1])
        else:
            blank = np.full((VIEW[1], VIEW[0], 3), 40, np.uint8)
            cv2.putText(blank, 'waiting /robot_description (test-faker 0?)', (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                        (200, 200, 200), 1)
            photos.append(to_photo(blank))
            self.robot_label.config(image=photos[-1])
            self.arm_label.config(image=photos[-1])
        self.photos = photos

        mode, _ = read_msg('operation_mode')
        self.speed_label.config(text=f'{self.speed:.2f} m/s')
        wl, wr = self.wheel
        sign = lambda d: {1: '앞', -1: '뒤', 0: '정지'}[d]
        self.info.config(text=(
            f'모드     {mode["data"] if mode else "?"}\n'
            f'명령     좌 {sign(wl)} · 우 {sign(wr)}\n'
            f'         v {v:+.2f} m/s · ω {w:+.2f} rad/s\n'
            f'실제     바퀴 좌 {wheel_vel[0]:+.2f} · 우 {wheel_vel[1]:+.2f} rad/s\n'
            f'상태     {"플랜트(OpenCR 흉내)" if self.sim.plant.alive else "/joint_states (mock)"}'))
        self.joint_text.config(text='\n'.join(
            f'{j.replace("_joint", ""):8} 목표 {self.target.get(j, 0):+.3f}  지금 {q.get(j, 0):+.3f}' for j in ARM_JOINTS))
        self.after(100, self.tick)

    def close(self):
        self.wheel = [0, 0]
        send_twist(0.0, 0.0)
        send_bool('/manual/enable', False)
        if self.on_close:
            self.on_close()
        self.destroy()
