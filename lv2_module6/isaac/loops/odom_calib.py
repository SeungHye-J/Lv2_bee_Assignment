#!/usr/bin/env python3
# odom_calib.py — 바퀴 보정용: /odom 누적 거리·yaw(unwrap) vs /imu 자이로 적분 yaw
#   python3 odom_calib.py            # 1단계 직진 — 시행 끝나면 줄자 거리[m] 입력 + Enter → 새 wheel_radius
#   python3 odom_calib.py --yaw      # 2단계 회전 — 시행 끝나면 테이프 각도[deg] 입력 + Enter → 새 wheel_separation_multiplier
#   움직인 시행은 Enter(숫자 없어도)·Ctrl+C 때 항상 저장 — 실측값은 빈칸으로 두고 나중에 채워도 됨
#   안 움직였으면 Enter = 영점만 다시 (출발 직전에 한 번) · Ctrl+C = 끝 + 요약 (Ctrl+Z는 일시정지라 쓰지 말 것)
#   결과는 real_logs/odom_calib_<시각>.csv 에도 남음
#   멈춰 있는 동안(바퀴 속도 0)의 자이로 값으로 영점 편향을 재서 imu yaw에서 뺀 값(imu*)도 같이 보여 줌
import argparse, csv, math, os, sys, threading, time
import rclpy
import rclpy.executors
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from rcl_interfaces.srv import GetParameters

R0, KB0 = 0.03112, 1.056   # 기본값 — 시작할 때 /diff_drive_controller에서 실제 값을 읽어 덮어씀 (못 읽으면 이 값)
STILL = 1e-3            # 바퀴 odom |v|[m/s]·|w|[rad/s] 이 아래면 정지로 봄
LOGDIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'real_logs')


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class C(Node):
    def __init__(s, a):
        super().__init__('odom_calib')
        s.a, s.runs, s.lock = a, [], threading.Lock()
        s.still = False
        s.b_sum = s.b_t = 0.0   # 정지 중 자이로 적분 [rad]·시간 [s] — 편향 = b_sum / b_t (영점 다시 잡아도 유지)
        os.makedirs(LOGDIR, exist_ok=True)
        s.path = os.path.join(LOGDIR, time.strftime('odom_calib_%m%d_%H%M%S.csv'))
        s.f = open(s.path, 'w', newline='')
        s.w = csv.writer(s.f)
        s.w.writerow(['run', 'mode', 'true', 'odom_dist_m', 'odom_yaw_deg', 'imu_yaw_deg', 'imu_yaw_debiased_deg',
                      'duration_s', 'gyro_bias_dps', 'r0', 'kb0'])
        s.reset()
        s.load_params()
        s.create_subscription(Odometry, '/odom', s.odom, 50)
        s.create_subscription(Imu, '/imu', s.imu, 200)
        s.create_timer(0.5, s.show)

    def load_params(s):
        global R0, KB0
        cli = s.create_client(GetParameters, '/diff_drive_controller/get_parameters')
        if cli.wait_for_service(timeout_sec=3.0):
            f = cli.call_async(GetParameters.Request(names=['wheel_radius', 'wheel_separation_multiplier']))
            rclpy.spin_until_future_complete(s, f, timeout_sec=3.0)
            v = f.result().values if f.done() and f.result() else []
            if len(v) == 2 and v[0].double_value > 0:
                R0, KB0 = v[0].double_value, v[1].double_value
                print(f'컨트롤러 값: wheel_radius {R0} · wheel_separation_multiplier {KB0}', flush=True)
                return
        print(f'WARN: /diff_drive_controller 파라미터 못 읽음 — 기본값 r {R0} · k {KB0}로 계산', flush=True)

    def reset(s):
        with s.lock:
            s.p0 = s.y_prev = None
            s.d = s.y_odom = s.y_imu = 0.0
            s.t_imu = s.t0 = None

    def bias(s):
        return s.b_sum / s.b_t if s.b_t > 5.0 else 0.0   # 5 s 이상 정지 데이터가 있어야 씀

    def odom(s, m):
        p, y = m.pose.pose.position, yaw(m.pose.pose.orientation)
        tw = m.twist.twist
        with s.lock:
            s.still = abs(tw.linear.x) < STILL and abs(tw.angular.z) < STILL
            if s.p0 is None:
                s.p0, s.y_prev = (p.x, p.y), y
            s.y_odom += math.atan2(math.sin(y - s.y_prev), math.cos(y - s.y_prev))
            s.y_prev = y
            s.d = math.hypot(p.x - s.p0[0], p.y - s.p0[1])

    def imu(s, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        with s.lock:
            if s.p0 is None:          # odom 첫 메시지 전엔 적분 안 함 (둘 영점 맞춤)
                s.t_imu = t
                return
            if s.t0 is None:
                s.t0 = t
            if s.t_imu is not None:
                dt = t - s.t_imu
                if 0.0 < dt < 0.1:
                    s.y_imu += m.angular_velocity.z * dt
                    if s.still:
                        s.b_sum += m.angular_velocity.z * dt
                        s.b_t += dt
            s.t_imu = t

    def debiased(s):
        dur = (s.t_imu - s.t0) if s.t0 is not None and s.t_imu is not None else 0.0
        return s.y_imu - s.bias() * dur, dur

    def show(s):
        with s.lock:
            if s.p0 is None:
                s.wait_n = getattr(s, 'wait_n', 0) + 1
                if s.wait_n % 6 == 0:   # 3 s마다 — 아무것도 안 찍히고 멈춘 것처럼 보이지 않게
                    print(f'/odom 안 들어옴 ({s.wait_n // 2} s) — 스택 떠 있는지 · ROS_DOMAIN_ID=88 확인, 안 되면 Ctrl+C 후 다시',
                          flush=True)
                return
            s.wait_n = 0
            yi_db, _ = s.debiased()
            print(f'odom dist {s.d:.4f} m | odom yaw {math.degrees(s.y_odom):8.2f} deg'
                  f' | imu yaw {math.degrees(s.y_imu):8.2f} | imu* {math.degrees(yi_db):8.2f} deg'
                  f' | 자이로 편향 {math.degrees(s.bias()):+.4f} deg/s', flush=True)

    def mark(s, true):
        with s.lock:
            moved = s.p0 is not None and (s.d > 0.05 or abs(math.degrees(s.y_odom)) > 5)
            if moved:
                yi_db, dur = s.debiased()
                r = (len(s.runs) + 1, s.d, math.degrees(s.y_odom), math.degrees(s.y_imu), math.degrees(yi_db), true)
                s.runs.append(r)
                s.w.writerow([r[0], 'yaw' if s.a.yaw else 'dist', '' if true is None else true, f'{s.d:.5f}', f'{r[2]:.3f}', f'{r[3]:.3f}',
                              f'{r[4]:.3f}', f'{dur:.2f}', f'{math.degrees(s.bias()):.5f}', R0, KB0])
                s.f.flush()
                print(f'--- 시행 {r[0]} 저장: 실측 {"(없음)" if true is None else true} | odom dist {s.d:.4f} m, odom yaw {r[2]:.2f}, '
                      f'imu* {r[4]:.2f}', flush=True)
            print('--- 영점 다시 잡음', flush=True)
        s.reset()

    def summary(s):
        print(f'\n자이로 편향 {math.degrees(s.bias()):+.4f} deg/s (정지 {s.b_t:.0f} s 기준) · 기록 {s.path}')
        if not s.runs:
            return
        print('시행 | 실측 | odom dist | odom yaw | imu yaw | imu*')
        for i, d, yo, yi, yd, tr in s.runs:
            print(f'{i:4d} | {"-" if tr is None else f"{tr:.3f}":>6} | {d:9.4f} | {yo:8.2f} | {yi:7.2f} | {yd:7.2f}')
        if s.a.yaw:
            ks = [KB0 * abs(yo) / abs(tr) for _, _, yo, _, _, tr in s.runs if tr]
            if not ks:
                print('테이프 각도가 0 아닌 시행 없음 — 제안 없음')
                return
            print(f'wheel_separation_multiplier 제안: {sum(ks) / len(ks):.4f}  (시행별 {", ".join(f"{k:.4f}" for k in ks)})')
            es = [100 * (abs(yd) - abs(tr)) / abs(tr) for _, _, _, _, yd, tr in s.runs if tr]
            print(f'IMU*(편향 뺌) vs 테이프 오차 %: {", ".join(f"{e:+.2f}" for e in es)}')
        else:
            rs = [R0 * tr / d for _, d, _, _, _, tr in s.runs if tr and d > 0.05]
            if not rs:
                print('실측 거리 넣은 시행 없음 — 제안 없음 (CSV true 열을 채우거나 값을 알려 주기)')
                return
            print(f'wheel_radius 제안: {sum(rs) / len(rs):.5f}  (시행별 {", ".join(f"{r:.5f}" for r in rs)})')
            for i, d, _, _, yd, _ in s.runs:   # 직진 중 IMU가 돈 만큼 = 실제로 휜 각도 (odom은 못 봄)
                print(f'  시행 {i}: 휜 각도 imu* {yd:+.2f} deg ({"왼쪽" if yd > 0 else "오른쪽"})'
                      f' → 끝점 옆 오차 약 {abs(d * math.sin(math.radians(yd)) / 2) * 100:.1f} cm')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--yaw', action='store_true', help='2단계 회전 (입력값 = 테이프 각도 deg). 없으면 1단계 직진 (입력값 = 줄자 m)')
    rclpy.init()
    n = C(ap.parse_args())
    unit = 'deg' if n.a.yaw else 'm'
    print(f'시행 끝나면 실측값[{unit}] 입력 + Enter (몰라도 그냥 Enter — 저장됨) · Ctrl+C = 끝', flush=True)

    def keys():
        while True:
            try:
                line = input().strip()
            except (EOFError, ValueError):
                return
            if not line:
                n.mark(None)
                continue
            try:
                v = float(line)
                if not n.a.yaw and v > 10:   # 직진 10 m 넘을 일 없음 → cm로 봄
                    print(f'{v} → cm로 보고 {v / 100} m', flush=True)
                    v /= 100
                n.mark(v)
            except ValueError:
                print(f'숫자가 아님: {line!r} — 다시 입력', flush=True)
    threading.Thread(target=keys, daemon=True).start()
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    n.mark(None)   # 저장 안 한 움직임이 남아 있으면 저장
    n.summary()
    n.f.close()
    # 정리 순서 지켜야 종료 때 Segmentation fault 안 남 (rclpy 소멸자 vs 인터프리터 종료)
    n.destroy_node()
    rclpy.try_shutdown()
    sys.stdout.flush()   # os._exit는 버퍼를 안 비움 (| tee 일 때 요약이 사라짐)
    os._exit(0)


if __name__ == '__main__':
    main()
