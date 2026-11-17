"""Log memory · CPU · GPU of the Jetson and of every ROS process once a second (CSV), until Ctrl-C / SIGTERM.

    python3 sys_log.py real_logs/run_1                 # standalone (e.g. a whole driving session)
    python3 sys_log.py OUT --interval 0.5 --tegrastats # + raw tegrastats log (power, per-core, temps)

approach_loop.py starts it into its run folder by itself (--real).

OUT/system.csv: t_s, mem_avail_mb, mem_used_mb, swap_mb, shm_mb (/dev/shm = Fast DDS shared memory), cpu_pct (all cores,
  100 = every core busy), gpu_pct (GR3D load), gpu_mhz, temp_max_c
OUT/procs.csv: t_s, name, pid, cpu_pct (100 = one core), rss_mb - ROS processes only (/opt/ros, a workspace install/,
  isaac_ros_ws, or a ros2 CLI), named by __node:= or the script / executable name. GPU memory is in RSS on the Jetson
  (unified memory), there is no per-process GPU load.
"""
import argparse
import csv
import os
import signal
import subprocess
import time

GPU_LOAD = '/sys/devices/platform/bus@0/17000000.gpu/load'  # per mille
GPU_FREQ = '/sys/devices/platform/bus@0/17000000.gpu/devfreq/17000000.gpu/cur_freq'
ROS_HINTS = ('/opt/ros/', '/install/', 'isaac_ros_ws', 'bin/ros2')
TICK = os.sysconf('SC_CLK_TCK')
PAGE_MB = os.sysconf('SC_PAGE_SIZE') / 1048576


def read(path, default=None):
    try:
        with open(path) as f:
            return f.read()
    except OSError:
        return default


def meminfo():
    m = {}
    for line in read('/proc/meminfo', '').splitlines():
        k, v = line.split(':', 1)
        m[k] = int(v.split()[0]) / 1024
    return m


def cpu_total():
    v = [int(x) for x in read('/proc/stat').splitlines()[0].split()[1:]]
    return sum(v), v[3] + v[4]  # all, idle + iowait


def shm_mb():
    total = 0
    for e in os.scandir('/dev/shm'):
        try:
            total += e.stat(follow_symlinks=False).st_blocks * 512
        except OSError:
            pass
    return total / 1048576


def temp_max():
    best = None
    for z in os.scandir('/sys/class/thermal'):
        t = read(os.path.join(z.path, 'temp'))
        if z.name.startswith('thermal_zone') and t and t.strip().lstrip('-').isdigit():
            best = max(best or -1e9, int(t) / 1000)
    return round(best, 1) if best is not None else ''


def ros_procs():
    """{pid: (name, utime + stime ticks, rss_mb)} of the ROS processes."""
    out = {}
    for pid in filter(str.isdigit, os.listdir('/proc')):
        args = read(f'/proc/{pid}/cmdline', '').split('\0')
        cmd = ' '.join(args)
        if not any(h in cmd for h in ROS_HINTS) or 'sys_log.py' in cmd:
            continue
        stat = read(f'/proc/{pid}/stat')
        if not stat:
            continue
        f = stat[stat.rindex(')') + 2:].split()  # fields after "(comm)"
        name = next((a.split(':=', 1)[1] for a in args if a.startswith('__node:=')), None)
        if not name:  # python3 <script> / ros2 launch ... / executable
            exe = next((a for a in args[1:] if a and not a.startswith('-')), '') if os.path.basename(args[0]).startswith('python') else args[0]
            name = os.path.basename(exe) or os.path.basename(args[0])
            if name == 'ros2':
                name = 'ros2 ' + ' '.join(args[2:4])
        out[int(pid)] = (name, int(f[11]) + int(f[12]), int(f[21]) * PAGE_MB)
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('out', help='folder for system.csv · procs.csv')
    ap.add_argument('--interval', type=float, default=1.0, help='[s]')
    ap.add_argument('--tegrastats', action='store_true', help='also write tegrastats.log (raw)')
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    stop = []
    signal.signal(signal.SIGTERM, lambda *_: stop.append(1))
    signal.signal(signal.SIGINT, lambda *_: stop.append(1))
    teg = subprocess.Popen(['tegrastats', '--interval', str(int(a.interval * 1000)), '--logfile',
                            os.path.join(a.out, 'tegrastats.log')], stdout=subprocess.DEVNULL,
                           stderr=subprocess.DEVNULL) if a.tegrastats else None
    with open(os.path.join(a.out, 'system.csv'), 'w', newline='') as fs, \
            open(os.path.join(a.out, 'procs.csv'), 'w', newline='') as fp:
        ws, wp = csv.writer(fs), csv.writer(fp)
        ws.writerow(['t_s', 'mem_avail_mb', 'mem_used_mb', 'swap_mb', 'shm_mb', 'cpu_pct', 'gpu_pct', 'gpu_mhz', 'temp_max_c'])
        wp.writerow(['t_s', 'name', 'pid', 'cpu_pct', 'rss_mb'])
        t0, prev_cpu, prev_p, prev_t = time.time(), cpu_total(), ros_procs(), time.time()
        while not stop:
            time.sleep(a.interval)
            now, cpu, procs = time.time(), cpu_total(), ros_procs()
            dt, t = now - prev_t, round(now - t0, 1)
            m = meminfo()
            busy = 100.0 * (1 - (cpu[1] - prev_cpu[1]) / max(cpu[0] - prev_cpu[0], 1))
            gl, gf = read(GPU_LOAD), read(GPU_FREQ)
            ws.writerow([t, round(m['MemAvailable']), round(m['MemTotal'] - m['MemAvailable']),
                         round(m['SwapTotal'] - m['SwapFree']), round(shm_mb()), round(busy, 1),
                         round(int(gl) / 10, 1) if gl else '', round(int(gf) / 1e6) if gf else '', temp_max()])
            for pid, (name, ticks, rss) in sorted(procs.items(), key=lambda kv: -kv[1][2]):
                was = prev_p.get(pid)
                cpu_p = round(100.0 * (ticks - was[1]) / TICK / dt, 1) if was else ''
                wp.writerow([t, name, pid, cpu_p, round(rss)])
            fs.flush()
            fp.flush()
            prev_cpu, prev_p, prev_t = cpu, procs, now
    if teg:
        teg.terminate()


if __name__ == '__main__':
    main()
