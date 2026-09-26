"""Task-manager numbers from Linux /proc - standard library only.

Used on the Pi (the cockpit's System tab) AND inside the PC's Docker
containers (detect_server.py, tts_server.py report it on GET), so one file
answers "who is using the CPU" on both machines.

Percentages are of the WHOLE machine, like Windows Task Manager: 100 % means
every core busy. A single thread can therefore show at most 100 / cores
(25 % on the Pi 4).

Inside Docker Desktop the "host" figures are the Docker VM, which on Windows
shares all the PC's cores - close to the PC's own numbers, but the load from
native Windows programs (Ollama, the browser) is not in them.
"""

import os
import threading
import time

_TICK = os.sysconf("SC_CLK_TCK") if hasattr(os, "sysconf") else 100


def _read(path):
    try:
        with open(path) as f:
            return f.read()
    except OSError:
        return ""


def _cpu_times():
    """[(busy, total)] for 'cpu' (all) then each core, in clock ticks."""
    out = []
    for line in _read("/proc/stat").splitlines():
        if not line.startswith("cpu"):
            break
        v = [int(x) for x in line.split()[1:]]
        idle = v[3] + (v[4] if len(v) > 4 else 0)          # idle + iowait
        total = sum(v[:8])                                  # guest time is inside user
        out.append((total - idle, total))
    return out


def _task_ticks(path):
    """utime + stime of a /proc/<pid>[/task/<tid>] entry, in clock ticks."""
    s = _read(path + "/stat")
    if not s:
        return None
    rest = s[s.rfind(")") + 2:].split()                     # comm may contain spaces
    return int(rest[11]) + int(rest[12])


def _meminfo():
    m = {}
    for line in _read("/proc/meminfo").splitlines():
        k, _, v = line.partition(":")
        try:
            m[k] = int(v.split()[0]) // 1024                # kB -> MB
        except (ValueError, IndexError):
            pass
    return m


def _temp_c():
    t = _read("/sys/class/thermal/thermal_zone0/temp").strip()
    return round(int(t) / 1000.0, 1) if t.isdigit() else None


def _thread_names():
    """native thread id -> a readable name. Python's default names are
    'Thread-7 (_run)', which says nothing; the target's qualified name
    ('Follower._run', 'Lidar._loop') says which part of the program it is."""
    out = {}
    for t in threading.enumerate():
        tid = getattr(t, "native_id", None)
        if tid is None:
            continue
        fn = getattr(t, "_target", None)
        name = getattr(fn, "__qualname__", None) or t.name
        if t is threading.main_thread():
            name = "main"
        out[tid] = name
    return out


class Sampler:
    """Call sample() every second or two; each call reports the CPU used
    since the previous one. The first call has no previous, so CPU is 0."""

    def __init__(self):
        self._lock = threading.Lock()
        self._prev = None
        self.ncpu = os.cpu_count() or 1

    def sample(self, threads=True):
        with self._lock:
            return self._sample(threads)

    def _sample(self, threads):
        now = time.monotonic()
        cpu = _cpu_times()
        proc = _task_ticks("/proc/self")
        tasks = {}
        if threads:
            try:
                tids = os.listdir("/proc/self/task")
            except OSError:
                tids = []
            for tid in tids:
                ticks = _task_ticks("/proc/self/task/" + tid)
                if ticks is not None:
                    tasks[int(tid)] = ticks
        prev, self._prev = self._prev, (now, cpu, proc, tasks)

        mem = _meminfo()
        host = {"cores": self.ncpu, "cpu_pct": 0.0, "per_core": [],
                "mem_used_mb": mem.get("MemTotal", 0) - mem.get("MemAvailable", 0),
                "mem_total_mb": mem.get("MemTotal", 0), "temp_c": _temp_c()}
        try:
            host["load1"] = round(os.getloadavg()[0], 2)
        except (OSError, AttributeError):
            host["load1"] = None
        status = _read("/proc/self/status")
        rss = next((int(l.split()[1]) // 1024 for l in status.splitlines() if l.startswith("VmRSS:")), None)
        me = {"pid": os.getpid(), "cpu_pct": 0.0, "rss_mb": rss, "threads": len(tasks) or None}
        out = {"host": host, "proc": me, "threads": [], "ok": bool(cpu)}
        if prev is None or not cpu:
            return out

        dt = max(1e-3, now - prev[0])

        def pct(busy0, tot0, busy1, tot1):
            return round(100.0 * (busy1 - busy0) / max(1, tot1 - tot0), 1)

        if len(prev[1]) == len(cpu):
            host["cpu_pct"] = pct(*prev[1][0], *cpu[0])
            host["per_core"] = [pct(*a, *b) for a, b in zip(prev[1][1:], cpu[1:])]
        machine = _TICK * dt * self.ncpu                   # ticks the whole machine had
        if proc is not None and prev[2] is not None:
            me["cpu_pct"] = round(100.0 * (proc - prev[2]) / machine, 1)
        if threads:
            names = _thread_names()
            groups = {}
            for tid, ticks in tasks.items():
                before = prev[3].get(tid)
                if before is None:
                    continue
                name = names.get(tid) or ("native: " + (_read("/proc/self/task/%d/comm" % tid).strip() or "?"))
                g = groups.setdefault(name, [0.0, 0])
                g[0] += 100.0 * (ticks - before) / machine
                g[1] += 1
            out["threads"] = sorted(({"name": k, "cpu_pct": round(v[0], 1), "n": v[1]}
                                     for k, v in groups.items()), key=lambda r: -r["cpu_pct"])
        return out
