#!/usr/bin/env python3
"""ZED / Jetson Orin resource monitor (read-only, no sudo, Python 3 standard library only).

Records the same numbers as the SDK 5.6 depth experiments (E0..E6, E4/E5 long runs):
tegrastats every 1 s (GPU %, CPU %, RAM, power rails, temperatures), the ZED container's
own CPU/RSS, container restarts, and ZED topic rates every 10 min (12 s each).

Live run (in a shell where the robot's ROS setup is sourced, same ROS_DOMAIN_ID / RMW):
    python3 zed_resource_monitor.py                    # 60 min, results in ~/zed_monitor_<host>_<time>/
    python3 zed_resource_monitor.py --duration 30      # minutes
    python3 zed_resource_monitor.py --no-hz            # without topic rates (no ROS needed)

Summarise existing tegrastats logs the same way (e.g. the E4/E5 runs):
    python3 zed_resource_monitor.py --summarize E4.tegrastats E5.tegrastats

Ctrl+C stops early; the summary is still written. Nothing on the robot is changed.

source /opt/ros/humble/setup.bash; source ~/zed_ws/install/setup.bash
export ROS_DOMAIN_ID=55 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
nohup python3 zed_resource_monitor.py > zed_monitor.out 2>&1 &    # 60분, 끝나면 자동 종료
- 켜는 환경: 위 export 두 줄은 run_all_auto.sh의 설정(도메인 55, cyclonedds)과 같아야 토픽이 보입니다.
- SSH로 돌릴 때: nohup을 붙이면 접속이 끊겨도 계속 돌아갑니다. 로봇 화면의 터미널에서 직접 실행해도 됩니다.
- 변경 없음: sudo 없이 읽기만 하고, 로봇 설정은 아무것도 바꾸지 않습니다.
- 결과 위치: ~/zed_monitor_<호스트>_<시각>/에 저장됩니다. summary.txt를 주시고, 가능하면 tegrastats.log도 함께 주세요. 원본 로그가 있으면 제가 같은 식으로 다시 계산해 볼 수 있습니다.
"""
import argparse
import datetime as dt
import glob
import json
import os
import re
import shutil
import signal
import socket
import statistics as st
import subprocess
import sys
import threading
import time

HZ_SECONDS = 12
BUCKET_S = 600

RE_GPU = re.compile(r"GR3D_FREQ (\d+)%")
RE_RAM = re.compile(r"RAM (\d+)/(\d+)MB")
RE_CPU = re.compile(r"(\d+)%@\d+")
RE_RAIL = re.compile(r"(\b[A-Z][A-Z0-9_]+) (\d+)mW/\d+mW")
RE_TEMP = re.compile(r"\b(gpu|tj|cpu)@(-?[\d.]+)C")


# ----------------------------------------------------------------------------- parsing
def parse_tegrastats_line(line):
    g, r = RE_GPU.search(line), RE_RAM.search(line)
    cpus = [int(x) for x in RE_CPU.findall(line.split("GR3D_FREQ")[0])]
    rails = {k: int(v) / 1000.0 for k, v in RE_RAIL.findall(line)}
    if not (g and r and cpus and rails):
        return None
    temps = {k: float(v) for k, v in RE_TEMP.findall(line)}
    return {
        "gpu": int(g.group(1)),
        "cpu": st.mean(cpus),
        "ram_gb": int(r.group(1)) / 1024.0,
        "rails": rails,
        "t_gpu": temps.get("gpu"),
        "t_tj": temps.get("tj"),
    }


def pct(sorted_vals, q):
    return sorted_vals[min(len(sorted_vals) - 1, int(len(sorted_vals) * q))]


def mean_of(rows, key):
    vals = [r[key] for r in rows if r.get(key) is not None]
    return st.mean(vals) if vals else float("nan")


def summarize_rows(rows):
    g = sorted(r["gpu"] for r in rows)
    rail_names = sorted({k for r in rows for k in r["rails"]})
    rails = {k: st.mean(r["rails"].get(k, 0.0) for r in rows) for k in rail_names}
    return {
        "n": len(rows),
        "gpu_avg": st.mean(g), "gpu_p50": pct(g, .5), "gpu_p95": pct(g, .95), "gpu_max": g[-1],
        "gpu_ge90_pct": sum(x >= 90 for x in g) / len(g) * 100,
        "cpu_avg": mean_of(rows, "cpu"), "ram_gb": mean_of(rows, "ram_gb"),
        "rails_w": rails, "power_sum_w": sum(rails.values()),
        "t_gpu": mean_of(rows, "t_gpu"), "t_tj": mean_of(rows, "t_tj"),
    }


def buckets(rows):
    out = []
    for i in range(0, len(rows), BUCKET_S):
        b = rows[i:i + BUCKET_S]
        if len(b) < 120:
            continue
        s = sorted(x["gpu"] for x in b)
        out.append({"from_min": i // 60, "gpu_avg": st.mean(s), "gpu_p95": pct(s, .95),
                    "gpu_soc_w": st.mean(x["rails"].get("VDD_GPU_SOC", 0.0) for x in b),
                    "cpu_avg": st.mean(x["cpu"] for x in b)})
    return out


def format_table(named):
    head = (f"{'run':28} {'n':>5} {'GPUavg':>6} {'p50':>4} {'p95':>4} {'max':>4} {'>=90%':>6} "
            f"{'CPU%':>5} {'RAM GB':>6} {'GPU_SOC W':>9} {'CPU_CV W':>8} {'SYS5V W':>7} {'sum W':>6} "
            f"{'Tgpu':>5} {'Ttj':>5}")
    lines = [head]
    for name, s in named:
        r = s["rails_w"]
        lines.append(
            f"{name[:28]:28} {s['n']:5} {s['gpu_avg']:6.1f} {s['gpu_p50']:4} {s['gpu_p95']:4} "
            f"{s['gpu_max']:4} {s['gpu_ge90_pct']:5.1f}% {s['cpu_avg']:5.1f} {s['ram_gb']:6.2f} "
            f"{r.get('VDD_GPU_SOC', float('nan')):9.2f} {r.get('VDD_CPU_CV', float('nan')):8.2f} "
            f"{r.get('VIN_SYS_5V0', float('nan')):7.2f} {s['power_sum_w']:6.2f} "
            f"{s['t_gpu']:5.1f} {s['t_tj']:5.1f}")
    return "\n".join(lines)


def format_buckets(name, bl):
    if not bl:
        return f"{name}: (shorter than 2 min, no 10-min buckets)"
    cells = " | ".join(f"{b['gpu_avg']:4.1f}/{b['gpu_p95']:2}/{b['gpu_soc_w']:4.1f}/{b['cpu_avg']:4.1f}"
                       for b in bl)
    return f"{name}: {cells}"


def load_tegrastats(path):
    rows = []
    with open(path, errors="replace") as f:
        for line in f:
            r = parse_tegrastats_line(line)
            if r:
                rows.append(r)
    return rows


# ----------------------------------------------------------------------------- environment
def run(cmd, timeout=10):
    try:
        p = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                           timeout=timeout, text=True)
        return p.stdout.strip()
    except Exception as e:  # noqa: BLE001 - report and continue
        return f"({type(e).__name__})"


def read(path):
    try:
        with open(path, errors="replace") as f:
            return f.read()
    except OSError:
        return ""


def sdk_version():
    h = read("/usr/local/zed/include/sl/Camera.hpp")
    v = [re.search(r"#define ZED_SDK_%s_VERSION (\d+)" % k, h) for k in ("MAJOR", "MINOR", "PATCH")]
    return ".".join(m.group(1) for m in v) if all(v) else "unknown"


def camera_configs(ws):
    cfg_dirs = glob.glob(os.path.join(ws, "src", "**", "zed_wrapper", "config"), recursive=True)
    cams = {}
    for d in cfg_dirs:
        for f in sorted(glob.glob(os.path.join(d, "common_stereo_zed_*.yaml"))):
            name = os.path.basename(f)[len("common_stereo_"):-len(".yaml")]
            txt = read(f)
            item = {}
            for key in ("depth_mode", "depth_precision", "depth_compute_device", "pub_frame_rate",
                        "od_enabled", "detection_model"):
                m = re.search(r"^\s+%s:\s*([^#\n]+)" % key, txt, re.M)
                if m:
                    item[key] = m.group(1).strip().strip("'\"")
            cam_txt = read(os.path.join(d, f"zedx_{name}.yaml"))
            for key in ("grab_resolution", "grab_frame_rate"):
                m = re.search(r"^\s+%s:\s*([^#\n]+)" % key, cam_txt, re.M)
                if m:
                    item[key] = m.group(1).strip().strip("'\"")
            cams[name] = item
        if cams:
            break
    return cams


def environment(ws):
    return {
        "host": socket.gethostname(),
        "l4t": read("/etc/nv_tegra_release").splitlines()[0] if read("/etc/nv_tegra_release") else "",
        "kernel": os.uname().release,
        "zed_sdk": sdk_version(),
        "zedx_driver": run(["dpkg-query", "-W", "-f=${Package} ${Version}\n", "stereolabs-*"]),
        "nvpmodel": run(["nvpmodel", "-q"]).splitlines()[0:1],
        "ros_domain_id": os.environ.get("ROS_DOMAIN_ID", ""),
        "rmw": os.environ.get("RMW_IMPLEMENTATION", ""),
        "workspace": ws,
        "cameras_config": camera_configs(ws),
    }


# ----------------------------------------------------------------------------- live probes
def zed_containers():
    pids = []
    for p in glob.glob("/proc/[0-9]*/cmdline"):
        try:
            with open(p, "rb") as f:
                cmd = f.read().replace(b"\0", b" ").decode(errors="replace")
        except OSError:
            continue
        if "component_container" in cmd and "zed" in cmd:
            pids.append(int(p.split("/")[2]))
    return sorted(pids)


def proc_cpu_rss(pid):
    s = read(f"/proc/{pid}/stat")
    if not s:
        return None
    f = s.rsplit(")", 1)[1].split()
    ticks = int(f[11]) + int(f[12])
    rss_kb = int(re.search(r"VmRSS:\s+(\d+)", read(f"/proc/{pid}/status") or "VmRSS: 0").group(1))
    return ticks, rss_kb


def publisher_count(topic):
    m = re.search(r"Publisher count: (\d+)", run(["ros2", "topic", "info", topic], timeout=20))
    return int(m.group(1)) if m else 0


def zed_topics():
    """Per running camera: its point cloud if it publishes depth, else its gray image.
    Topics without a publisher (e.g. a point cloud that only has subscribers) are skipped."""
    tops = set(run(["ros2", "topic", "list"], timeout=20).splitlines())
    cams = sorted({t.split("/point_cloud/")[0] for t in tops if t.endswith("/point_cloud/cloud_registered")}
                  | {t.split("/left/gray/")[0] for t in tops if t.endswith("/left/gray/rect/image")})
    chosen = []
    for c in cams:
        for t in (c + "/point_cloud/cloud_registered", c + "/left/gray/rect/image"):
            if t in tops and publisher_count(t) > 0:
                chosen.append(t)
                break
    return chosen


def topic_hz(topic):
    try:
        p = subprocess.run(["ros2", "topic", "hz", topic], stdout=subprocess.PIPE,
                           stderr=subprocess.DEVNULL, text=True, timeout=HZ_SECONDS)
        out = p.stdout
    except subprocess.TimeoutExpired as e:
        out = e.stdout.decode() if isinstance(e.stdout, bytes) else (e.stdout or "")
    rates = re.findall(r"average rate: ([\d.]+)", out)
    return float(rates[-1]) if rates else 0.0


# ----------------------------------------------------------------------------- live run
def live(args):
    if not shutil.which("tegrastats"):
        sys.exit("tegrastats not found: run this on the Jetson Orin")
    stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    host = socket.gethostname()
    out = os.path.expanduser(args.out or f"~/zed_monitor_{host}_{stamp}")
    os.makedirs(out, exist_ok=True)
    env = environment(os.path.expanduser(args.ws))
    with open(os.path.join(out, "environment.json"), "w") as f:
        json.dump(env, f, indent=2, ensure_ascii=False)

    use_hz = not args.no_hz and shutil.which("ros2")
    topics = zed_topics() if use_hz else []
    if not args.no_hz and not use_hz:
        print("ros2 not found in this shell: topic rates are skipped (source the robot's ROS setup)")
    elif use_hz and not topics:
        print("no ZED topics found: check ROS_DOMAIN_ID / RMW_IMPLEMENTATION; topic rates are skipped")
    print(f"output: {out}\nZED SDK {env['zed_sdk']}, cameras: "
          + ", ".join(f"{k}={v.get('depth_mode', '?')}" for k, v in env["cameras_config"].items()))
    print(f"topics: {topics if topics else '-'}\nrunning {args.duration} min (Ctrl+C stops early) ...")

    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    ts_path = os.path.join(out, "tegrastats.log")
    ts_file = open(ts_path, "w")
    ts = subprocess.Popen(["tegrastats", "--interval", "1000"], stdout=ts_file,
                          stderr=subprocess.STDOUT)
    hz_log, cont_log = [], []
    seen_pids = set(zed_containers())
    first_pids = sorted(seen_pids)
    restarts = 0
    prev = {p: proc_cpu_rss(p) for p in seen_pids}
    t0 = time.time()
    next_hz = t0 + args.hz_interval * 60
    next_proc = t0 + 10
    last_proc = t0
    hz_tick = os.sysconf("SC_CLK_TCK")
    end = t0 + args.duration * 60
    try:
        while not stop.is_set() and time.time() < end:
            stop.wait(1.0)
            now = time.time()
            if now >= next_proc:
                pids = zed_containers()
                new = [p for p in pids if p not in seen_pids]
                if new:
                    restarts += 1
                    seen_pids.update(new)
                    print(f"  {dt.datetime.now():%H:%M:%S} ZED container (re)started: pid {new}")
                cpu_pct, rss_mb = 0.0, 0.0
                for p in pids:
                    cur = proc_cpu_rss(p)
                    if cur and prev.get(p):
                        cpu_pct += (cur[0] - prev[p][0]) / hz_tick / (now - last_proc) * 100
                    if cur:
                        rss_mb += cur[1] / 1024
                    prev[p] = cur
                cont_log.append({"t_min": round((now - t0) / 60, 2), "pids": pids,
                                 "cpu_pct_of_one_core": round(cpu_pct, 1), "rss_mb": round(rss_mb)})
                if not pids:
                    print(f"  {dt.datetime.now():%H:%M:%S} no ZED container running")
                next_proc = now + 10
                last_proc = now
            if topics and now >= next_hz:
                sample = {"t_min": round((now - t0) / 60), **{t: topic_hz(t) for t in topics}}
                hz_log.append(sample)
                print(f"  +{sample['t_min']} min  "
                      + "  ".join(f"{t.split('/')[2] if t.count('/') > 2 else t}={sample[t]:.2f}"
                                  for t in topics))
                next_hz = now + args.hz_interval * 60
    finally:
        ts.terminate()  # resource numbers cover the requested window only
        try:
            ts.wait(5)
        except subprocess.TimeoutExpired:
            ts.kill()
        ts_file.close()
    if topics and (not hz_log or time.time() - t0 - hz_log[-1]["t_min"] * 60 > 120):
        hz_log.append({"t_min": round((time.time() - t0) / 60), **{t: topic_hz(t) for t in topics}})

    rows = load_tegrastats(ts_path)
    if not rows:
        sys.exit(f"no tegrastats samples in {ts_path}")
    s = summarize_rows(rows)
    bl = buckets(rows)
    zc = [c for c in cont_log if c["pids"]]
    result = {
        "environment": env, "start": dt.datetime.fromtimestamp(t0).isoformat(timespec="seconds"),
        "minutes": round(len(rows) / 60, 1), "summary": s, "buckets_10min": bl,
        "topic_hz": hz_log, "zed_container": {
            "pids_at_start": first_pids, "restarts": restarts,
            "cpu_pct_of_one_core_avg": round(st.mean(c["cpu_pct_of_one_core"] for c in zc[1:]), 1)
            if len(zc) > 1 else None,
            "rss_mb_avg": round(st.mean(c["rss_mb"] for c in zc)) if zc else None},
    }
    with open(os.path.join(out, "summary.json"), "w") as f:
        json.dump(result, f, indent=2, ensure_ascii=False)
    with open(os.path.join(out, "container.jsonl"), "w") as f:
        for c in cont_log:
            f.write(json.dumps(c) + "\n")

    name = f"{host} SDK {env['zed_sdk']}"
    text = [f"# ZED resource monitor  {host}  {result['start']}  {result['minutes']} min",
            f"L4T: {env['l4t']}", f"ZED SDK {env['zed_sdk']}  driver: {env['zedx_driver']}",
            f"power mode: {' '.join(env['nvpmodel'])}", "cameras (config files):"]
    for k, v in env["cameras_config"].items():
        text.append(f"  {k}: " + ", ".join(f"{a}={b}" for a, b in v.items()))
    text += ["", format_table([(name, s)]), "",
             "10-min buckets  GPU avg / p95 / GPU_SOC W / CPU%", format_buckets(name, bl), ""]
    zcs = result["zed_container"]
    text.append(f"ZED container: restarts {zcs['restarts']}, CPU {zcs['cpu_pct_of_one_core_avg']}% "
                f"of one core, RSS {zcs['rss_mb_avg']} MB")
    if hz_log:
        text.append("topic rates (Hz): " + ", ".join(
            f"{t.split('/')[2]}={'point_cloud' if 'point_cloud' in t else 'image'}" for t in topics))
        for h in hz_log:
            text.append(f"  +{h['t_min']:>3} min  " + "  ".join(
                f"{t.split('/')[2]}={h[t]:.2f}" for t in topics))
        low = [(h["t_min"], t.split("/")[2], h[t]) for h in hz_log for t in topics if h[t] < 14.0]
        text.append(f"  samples below 14 Hz: {len(low)} {low if low else ''}")
    report = "\n".join(text) + "\n"
    with open(os.path.join(out, "summary.txt"), "w") as f:
        f.write(report)
    print("\n" + report + f"\nfiles: {out}/ (summary.txt, summary.json, tegrastats.log, "
          "environment.json, container.jsonl)")


def summarize_files(paths):
    named, all_b = [], []
    for p in paths:
        rows = load_tegrastats(p)
        if not rows:
            print(f"{p}: no tegrastats samples")
            continue
        name = os.path.basename(p).rsplit(".", 1)[0]
        if name == "tegrastats":
            name = os.path.basename(os.path.dirname(os.path.abspath(p)))
        named.append((name, summarize_rows(rows)))
        all_b.append(format_buckets(name, buckets(rows)))
    if named:
        print(format_table(named))
        print("\n10-min buckets  GPU avg / p95 / GPU_SOC W / CPU%")
        print("\n".join(all_b))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--duration", type=float, default=60, help="minutes (default 60)")
    ap.add_argument("--out", help="output folder (default ~/zed_monitor_<host>_<time>)")
    ap.add_argument("--ws", default="~/zed_ws", help="ZED workspace for reading camera configs")
    ap.add_argument("--hz-interval", type=float, default=10, help="minutes between topic-rate samples")
    ap.add_argument("--no-hz", action="store_true", help="do not measure topic rates")
    ap.add_argument("--summarize", nargs="+", metavar="TEGRASTATS_LOG",
                    help="only summarise existing tegrastats logs")
    args = ap.parse_args()
    if args.summarize:
        summarize_files(args.summarize)
    else:
        live(args)


if __name__ == "__main__":
    main()
