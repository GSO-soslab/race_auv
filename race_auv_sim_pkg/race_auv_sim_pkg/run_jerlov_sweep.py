"""Sequentially run the sim+bag for each value of `environment.ocean.jerlov`.

Invoked by `race_station_light.launch.py` when `jerlov` is a list. For each
value it writes a temporary sim.yaml (with that single value) and launches
the same launch file with `config:=<tmp>`. The inner launch handles the
sim and the bag via the existing inline bag runner; the helper waits for
the bag to be flushed (`sim_info.txt` appears in the new bag dir), then
SIGTERMs the inner `ros2 launch` process group before starting the next
iteration.
"""

import argparse
import copy
import os
import shutil
import signal
import subprocess
import sys
import tempfile
import time
from typing import List

import yaml


def _load_yaml(path: str) -> dict:
    with open(path, 'r') as f:
        return yaml.safe_load(f) or {}


def _normalize_jerlov(raw) -> List[float]:
    if isinstance(raw, (int, float)):
        return [float(raw)]
    if isinstance(raw, (list, tuple)):
        out = []
        for v in raw:
            if not isinstance(v, (int, float)):
                raise ValueError(f"jerlov list entries must be numbers, got {type(v).__name__}: {v!r}")
            out.append(float(v))
        if not out:
            raise ValueError("jerlov list is empty")
        return out
    raise ValueError(f"jerlov must be a number or list of numbers, got {type(raw).__name__}")


def _jerlov_tag(value: float) -> str:
    return "jerlov_" + str(value).replace('.', 'p')


def _build_iter_config(src_cfg: dict, value: float) -> dict:
    cfg = copy.deepcopy(src_cfg)
    env = cfg.setdefault('environment', {})
    ocean = env.setdefault('ocean', {})
    ocean['jerlov'] = value
    return cfg


def _resolve_bag_dir(rec_cfg: dict, value: float) -> str:
    output_dir = rec_cfg.get('output_dir', '/tmp')
    bag_name = rec_cfg.get('bag_name', 'race_station_light')
    return os.path.join(output_dir, f"{bag_name}_{_jerlov_tag(value)}")


def _resolve_bag_dir_with_suffix(base: str) -> str:
    """Apply the same _N collision suffix logic the inline bag runner uses."""
    if not os.path.exists(base):
        return base
    n = 1
    while os.path.exists(f"{base}_{n}"):
        n += 1
    return f"{base}_{n}"


def _print_progress(line: str) -> None:
    sys.stdout.write('\r' + line)
    sys.stdout.flush()


def _wait_for_sim_info(
    bag_dir: str,
    total_s: float,
    start_delay_s: float,
    recording_s: float,
    poll_s: float = 0.5,
    bar_width: int = 30,
) -> bool:
    """Block until sim_info.txt appears or total_s elapses. Renders a bar."""
    deadline = time.time() + total_s
    start = time.time()
    flush_start = start_delay_s + recording_s
    while time.time() < deadline:
        if os.path.isfile(os.path.join(bag_dir, 'sim_info.txt')):
            bar = '[' + '#' * bar_width + ']'
            _print_progress(
                f"  {bar} 100.0%  phase=flushed  "
                f"t={total_s:6.1f}/{total_s:6.1f}s"
            )
            sys.stdout.write('\n')
            sys.stdout.flush()
            return True
        elapsed = time.time() - start
        pct = min(1.0, elapsed / total_s) if total_s > 0 else 0.0
        filled = int(pct * bar_width)
        bar = '[' + '#' * filled + '-' * (bar_width - filled) + ']'
        if elapsed < start_delay_s:
            phase = 'pre-roll'
        elif elapsed < flush_start:
            phase = 'recording'
        else:
            phase = 'flushing '
        _print_progress(
            f"  {bar} {pct*100:5.1f}%  phase={phase}  "
            f"t={elapsed:6.1f}/{total_s:6.1f}s"
        )
        time.sleep(poll_s)
    _print_progress(
        f"  [{'-' * bar_width}] timeout  phase=timeout  "
        f"t={total_s:6.1f}/{total_s:6.1f}s\n"
    )
    return False


def _wait_with_progress(total_s: float, poll_s: float = 0.5, bar_width: int = 30) -> None:
    """Sleep total_s while showing a generic 'running' bar (no bag)."""
    deadline = time.time() + total_s
    start = time.time()
    while time.time() < deadline:
        elapsed = time.time() - start
        pct = min(1.0, elapsed / total_s) if total_s > 0 else 0.0
        filled = int(pct * bar_width)
        bar = '[' + '#' * filled + '-' * (bar_width - filled) + ']'
        _print_progress(
            f"  {bar} {pct*100:5.1f}%  phase=running   "
            f"t={elapsed:6.1f}/{total_s:6.1f}s"
        )
        time.sleep(poll_s)
    _print_progress(
        f"  [{'#' * bar_width}] 100.0%  phase=stopping  "
        f"t={total_s:6.1f}/{total_s:6.1f}s\n"
    )


def _terminate_proc(proc: subprocess.Popen, grace_s: float = 10.0) -> int:
    if proc.poll() is not None:
        return proc.returncode
    try:
        os.killpg(proc.pid, signal.SIGTERM)
    except ProcessLookupError:
        return proc.returncode or 0
    try:
        return proc.wait(timeout=grace_s)
    except subprocess.TimeoutExpired:
        pass
    try:
        os.killpg(proc.pid, signal.SIGKILL)
    except ProcessLookupError:
        return proc.returncode or 0
    try:
        return proc.wait(timeout=5.0)
    except subprocess.TimeoutExpired:
        return -1


def _iter_duration_s(rec_cfg: dict) -> float:
    try:
        return float(rec_cfg.get('duration_sec', 0) or 0)
    except (TypeError, ValueError):
        return 0.0


def _iter_start_delay_s(rec_cfg: dict) -> float:
    try:
        return float(rec_cfg.get('start_delay_sec', 0) or 0)
    except (TypeError, ValueError):
        return 0.0


def _run_one(
    src_cfg: dict,
    src_config_path: str,
    value: float,
    idx: int,
    total: int,
    use_sim_time: str,
    sim_world: str,
    continue_on_error: bool,
    iter_tmpdirs: List[str],
) -> int:
    rec_cfg = src_cfg.get('recording', {}) or {}
    rec_enabled = str(rec_cfg.get('enabled', False)).lower() == 'true'
    rec_duration = _iter_duration_s(rec_cfg)
    rec_start_delay = _iter_start_delay_s(rec_cfg)

    iter_cfg = _build_iter_config(src_cfg, value)
    tmp_dir = tempfile.mkdtemp(prefix=f'jerlov_{_jerlov_tag(value)}_')
    iter_tmpdirs.append(tmp_dir)
    tmp_yaml = os.path.join(tmp_dir, 'sim.yaml')
    with open(tmp_yaml, 'w') as f:
        yaml.safe_dump(iter_cfg, f, sort_keys=False)

    base_bag_dir = _resolve_bag_dir(rec_cfg, value) if rec_enabled else ''
    expected_bag_dir = _resolve_bag_dir_with_suffix(base_bag_dir) if base_bag_dir else ''

    print(
        f"[sweep {idx}/{total} jerlov={value}] launching "
        f"config={tmp_yaml} bag_dir={expected_bag_dir or '(recording disabled)'}",
        flush=True,
    )

    cmd = [
        'ros2', 'launch', 'race_auv_sim_pkg', 'race_station_light.launch.py',
        f'config:={tmp_yaml}',
        f'sim_world:={sim_world}',
        f'use_sim_time:={use_sim_time}',
    ]
    try:
        proc = subprocess.Popen(cmd, preexec_fn=os.setsid)
    except FileNotFoundError:
        print("[sweep] 'ros2' CLI not found in PATH", file=sys.stderr)
        return 127

    # Total wall-clock budget for this iteration, used to drive the
    # progress bar. Covers pre-roll + recording + 20s flush grace.
    if rec_duration > 0:
        wait_budget = rec_start_delay + rec_duration + 20.0
    else:
        wait_budget = max(60.0, rec_start_delay + 30.0)

    finished_normally = False
    if expected_bag_dir:
        if _wait_for_sim_info(
            expected_bag_dir,
            total_s=wait_budget,
            start_delay_s=rec_start_delay,
            recording_s=rec_duration if rec_duration > 0 else 0.0,
        ):
            finished_normally = True
        else:
            print(
                f"[sweep {idx}/{total} jerlov={value}] WARNING: "
                f"sim_info.txt not seen within {wait_budget:.1f}s; "
                "stopping inner launch anyway.",
                file=sys.stderr,
                flush=True,
            )
    else:
        # No bag to wait on. Run the sim for a fixed budget with a bar.
        _wait_with_progress(wait_budget)
        if proc.poll() is not None:
            finished_normally = True

    rc = _terminate_proc(proc)
    if finished_normally and rc < 0:
        rc = 0
    print(
        f"[sweep {idx}/{total} jerlov={value}] inner launch exited rc={rc} "
        f"({'sim_info seen' if finished_normally else 'timeout/forced stop'})",
        flush=True,
    )
    if rc != 0 and not continue_on_error:
        return rc
    return 0


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument('--config', required=True, help='Path to sim.yaml')
    p.add_argument('--sim-world', default='race_station_light.scn')
    p.add_argument('--use-sim-time', default='false')
    p.add_argument(
        '--list-only', action='store_true',
        help='Print the iteration plan and exit.',
    )
    p.add_argument(
        '--continue-on-error', action='store_true',
        help='Keep going if an inner launch returns non-zero.',
    )
    args = p.parse_args()

    src_cfg = _load_yaml(args.config)
    jerlov_raw = (src_cfg.get('environment', {}) or {}).get('ocean', {}).get('jerlov', 0.2)
    values = _normalize_jerlov(jerlov_raw)

    print(f"[sweep] plan: jerlov values = {values}", flush=True)
    if args.list_only:
        return 0

    iter_tmpdirs: List[str] = []
    overall_rc = 0
    for i, v in enumerate(values, start=1):
        rc = _run_one(
            src_cfg=src_cfg,
            src_config_path=args.config,
            value=v,
            idx=i,
            total=len(values),
            use_sim_time=args.use_sim_time,
            sim_world=args.sim_world,
            continue_on_error=args.continue_on_error,
            iter_tmpdirs=iter_tmpdirs,
        )
        if rc != 0:
            overall_rc = rc
            if not args.continue_on_error:
                break

    for d in iter_tmpdirs:
        shutil.rmtree(d, ignore_errors=True)

    print(f"[sweep] done. overall rc={overall_rc}", flush=True)
    return overall_rc


if __name__ == '__main__':
    sys.exit(main())
