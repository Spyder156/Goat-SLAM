#!/usr/bin/env python3
"""Run prepared LaMAria suite cases in parallel with a stall watchdog.

Each case directory holds <sequence>_full/plan.json written by lamaria_suite.py
prepare. For every case: replay -> official scoring -> diagnostics PNG/summary,
exactly as pipeline/run/continuity_experiment.py does, but with a configurable
number of parallel slots, a heartbeat line per running case, and a stall cap:
a replay whose run.log has not grown for --stall-minutes is killed (docker kill
of its container) and marked failed instead of holding a slot forever.
No estimator input, score rule or threshold is changed here.
"""
import argparse
import json
import re
import subprocess
import sys
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

sys.dont_write_bytecode = True
HERE = Path(__file__).resolve()
sys.path.insert(0, str(HERE.parent))
from continuity_experiment import command, diagnostics, save, unscorable_diagnostics  # noqa: E402

ROOT = HERE.parents[2]
LM_TIME = re.compile(r'"time":([0-9.]+)')


def find_plan(case):
    plans = [p for p in case.glob('*/plan.json') if p.parent.name != 'runs']   # one prepared profile dir (full, startup, loss-prefix, ...)
    if len(plans) != 1:
        raise ValueError(f'Expected one prepared profile under {case}')
    return json.loads(plans[0].read_text())


def container_name(run_command):
    out = Path(run_command[run_command.index('--out') + 1])
    return f'lamaria-{out.name}'


def container_cpu_percent(name):
    """CPU use of a running container, or None if it is not running."""
    out = subprocess.run(['docker', 'stats', '--no-stream', '--format', '{{.CPUPerc}}', name], capture_output=True, text=True)
    value = out.stdout.strip().rstrip('%')
    try:
        return float(value)
    except ValueError:
        return None


def tail_time(log):
    """Latest native LM_DIAG time in the last 64 KiB of run.log, or None."""
    try:
        with log.open('rb') as handle:
            handle.seek(0, 2)
            size = handle.tell()
            handle.seek(max(0, size - 65536))
            chunk = handle.read().decode(errors='replace')
    except OSError:
        return None
    hits = LM_TIME.findall(chunk)
    return float(hits[-1]) if hits else None


class Watchdog(threading.Thread):
    def __init__(self, case, plan, stall_s, status):
        super().__init__(daemon=True)
        self.case, self.plan, self.stall_s, self.status = case, plan, stall_s, status
        self.log = Path(plan['run_directory']) / 'run.log'
        self.stop = threading.Event()
        self.killed = False

    def run(self):
        last_size, last_change = -1, time.monotonic()
        while not self.stop.wait(30):
            size = self.log.stat().st_size if self.log.exists() else -1
            if size != last_size:
                last_size, last_change = size, time.monotonic()
            idle = time.monotonic() - last_change
            cpu = container_cpu_percent(container_name(self.plan['run_command']))
            self.status.update(log_bytes=size, idle_s=round(idle), latest_native_s=tail_time(self.log), cpu_percent=cpu)
            # A silent log while the container burns CPU is a long single-threaded
            # solve (periodic full-map calibration), not a hang: only a quiet log
            # AND an idle container count as a stall.
            if idle > self.stall_s and cpu is not None and cpu < 5.0:
                name = container_name(self.plan['run_command'])
                subprocess.run(['docker', 'kill', name], capture_output=True)
                self.killed = True
                self.status.update(state='killed_stalled', idle_s=round(idle))
                return


def wait_for_container(plan, status):
    """Attach mode: the replay was launched by an earlier runner; wait for its container to exit."""
    name = container_name(plan['run_command'])
    log = Path(plan['run_directory']) / 'run.log'
    while subprocess.run(['docker', 'ps', '-q', '--filter', f'name=^{name}$'], capture_output=True, text=True).stdout.strip():
        status.update(state='running_attached', latest_native_s=tail_time(log), cpu_percent=container_cpu_percent(name))
        time.sleep(30)
    time.sleep(20)   # let run_lamaria.py finish its coverage/export bookkeeping
    coverage = Path(plan['run_directory']) / 'coverage.json'
    if not coverage.exists():
        raise RuntimeError('attached replay ended without coverage.json')
    rc = json.loads(coverage.read_text()).get('estimator_returncode')
    if rc not in (0, None):
        raise RuntimeError(f'attached replay estimator returncode {rc}')
    return {'attached': True, 'returncode': rc}


def run_case(case, label, stall_minutes, status, attach=False):
    plan = find_plan(case)
    state = {'case': str(case), 'label': label, 'state': 'running', 'started': time.time(),
             'run_directory': plan['run_directory'], 'attached': attach}
    status.update(state)
    save(case / 'status.json', state)
    dog = Watchdog(case, plan, stall_minutes * 60, status)
    if not attach:
        dog.start()
    try:
        try:
            state['replay'] = wait_for_container(plan, status) if attach else command(case, 'launch', plan['run_command'])
        finally:
            dog.stop.set()
        if dog.killed:
            raise RuntimeError('replay killed by stall watchdog')
        if not plan.get('full_sequence_scoring_allowed', True):
            # Partial profile (startup / loss-prefix / loss-context): no official
            # score is defined. Summarise continuity from the run's own outputs.
            state['result'] = partial_summary(case, plan, label)
            state.update(state='complete_partial', finished=time.time())
            status.update(state='complete_partial')
            save(case / 'status.json', state)
            return state
        state.update(state='scoring'); status.update(state='scoring'); save(case / 'status.json', state)
        try:
            state['scoring'] = command(case, 'score', plan['score_command'])
        except RuntimeError:
            if 'Official control-point alignment failed; no score fabricated' not in (case / 'score.log').read_text():
                raise
            state['scoring'] = json.loads((case / 'score.json').read_text())
            state['result'] = unscorable_diagnostics(case, plan, label)
            state.update(state='complete_unscorable', finished=time.time())
            status.update(state='complete_unscorable')
            save(case / 'status.json', state)
            return state
        state['result'] = diagnostics(case, plan, label)
        state.update(state='complete', finished=time.time())
        status.update(state='complete', Score2D=state['result']['Score2D'])
        save(case / 'status.json', state)
        return state
    except Exception as error:  # noqa: BLE001 - recorded, not hidden
        state.update(state='failed', error=str(error), finished=time.time())
        status.update(state='failed', error=str(error))
        save(case / 'status.json', state)
        return state


def partial_summary(case, plan, label):
    """Continuity summary for a partial-profile replay: resets, map epochs, coasting, first pose."""
    import csv
    run = Path(plan['run_directory'])
    rows = list(csv.DictReader((next(run.glob('online_*.csv'))).open()))
    keys = [(r['map_id'], r['map_init_kf_id']) for r in rows]
    epochs = list(dict.fromkeys(keys))
    coverage = json.loads((run / 'coverage.json').read_text()) if (run / 'coverage.json').exists() else {}
    first_pose = next((float(r['input_t_s']) for r in rows if r['pose_available'] == '1' and r['state'] == '2'), None)
    first_imu = next((float(r['input_t_s']) for r in rows if r['imu_initialized'] == '1'), None)
    resets = sum(1 for line in (run / 'run.log').open(errors='replace') if 'Reseting active map' in line)
    result = {'label': label, 'run': str(run), 'profile': plan.get('profile'), 'input_frames': len(rows),
              'first_s': float(rows[0]['input_t_s']), 'last_s': float(rows[-1]['input_t_s']),
              'first_tracked_pose_s': first_pose, 'first_imu_initialized_s': first_imu,
              'online_map_epochs': len(epochs), 'active_map_resets_logged': resets,
              'state3_frames': sum(r['state'] == '3' for r in rows), 'coasting_frames': sum(r['coasting'] == '1' for r in rows),
              'tracking_ok_frames': sum(r['state'] == '2' for r in rows),
              'exported_pose_fraction': coverage.get('exported_pose_fraction'),
              'longest_segment': coverage.get('longest_independently_framed_contiguous_segment'),
              'segments': [(seg['coordinate_frame'], seg['exported_rows']) for seg in coverage.get('segments', [])]}
    save(case / 'summary.json', result)
    return result


def heartbeat(statuses, stop, period):
    t0 = time.monotonic()
    while not stop.wait(period):
        mem = subprocess.run(['free', '-g'], capture_output=True, text=True).stdout.splitlines()
        avail = mem[1].split()[-1] if len(mem) > 1 else '?'
        parts = []
        for label, s in statuses.items():
            parts.append(f"{label}:{s.get('state','queued')}"
                         + (f"@{s['latest_native_s']:.0f}s" if s.get('latest_native_s') else '')
                         + (f"(idle {s['idle_s']}s)" if s.get('idle_s', 0) > 120 else ''))
        print(f'[heartbeat +{(time.monotonic()-t0)/60:.0f} min, {avail} GiB free] ' + ' | '.join(parts), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cases', type=Path, nargs='+', required=True)
    parser.add_argument('--max-parallel', type=int, default=2)
    parser.add_argument('--stall-minutes', type=float, default=15)
    parser.add_argument('--heartbeat-seconds', type=float, default=120)
    parser.add_argument('--summary', type=Path, required=True)
    parser.add_argument('--attach', action='store_true', help='Adopt replays already running in their containers: wait, score, summarise')
    args = parser.parse_args()
    for case in args.cases:
        if (case / 'status.json').exists() and not args.attach:
            raise FileExistsError(f'Case already launched once; never reuse: {case}')
    statuses = {case.name: {} for case in args.cases}
    stop = threading.Event()
    threading.Thread(target=heartbeat, args=(statuses, stop, args.heartbeat_seconds), daemon=True).start()
    results = []
    with ThreadPoolExecutor(max_workers=args.max_parallel) as pool:
        futures = [pool.submit(run_case, case.resolve(), case.name, args.stall_minutes, statuses[case.name], args.attach) for case in args.cases]
        for future in futures:
            results.append(future.result())
            print('DONE ' + json.dumps({k: results[-1].get(k) for k in ('label', 'state', 'error')}), flush=True)
    stop.set()
    save(args.summary, {'cases': results})
    print('BATCH COMPLETE ' + str(args.summary), flush=True)


if __name__ == '__main__':
    main()
