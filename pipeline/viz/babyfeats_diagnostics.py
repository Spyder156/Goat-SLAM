#!/usr/bin/env python3
"""Inspect a completed BabyFeatures run without altering poses or alignment.

BABY_DIAG is JSON emitted by the actual tracker. Accepted SOS updates are not
counted as restored mature-map tracking; recovered events are kept separate.
"""
import argparse
from collections import Counter
import csv
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
BASELINE = ROOT / ('experiments/lamaria_online_full_v2_long_native_full_20261003/'
                   'runs/lamaria_online_full_v2_long_native_full_20261003_long_full')


def read_json(path):
    return json.loads(path.read_text())


def read_baby_events(path):
    events, malformed = [], []
    decoder = json.JSONDecoder()
    with path.open(errors='replace') as handle:
        for number, line in enumerate(handle, 1):
            if '[BABY_DIAG]' not in line:
                continue
            payload = line.split('[BABY_DIAG]', 1)[1].strip()
            try:
                event, _ = decoder.raw_decode(payload)
                if not isinstance(event, dict) or 'event' not in event:
                    raise ValueError('Missing event field')
            except (json.JSONDecodeError, ValueError) as error:
                malformed.append({'line': number, 'error': str(error), 'text': payload[:500]})
                continue
            event['log_line'] = number
            events.append(event)
    return events, malformed


def episodes(events):
    result, active = [], None
    for event in events:
        kind = event['event']
        if kind == 'enter':
            if active is not None:
                active['outcome'] = 'next_enter_without_logged_end'
                result.append(active)
            active = {'start_native_s': event.get('time'), 'start_map': event.get('map'),
                      'accepted_updates': 0, 'rejected_updates': 0}
        if active is not None:
            if kind in ('accepted', 'rejected'):
                active[kind + '_updates'] += 1
            active['end_native_s'] = event.get('time')
            # A backend/calibration update clears cached observations while
            # keeping the SOS episode active; it is not a map failure/reset.
            window_reset = kind == 'reset' and event.get('reason') in ('map_update', 'calibration_update')
            if window_reset:
                active.setdefault('window_resets', []).append(
                    {'native_s': event.get('time'), 'reason': event.get('reason')})
            if kind in ('recovered', 'expired') or (kind == 'reset' and not window_reset):
                active.update(outcome=kind, end_map=event.get('map'), reason=event.get('reason'))
                result.append(active)
                active = None
    if active is not None:
        active['outcome'] = 'open_at_end_of_log'
        result.append(active)
    for item in result:
        if item['start_native_s'] is not None and item.get('end_native_s') is not None:
            item['duration_s'] = float(item['end_native_s']) - float(item['start_native_s'])
    return result


def run_summary(run):
    coverage = read_json(run / 'coverage.json')
    path = run / 'lamaria_score/scores.json'
    score = read_json(path) if path.exists() else None
    return {'run': str(run), 'Score2D': score['Score2D'] if score else None,
            'retained_maps': coverage['independent_segments_exported'],
            'completed_active_map_resets': coverage['completed_active_map_resets'],
            'new_map_events': coverage['new_map_events'],
            'exported_pose_fraction_all_independent_maps': coverage['exported_pose_fraction'],
            'scored_native_pose_fraction': score['pose_coverage_fraction'] if score else None,
            'CP_recovered': [score['CP_triangulated'], score['CP_total']] if score else None,
            'GT_associated': [score['GT_associated'], score['GT_total_denominator']] if score else None,
            'longest_same_map_contiguous_span': coverage['longest_independently_framed_contiguous_segment'],
            'coverage': coverage}


def accepted_pose_audit(events, online, run):
    """Join accepted SOS timestamps to actual state and final exported poses."""
    import numpy as np
    native = np.asarray([float(row['input_t_s']) for row in online])
    accepted = {float(event['time']) for event in events
                if event['event'] == 'accepted' and 'time' in event}
    mapped, unmatched = {}, []
    for stamp in sorted(accepted):
        index = int(np.searchsorted(native, stamp))
        choices = [i for i in (index - 1, index) if 0 <= i < len(native)]
        closest = min(choices, key=lambda i: abs(native[i] - stamp))
        if abs(native[closest] - stamp) > 1e-6:
            unmatched.append(stamp)
        else:
            mapped[round(float(native[closest]), 6)] = online[closest]
    atlas = next(run.glob('atlas_*_trajectory.csv'))
    with atlas.open() as handle:
        exported = {round(float(row['t_s']), 6): row for row in csv.DictReader(handle)}
    score_path = run / 'lamaria_score/scores.json'
    selected = (read_json(score_path)['trajectory_selection']['selected_coordinate_frame']
                if score_path.exists() else None)
    retained = set(mapped) & set(exported)
    return {'accepted_unique_timestamps': len(accepted),
            'accepted_timestamps_without_native_trace': unmatched,
            'accepted_online_state_counts': dict(Counter(row['state'] for row in mapped.values())),
            'accepted_pose_available_count': sum(int(row['pose_available']) for row in mapped.values()),
            'accepted_marked_coasting_count': sum(int(row['coasting']) for row in mapped.values()),
            'accepted_exported_across_all_independent_maps': len(retained),
            'accepted_exported_in_scored_map': (sum(exported[t]['coordinate_frame'] == selected for t in retained)
                                               if selected else None),
            'note': 'SOS state 3 is RECENTLY_LOST, distinct from mature tracking state 2; retained exports remain estimates, not proof of visual recovery.'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case', type=Path, required=True)
    parser.add_argument('--out', type=Path, help='Fresh output; default <case>/babyfeats_diagnostics')
    args = parser.parse_args()
    case = args.case.resolve(strict=True)
    out = (args.out or case / 'babyfeats_diagnostics').resolve()
    if out.exists():
        raise FileExistsError(f'Diagnostics already exist; choose a fresh --out: {out}')
    plan = read_json(case / 'long_full/plan.json')
    run = Path(plan['run_directory'])
    candidate, baseline = run_summary(run), run_summary(BASELINE)
    events, malformed = read_baby_events(run / 'run.log')
    with next(run.glob('online_*.csv')).open() as handle:
        online = list(csv.DictReader(handle))
    counts = dict(Counter(event['event'] for event in events))
    spans = episodes(events)
    result = {'candidate': candidate, 'frozen_native_v2': baseline,
              'baby_event_counts': counts, 'sos_episodes': spans,
              'accepted_pose_audit': accepted_pose_audit(events, online, run),
              'malformed_baby_log_rows': malformed,
              'historical_long': {'Score2D': 49.881533645273585, 'retained_maps': 1,
                                  'caveat': 'Different initial calibration; not a matched ablation'},
              'interpretation': ['SOS accepted updates are distinct from mature-map recovery.',
                                 'All-map coverage does not imply a single connected trajectory.',
                                 'Retained maps, map-creation events and resets are separate counts.',
                                 'No poses, independent-map transforms or scores are synthesized.']}
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import numpy as np
    out.mkdir(parents=True)
    (out / 'summary.json').write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    (out / 'events.json').write_text(json.dumps(events, indent=2, allow_nan=False) + '\n')
    fig, axes = plt.subplots(3, 1, figsize=(15, 9), sharex=True, constrained_layout=True)
    origin = candidate['coverage']['input_start_s']
    for name, summary, color in [('Frozen v2', baseline, '#855196'), ('BabyFeatures SOS', candidate, '#188455')]:
        for index, segment in enumerate(summary['coverage']['segments']):
            lane = index + (0 if name == 'Frozen v2' else len(baseline['coverage']['segments']) + 1)
            for interval in segment['contiguous_spans']:
                axes[0].plot([interval['start_s'] - origin, interval['end_s'] - origin],
                             [lane, lane], color=color, lw=5,
                             label=name if index == 0 and interval == segment['contiguous_spans'][0] else None)
    axes[0].set(ylabel='Retained independent map')
    axes[0].legend(loc='upper left')
    times = np.array([float(row['input_t_s']) - origin for row in online])
    axes[1].plot(times, [int(row['inliers']) for row in online], lw=.55, color='#23609d', label='Mature-map inliers')
    axes[1].set(ylabel='Online map inliers', ylim=(0, None))
    accepted = [event for event in events if event['event'] == 'accepted' and 'time' in event]
    rejected = [event for event in events if event['event'] == 'rejected' and 'time' in event]
    for group, color, label in [(accepted, '#188455', 'SOS accepted'), (rejected, '#bd3c38', 'SOS rejected')]:
        axes[2].scatter([float(event['time']) - origin for event in group],
                        [int(event.get('inliers', 0)) for event in group], s=9, color=color, label=label)
    for episode in spans:
        if episode.get('start_native_s') is None or episode.get('end_native_s') is None:
            continue
        for axis in axes[1:]:
            axis.axvspan(float(episode['start_native_s']) - origin,
                         float(episode['end_native_s']) - origin, color='#e7ba52', alpha=.18)
    axes[1].legend(loc='upper left')
    axes[2].legend(loc='upper left')
    axes[2].set(xlabel='Elapsed input time [s]', ylabel='BabyFeature inliers', ylim=(0, None))
    for axis in axes:
        axis.grid(alpha=.2)
    candidate_score = 'unscorable' if candidate['Score2D'] is None else f'{candidate["Score2D"]:.2f}'
    fig.suptitle(f'Long: frozen v2 {baseline["Score2D"]:.2f}, {baseline["retained_maps"]} maps → '
                 f'BabyFeatures {candidate_score}, {candidate["retained_maps"]} maps\n'
                 'Map survival and accepted SOS constraints are reported separately')
    fig.savefig(out / 'continuity.png', dpi=170)
    plt.close(fig)
    print(json.dumps({key: value for key, value in result.items() if key not in ('candidate', 'frozen_native_v2')}, indent=2))
    print(str(out / 'continuity.png'))


if __name__ == '__main__':
    main()
