"""Reconstruct passive BabyFeature descriptor tracks for a saved run.

This is visualization, not estimator replay. The saved keypoint dump determines
the actual ordered, domain-filtered feature set; cached descriptors reconstruct
the original mutual-Hamming matching. Per-point SOS optimizer inlier identities
were not recorded and are deliberately never inferred here. Aggregate accepted
inlier counts and SOS activity come from the original run's diagnostic events.
"""
from bisect import bisect_left, bisect_right
from collections import Counter, defaultdict
import csv
import json
from pathlib import Path
import struct
import time

import cv2
import numpy as np


def _empty_camera():
    return {'pixels': np.empty((0, 2), dtype=np.float32),
            'track_ids': np.empty(0, dtype=np.int64),
            'ages': np.empty(0, dtype=np.int32),
            'mapped': np.empty(0, dtype=bool)}


def _ordered_cache_indices(cached_pixels, saved_pixels):
    """Recover unique row correspondence, including repeated rounded pixels.

    Native CSV rounds pixel coordinates to two decimals. Forward and backward
    subsequence embeddings must agree before a descriptor identity is trusted.
    If the dump genuinely reordered rows, unique rounded pixels still identify
    their descriptors. Ambiguous duplicates fail explicitly.
    """
    saved = np.rint(np.asarray(saved_pixels, dtype=np.float64) * 100).astype(np.int64)
    cached = np.rint(np.asarray(cached_pixels, dtype=np.float64) * 100).astype(np.int64)
    if len(saved) == 0:
        return np.empty(0, dtype=np.int64), 'empty'
    if len(saved) == len(cached) and np.array_equal(saved, cached):
        return np.arange(len(saved), dtype=np.int64), 'all_rows'
    positions = defaultdict(list)
    for index, pixel in enumerate(cached):
        positions[(int(pixel[0]), int(pixel[1]))].append(index)
    keys = [(int(p[0]), int(p[1])) for p in saved]
    forward, cursor = [], -1
    for key in keys:
        choices = positions.get(key, ())
        index = bisect_right(choices, cursor)
        if index == len(choices):
            forward = None
            break
        cursor = choices[index]
        forward.append(cursor)
    if forward is not None:
        reverse, cursor = [], len(cached)
        for key in reversed(keys):
            choices = positions[key]
            index = bisect_left(choices, cursor) - 1
            if index < 0:
                raise ValueError('Internal subsequence inconsistency')
            cursor = choices[index]
            reverse.append(cursor)
        reverse.reverse()
        if forward == reverse:
            return np.asarray(forward, dtype=np.int64), 'filtered_subsequence'
        raise ValueError('ambiguous_rounded_duplicate')
    if all(len(positions.get(key, ())) == 1 for key in keys) and len(set(keys)) == len(keys):
        return np.asarray([positions[key][0] for key in keys], dtype=np.int64), 'unique_reordered'
    if any(key not in positions for key in keys):
        raise ValueError('saved_pixel_absent_from_cache')
    raise ValueError('ambiguous_reordered_duplicate')


def _mutual_matches(previous, current):
    """Same per-camera matcher and gates as LamariaBabyTracker::match."""
    if len(previous) < 2 or len(current) < 2:
        return []
    matcher = cv2.BFMatcher(cv2.NORM_HAMMING)
    forward = matcher.knnMatch(previous, current, k=2)
    backward = matcher.match(current, previous)
    result = []
    for old_index, matches in enumerate(forward):
        if len(matches) < 2:
            continue
        best, second = matches
        new_index = best.trainIdx
        ratio_limit = float(np.float32(.8) * np.float32(second.distance))
        if (best.distance > 70. or best.distance >= ratio_limit or
                new_index < 0 or new_index >= len(backward) or
                backward[new_index].trainIdx != old_index):
            continue
        result.append((old_index, new_index))
    return result


class BabyFeatureOverlay:
    """Sequential visualization helper; call once per original input frame.

    Input observations match make_lamaria_output.keypoint_groups(): two lists
    of (map_point_id, u, v, tracked). Output IDs identify reconstructed passive
    tracks; they are not claimed to be native BabyTrack IDs or accepted inliers.
    """
    def __init__(self, run: Path):
        self.run = Path(run).resolve(strict=True)
        command_path = self.run / 'command.json'
        command = json.loads(command_path.read_text()).get('command', [])
        environment = {}
        for token in command:
            if isinstance(token, str) and token.startswith(('KP_DIR=', 'KP_DIR1=')):
                key, value = token.split('=', 1)
                environment[key] = value
        missing = [key for key in ('KP_DIR', 'KP_DIR1') if key not in environment]
        if missing:
            raise ValueError(f'Missing saved feature cache paths: {missing}')
        self.cache_dirs = [Path(environment[key]).resolve(strict=True)
                           for key in ('KP_DIR', 'KP_DIR1')]
        settings = cv2.FileStorage(str(self.run / 'config/settings.yaml'), cv2.FILE_STORAGE_READ)
        if not settings.isOpened():
            raise ValueError('Cannot read saved camera settings')
        width = int(settings.getNode('Camera.width').real())
        self.overlap = []
        for camera in (1, 2):
            begin = settings.getNode(f'Camera{camera}.overlappingBegin')
            end = settings.getNode(f'Camera{camera}.overlappingEnd')
            x0 = 0 if begin.empty() else int(begin.real())
            x1 = width if end.empty() or end.real() < 0 else int(end.real())
            self.overlap.append((x0, x1))
        settings.release()
        self.events_by_stamp = defaultdict(list)
        self.malformed_events = []
        decoder = json.JSONDecoder()
        with (self.run / 'run.log').open(errors='replace') as handle:
            for line_number, line in enumerate(handle, 1):
                marker = line.find('[BABY_DIAG]')
                if marker < 0:
                    continue
                try:
                    event, _ = decoder.raw_decode(line[marker+len('[BABY_DIAG]'):].lstrip())
                    stamp = round(float(event['time']) * 1e9)
                    event = dict(event, log_line=line_number)
                    self.events_by_stamp[stamp].append(event)
                except (ValueError, KeyError, TypeError) as error:
                    self.malformed_events.append({'line': line_number, 'error': str(error)})
        online = list(self.run.glob('online_*.csv'))
        if len(online) != 1:
            raise ValueError('Expected one saved online state trace')
        self.online_path = online[0]
        with self.online_path.open() as handle:
            self.online = {round(float(row['input_t_s']) * 1e9): row for row in csv.DictReader(handle)}
        self.event_stamps = sorted(self.events_by_stamp)
        self.online_stamps = sorted(self.online)
        self.previous = None
        self.last_stamp = None
        self.active = False
        self.next_id = 1
        self.counters = Counter()
        self.mapping_methods = Counter()
        self.issues = []
        self.pair_mismatches = []
        self.processed_event_lines = set()
        self.seconds = 0.
        # Matching 1,500-row descriptor sets should not spawn one worker per
        # host CPU. This renderer is sequential and benefits from bounded work.
        self.opencv_threads_before = cv2.getNumThreads()
        cv2.setNumThreads(min(4, max(1, self.opencv_threads_before)))

    @staticmethod
    def _lookup_stamp(stamp, available):
        position = bisect_left(available, stamp)
        candidates = available[max(0, position-1):position+1]
        if not candidates:
            return None
        nearest = min(candidates, key=lambda value: abs(value-stamp))
        return nearest if abs(nearest-stamp) <= 1000 else None

    def _record_issue(self, stamp, camera, reason):
        self.counters['unavailable_descriptor_camera_frames'] += 1
        if len(self.issues) < 30:
            self.issues.append({'timestamp_ns': int(stamp), 'camera': camera, 'reason': reason})

    def _load_camera(self, stamp, camera, observations):
        rows = np.asarray(observations, dtype=np.float64).reshape(-1, 4)
        result = _empty_camera()
        if not len(rows):
            result['descriptors'] = np.empty((0, 32), dtype=np.uint8)
            self.counters['empty_camera_frames'] += 1
            return result
        result['pixels'] = rows[:, 1:3].astype(np.float32)
        result['mapped'] = (rows[:, 0] >= 0) & (rows[:, 3] != 0)
        result['track_ids'] = np.arange(self.next_id, self.next_id+len(rows), dtype=np.int64)
        result['ages'] = np.ones(len(rows), dtype=np.int32)
        self.next_id += len(rows)
        path = self.cache_dirs[camera] / f'{int(stamp)}.kp'
        try:
            data = path.read_bytes()
            if len(data) < 4:
                raise ValueError('truncated_cache_header')
            count = struct.unpack_from('<i', data)[0]
            if count < 0 or count > 100000 or len(data) != 4+44*count:
                raise ValueError('invalid_cache_size')
            pixels = np.frombuffer(data, dtype='<f4', count=2*count, offset=4).reshape(count, 2)
            descriptors = np.frombuffer(data, dtype=np.uint8, count=32*count,
                                        offset=4+12*count).reshape(count, 32)
            x0, x1 = self.overlap[camera]
            outside = (x1 <= x0) | (pixels[:, 0] < x0) | (pixels[:, 0] > x1)
            order = np.r_[np.flatnonzero(outside), np.flatnonzero(~outside)]
            indices, method = _ordered_cache_indices(pixels[order], rows[:, 1:3])
            indices = order[indices]
            # These are exact cache pixels for the observed native rows. No
            # new lens inverse or inferred domain mask is used by the viewer.
            result['pixels'] = pixels[indices].copy()
            result['descriptors'] = np.ascontiguousarray(descriptors[indices])
            self.mapping_methods[method] += 1
            self.counters['descriptor_camera_frames'] += 1
        except (OSError, ValueError, struct.error) as error:
            # Matching a partial descriptor pool would change ratio/mutual
            # decisions. Disable this camera's links for this frame instead.
            result['descriptors'] = None
            self._record_issue(stamp, camera, str(error))
        return result

    def advance(self, stamp_ns: int, observations):
        started = time.perf_counter()
        stamp = int(stamp_ns)
        if self.last_stamp is not None and stamp <= self.last_stamp:
            raise ValueError('Baby overlay frames must advance strictly in time')
        if len(observations) != 2:
            raise ValueError('Baby overlay requires two camera observation lists')
        event_stamp = self._lookup_stamp(stamp, self.event_stamps)
        events = self.events_by_stamp.get(event_stamp, [])
        state_stamp = self._lookup_stamp(stamp, self.online_stamps)
        online = self.online.get(state_stamp, {})
        reset = any(event['event'] == 'reset' for event in events)
        jump = self.last_stamp is not None and stamp-self.last_stamp > 150_000_000
        if reset or jump:
            self.previous = None
            self.counters['logged_cache_resets' if reset else 'input_gap_resets'] += 1
        accepted_counts = None
        for event in events:
            self.processed_event_lines.add(event['log_line'])
            kind = event['event']
            if kind == 'enter':
                self.active = True
            elif kind in ('recovered', 'expired') or (kind == 'reset' and
                    event.get('reason') not in ('map_update', 'calibration_update')):
                self.active = False
            if kind == 'accepted':
                accepted_counts = [int(value) for value in event['camera_inliers']]
        cameras = [self._load_camera(stamp, camera, observations[camera]) for camera in (0, 1)]
        pairs = 0
        complete_descriptors = all(camera['descriptors'] is not None for camera in cameras)
        if self.previous is not None:
            complete_descriptors &= all(camera['descriptors'] is not None for camera in self.previous)
            for old, new in zip(self.previous, cameras):
                if old['descriptors'] is None or new['descriptors'] is None:
                    continue
                links = _mutual_matches(old['descriptors'], new['descriptors'])
                pairs += len(links)
                for previous_index, current_index in links:
                    new['track_ids'][current_index] = old['track_ids'][previous_index]
                    new['ages'][current_index] = old['ages'][previous_index]+1
        # reset events report the previous frame's pair count before clearing;
        # every other event reports current matching and can be compared.
        comparable = [event for event in events if event['event'] != 'reset' and 'tracks' in event]
        if comparable and complete_descriptors:
            expected = int(comparable[-1]['tracks'])
            self.counters['logged_pair_count_comparisons'] += 1
            if expected != pairs:
                self.counters['logged_pair_count_mismatches'] += 1
                if len(self.pair_mismatches) < 30:
                    self.pair_mismatches.append({'timestamp_ns': stamp, 'logged': expected,
                                                'reconstructed': pairs, 'event': comparable[-1]['event']})
        state = int(online.get('state', -1))
        pose_available = int(online.get('pose_available', 0)) != 0
        # Runtime snapshots are remembered only after initialized Track(); an
        # uninitialized input must not quietly seed passive optical history.
        self.previous = cameras if pose_available and state in (2, 3) else None
        if accepted_counts is not None and self.active:
            mode = 'baby'
            status = ('SOS: BabyFeatures + IMU accepted. Reconstructed passive tracks shown; '
                      f'actual accepted totals cam0={accepted_counts[0]}, cam1={accepted_counts[1]}. '
                      'Accepted point identities were not recorded.')
        elif self.active or state == 3:
            mode = 'imu_only'
            status = ('SOS: no accepted Baby update this frame; IMU fallback. '
                      'Reconstructed passive descriptor tracks are candidates only.')
        else:
            mode = 'normal'
            status = ('Normal SLAM. Baby descriptor tracks are passive and excluded from its estimator. '
                      'Displayed IDs are reconstructed visualization IDs, not saved solver inliers.')
        self.counters['frames'] += 1
        self.counters[f'mode_{mode}_frames'] += 1
        self.counters['reconstructed_pair_links'] += pairs
        self.counters['passive_features'] += sum(len(camera['pixels']) for camera in cameras)
        self.last_stamp = stamp
        self.seconds += time.perf_counter()-started
        # Strip internal descriptors while preserving them in self.previous.
        return {'cameras': [{key: value for key, value in camera.items() if key != 'descriptors'}
                            for camera in cameras], 'mode': mode, 'active': self.active,
                'accepted_counts': accepted_counts, 'events': events, 'status': status}

    def summary(self):
        return {'kind': 'Reconstructed passive BabyFeature descriptor tracks',
                'native_accepted_point_identities_available': False,
                'native_accepted_point_identities_inferred': False,
                'solver_replayed': False,
                'matching': {'norm': 'HAMMING', 'maximum_distance': 70,
                             'ratio': .8, 'mutual': True, 'same_camera_only': True},
                'domain_filter': 'Actual retained native keypoint dump; unique correspondence to cached pixels',
                'track_identity_scope': 'Visualization-only IDs reconstructed from passive descriptor links',
                'run': str(self.run), 'cache_directories': [str(path) for path in self.cache_dirs],
                'opencv_version': cv2.__version__, 'opencv_threads': cv2.getNumThreads(),
                'elapsed_overlay_seconds': self.seconds,
                'counts': dict(self.counters), 'descriptor_mapping_methods': dict(self.mapping_methods),
                'descriptor_mapping_issues_first30': self.issues,
                'pair_count_mismatches_first30': self.pair_mismatches,
                'malformed_events': self.malformed_events,
                'processed_event_count': len(self.processed_event_lines),
                'total_saved_event_count': sum(map(len, self.events_by_stamp.values()))}
