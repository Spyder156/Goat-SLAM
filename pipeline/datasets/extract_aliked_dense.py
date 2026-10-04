#!/usr/bin/env python3
"""Denser ALIKED keypoint cache for the ORB-SLAM3 fork's external loader.

Same file format and the SAME fixed random-hyperplane binarization (seed 42,
128-d float -> 256 bits) as the original bench_basalt extract_aliked_seq.py, so
descriptors stay comparable with the inherited TH_LOW/TH_HIGH gates. Only the
detector budget changes: more keypoints and a lower detection threshold, aimed
at texture-poor / dark segments where the 1500-keypoint, 0.2-threshold cache
returns only 300-800 points (Long after 1550 s, Medium 770-778 s).

Per image <ns>.png -> <out>/<ns>.kp:
  int32 N; N*float32 x,y (stored image orientation); N*float32 response; N*32 uint8
"""
import argparse
import hashlib
import json
import time
from pathlib import Path

import cv2
import numpy as np
import torch
from lightglue import ALIKED


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--images', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--max-keypoints', type=int, default=3000)
    parser.add_argument('--detection-threshold', type=float, default=0.05)
    parser.add_argument('--nms-radius', type=int, default=2)
    args = parser.parse_args()
    if args.out.exists():
        raise FileExistsError(f'Preserving existing cache: {args.out}')
    args.out.mkdir(parents=True)
    dev = 'cuda'
    ext = ALIKED(max_num_keypoints=args.max_keypoints, detection_threshold=args.detection_threshold,
                 nms_radius=args.nms_radius).eval().to(dev)
    rng = np.random.default_rng(42)                      # identical projection to the original cache
    P = torch.from_numpy(rng.standard_normal((128, 256)).astype(np.float32)).to(dev)
    files = sorted(args.images.glob('*.png'))
    counts = []
    start = time.time()
    for k, f in enumerate(files):
        img = cv2.imread(str(f), 0)                      # stored orientation, no rotation
        t = torch.from_numpy(img)[None, None].float().to(dev) / 255.0
        with torch.no_grad():
            F = ext.extract(t.expand(1, 3, *img.shape))
            kps, desc, scores = F['keypoints'][0], F['descriptors'][0], F['keypoint_scores'][0]
            bits = (desc @ P > 0)
        n = len(kps)
        counts.append(n)
        packed = np.packbits(bits.cpu().numpy().astype(np.uint8), axis=1)
        with (args.out / (f.stem + '.kp')).open('wb') as fh:
            fh.write(np.int32(n).tobytes())
            fh.write(kps.cpu().numpy().astype(np.float32).tobytes())
            fh.write(scores.cpu().numpy().astype(np.float32).tobytes())
            fh.write(packed.tobytes())
        if k % 2000 == 0:
            print(f'  {k}/{len(files)} n={n} elapsed={time.time()-start:.0f}s', flush=True)
    counts = np.array(counts)
    (args.out / 'extraction_manifest.json').write_text(json.dumps({
        'images': str(args.images), 'frames': len(files), 'model': ext.conf.model_name if hasattr(ext, 'conf') else 'aliked-n16',
        'max_keypoints': args.max_keypoints, 'detection_threshold': args.detection_threshold, 'nms_radius': args.nms_radius,
        'binarization': 'random hyperplane seed 42, 128->256 bits (identical to extract_aliked_seq.py)',
        'keypoints_per_frame': {'median': float(np.median(counts)), 'p10': float(np.percentile(counts, 10)), 'min': int(counts.min()), 'max': int(counts.max())},
        'elapsed_s': time.time() - start, 'script_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
    }, indent=1) + '\n')
    print('DONE', args.out, 'median keypoints', np.median(counts), flush=True)


if __name__ == '__main__':
    main()
