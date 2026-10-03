# Project storage and separation

Updated 2026-10-03 after the user requested removal of this project's artifacts
from the original INSV_STITCHING tree.

## Current locations

Source, patches, configs and private builds:

```text
/home/raghav/workspace/MeckaAI/Raghavs_ORB-SLAM3
```

Dedicated experiment storage, exposed by the source folder's `experiments/` link:

```text
/media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments
```

The artifacts occupy approximately 187 GiB, so they stay on the hard drive in
their own project namespace rather than consuming the smaller workspace drive.
There is no shared experiment root or forwarding link in the original project.
Only the 58 independently inventoried `lamaria*` experiment directories belonging
to this project were moved. Unrelated user experiments and the original source
were not changed.

## What was preserved and checked

All 13,434 regular files retained their device/inode, size, modification time and
mode across the same-filesystem move. All 14 saved official score JSON hashes
were unchanged. Cross-experiment absolute symlinks were rebased, previously
working symlinks still resolved, and none of the moved artifact links resolved
back into the original tree. The new source folder's `experiments/` symlink was
atomically changed to the dedicated location.

Completed scores, execution manifests, source snapshots, logs and RRD contents
remain historical evidence. Their embedded old paths describe where the runs
actually executed; they are not a request to reopen storage in the old project.
Runtime path resolution uses the explicit relocation mapping. Do not rewrite
historical hashes merely to make their strings look current.

Current project documentation uses the new locations. Original documentation
copies and checksums are retained in the relocation audit. The live path index
is:

```text
/home/raghav/workspace/MeckaAI/Raghavs_ORB-SLAM3/maintenance/relocation_20261003/VISUALIZATIONS.txt
```

Best completed Short Rerun, Score2D 80.20:

```text
/media/raghav/HardDrive1/MeckaAI/Raghavs_ORB-SLAM3/experiments/lamaria_online_full_v2_short_full_20261003/online_full_evaluation.rrd
```

## External dependencies that are not the original project

LaMAria input data and feature caches are under
`/media/raghav/HardDrive1/Mecka/lamaria`. The Python environment, Docker runtime,
and the unchanged official evaluation toolkit are external tools. Moving
artifacts does not make this a hermetic container or copy the entire dataset.
The current source and artifact pipeline must not read source, data, or output
from the original INSV_STITCHING project. Legacy runners require their own
explicit input-data path rather than falling back to that original project.

## Audit

```text
/home/raghav/workspace/MeckaAI/Raghavs_ORB-SLAM3/maintenance/relocation_20261003/manifest.json
/home/raghav/workspace/MeckaAI/Raghavs_ORB-SLAM3/maintenance/relocation_20261003/regular_files_before.json
/home/raghav/workspace/MeckaAI/Raghavs_ORB-SLAM3/maintenance/relocation_20261003/symlinks_before.json
/home/raghav/workspace/MeckaAI/Raghavs_ORB-SLAM3/maintenance/relocation_20261003/documentation_updates.json
```

No SLAM experiment or previously stopped calibration recording is restarted by
this migration. The calibrated offline recording remains explicitly partial.
