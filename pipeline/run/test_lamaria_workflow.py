"""Focused checks for build provenance, input clock and safe run isolation."""
import argparse
import contextlib
import csv
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
from subprocess import CompletedProcess

import numpy as np

from build_lamaria import rewrite_link, sha256, verify_baseline
from run_lamaria import run, timestamp_list
from evaluate_lamaria_coverage import summarize


class LamariaWorkflowTest(unittest.TestCase):
    def test_skip_viz_runs_estimator_and_coverage_and_preserves_failures(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            build = root / "build"
            vendor = root / "vendor"
            vendor.mkdir()
            for relative in ("lib/libORB_SLAM3.so", "bin/stereo_lamaria_euroc"):
                p = build / relative
                p.parent.mkdir(parents=True)
                p.write_bytes(b"fixture")
            (build / "build_manifest.json").write_text(json.dumps({
                "library_sha256": sha256(build / "lib/libORB_SLAM3.so"),
                "runner_sha256": sha256(build / "bin/stereo_lamaria_euroc"),
                "image": "local:test", "image_id": "fixture", "vendor": str(vendor)}))
            timestamps = root / "timestamps.txt"
            timestamps.write_text("1000000000\n1050000000\n")
            dataset = root / "dataset"
            for camera in (0, 1):
                directory = dataset / f"mav0/cam{camera}/data"
                directory.mkdir(parents=True)
                for stamp in (1000000000, 1050000000):
                    (directory / f"{stamp}.png").touch()
            kp = root / "kp"
            kp.mkdir()
            config = root / "settings.yaml"
            config.write_text('Camera.type: "Fisheye624"\nRig.non_overlapping: 0\n')
            imu = root / "imu.csv"
            imu.write_text("factory fixture\n")
            imu.with_suffix(".csv.manifest.json").write_text(json.dumps({
                "output_sha256": sha256(imu), "ground_truth_used": False,
                "imu_label": "imu-right", "first_timestamp_ns": 999999999,
                "last_timestamp_ns": 1050000001}))
            for i, (estimator_code, raw_present, coverage_code, expected) in enumerate(
                    [(0, True, 0, 0), (7, True, 0, 7), (0, False, 0, 1), (0, True, 9, 9)]):
                with self.subTest(estimator=estimator_code, raw=raw_present, coverage=coverage_code):
                    out = root / f"run-{i}"
                    args = argparse.Namespace(out=out, timestamps=timestamps, config=config,
                        build=build, vendor=vendor, dataset=dataset, kp0=kp, kp1=kp,
                        calibrated_imu=imu, imu_manifest=None, vrs=None, raw_imu=None,
                        imu_python=Path("python3"), viz_python=Path("python3"), gt=None,
                        image="local:test", cpus=4, timeout=0, dry_run=False,
                        temporal_associations=True, skip_viz=True)
                    called = []

                    def execute(command, **kwargs):
                        called.append(command)
                        if command[0] == "docker":
                            if raw_present:
                                (out / f"f_{out.name}.txt").write_text("1000000000 0 0 0 0 0 0 1\n")
                            return CompletedProcess(command, estimator_code)
                        self.assertIn("evaluate_lamaria_coverage.py", command[2])
                        (out / "coverage.json").write_text(json.dumps({
                            "exported_pose_fraction": 1., "independent_segments_exported": 1,
                            "completed_active_map_resets": 0}))
                        return CompletedProcess(command, coverage_code)

                    with patch("run_lamaria.subprocess.check_output", return_value="fixture"), \
                         patch("run_lamaria.subprocess.run", side_effect=execute), \
                         contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
                        self.assertEqual(run(args), expected)
                    self.assertEqual(len(called), 2)
                    status = json.loads((out / "result.json").read_text())
                    self.assertTrue(status["visualization_skipped"])
                    self.assertIsNone(status["export_returncode"])
                    self.assertEqual(status["raw_export_returncode"], int(not raw_present))
                    self.assertEqual(status["returncode"], estimator_code)
                    self.assertEqual(status["coverage_returncode"], coverage_code)

    def test_reused_map_id_does_not_hide_reset_or_claim_lost_pose_coverage(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            native = np.arange(6) * .05 + 1.
            (root / "run.log").write_text("LM: Active map reset, Done!!!\n")
            with (root / "online_fixture.csv").open("w") as handle:
                writer = csv.writer(handle)
                writer.writerow(["input_t_s", "pose_available", "state", "coasting", "map_id", "map_init_kf_id"])
                for i, stamp in enumerate(native):
                    writer.writerow([stamp, int(i > 0), 1 if i == 0 else 3 if i == 2 else 2,
                                     int(i == 2), 0, 0 if i < 3 else 10])
            with (root / "atlas_fixture_trajectory.csv").open("w") as handle:
                writer = csv.writer(handle)
                writer.writerow(["t_s", "coordinate_frame", "map_id", "map_init_kf_id"])
                for stamp in native[3:]:
                    writer.writerow([stamp, "map0_init10", 0, 10])
            result = summarize(root, native)
            self.assertEqual(result["completed_active_map_resets"], 1)
            self.assertEqual(len(result["online_map_epochs"]), 2)
            self.assertEqual(result["independent_segments_exported"], 1)
            self.assertEqual(result["exported_pose_fraction"], .5)
            self.assertEqual(result["logged_visual_ok_frames"], 4)
            self.assertEqual(result["logged_coasted_frames"], 1)
            self.assertEqual(result["logged_visual_ok_without_final_export"], 1)
            self.assertTrue(result["online_pose_rows_cover_every_input"])

    def test_link_replaces_optimizer_and_preserves_baseline_objects(self):
        command = ["/usr/bin/c++", "-shared", "-o", "../lib/libORB_SLAM3.so",
                   "CMakeFiles/ORB_SLAM3.dir/src/Optimizer.cc.o",
                   "CMakeFiles/ORB_SLAM3.dir/src/System.cc.o",
                   "../Thirdparty/g2o/lib/libg2o.so"]
        replacement = {command[4]: "/work/objects/src_Optimizer.cc.o"}
        result = rewrite_link(command, replacement, "/work/lib/libORB_SLAM3.so")
        self.assertIn("/work/objects/src_Optimizer.cc.o", result)
        self.assertNotIn("/orb/build/" + command[4], result)
        self.assertIn("/orb/build/CMakeFiles/ORB_SLAM3.dir/src/System.cc.o", result)
        self.assertIn("/orb/Thirdparty/g2o/lib/libg2o.so", result)

    def test_baseline_rejects_source_changed_after_packaging(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            source = root / "source.cc"
            source.write_text("recorded baseline\n")
            manifest = root / "manifest.json"
            manifest.write_text(json.dumps({"files": {source.name: sha256(source)}}))
            verify_baseline(root, manifest)
            source.write_text("unreviewed replacement\n")
            with self.assertRaisesRegex(ValueError, "differs from the tested baseline"):
                verify_baseline(root, manifest)

    def test_dry_run_keeps_large_ns_exact_and_writes_nothing(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            build = root / "build"
            for relative in ("lib/libORB_SLAM3.so", "bin/stereo_lamaria_euroc"):
                path = build / relative
                path.parent.mkdir(parents=True)
                path.write_bytes(b"fixture")
            vendor = root / "vendor"
            vendor.mkdir()
            manifest = {"library_sha256": sha256(build / "lib/libORB_SLAM3.so"),
                        "runner_sha256": sha256(build / "bin/stereo_lamaria_euroc"),
                        "image": "local:test", "image_id": "fixture", "vendor": str(vendor)}
            (build / "build_manifest.json").write_text(json.dumps(manifest))
            stamps = [1_700_000_000_000_000_123, 1_700_000_000_050_000_124]
            timestamps = root / "timestamps.txt"
            timestamps.write_text("# ns, not seconds\n" + "\n".join(map(str, stamps)) + "\n")
            self.assertEqual(timestamp_list(timestamps), stamps)
            dataset = root / "dataset"
            originals = root / "original-jpg"
            originals.mkdir()
            for camera in (0, 1):
                images = dataset / f"mav0/cam{camera}/data"
                images.mkdir(parents=True)
                for stamp in stamps:
                    original = originals / f"{camera}-{stamp}.jpg"
                    original.touch()
                    (images / f"{stamp}.png").symlink_to(original)
            kp = root / "kp"
            kp.mkdir()
            config = root / "settings.yaml"
            config.write_text('Camera.type: "Fisheye624"\nRig.non_overlapping: 0\n')
            imu = root / "factory.csv"
            imu.write_text("# fixture\n")
            (root / "factory.csv.manifest.json").write_text(json.dumps({
                "output_sha256": sha256(imu), "ground_truth_used": False,
                "imu_label": "imu-right", "first_timestamp_ns": stamps[0]-1,
                "last_timestamp_ns": stamps[-1]+1}))
            args = argparse.Namespace(out=root / "new-run", timestamps=timestamps, config=config,
                build=build, vendor=vendor, dataset=dataset, kp0=kp, kp1=kp,
                calibrated_imu=imu, imu_manifest=None, vrs=None, raw_imu=None,
                imu_python=Path("python3"), viz_python=Path("python3"), gt=root / "gt.txt",
                image="local:test", cpus=4, timeout=0, dry_run=True, temporal_associations=True)
            with contextlib.redirect_stdout(io.StringIO()) as output:
                self.assertEqual(run(args), 0)
            self.assertFalse(args.out.exists())
            plan = json.loads(output.getvalue())
            self.assertEqual(plan["first_timestamp_ns"], stamps[0])
            self.assertAlmostEqual(plan["expected_duration_s"], .050000001)
            self.assertNotIn(str(args.gt), plan["command"])
            self.assertNotIn(str(args.gt), plan["coverage_command"])
            self.assertIn(str(args.out / "coverage.json"), plan["coverage_command"])
            self.assertIn(str(args.gt), plan["export_command"])
            self.assertFalse(plan["ground_truth_used_by_estimator"])
            self.assertEqual(plan["command"].count("LAMARIA_TEMPORAL_ASSOCIATIONS=1"), 1)
            self.assertIn(str(vendor)+":/orb:ro", plan["command"])
            self.assertIn(str(originals)+":"+str(originals)+":ro", plan["command"])
            self.assertFalse(plan["match_diagnostics"])
            self.assertFalse(any("LAMARIA_MATCH_DIAG" in arg for arg in plan["command"]))
            args.match_diagnostics = True
            with contextlib.redirect_stdout(io.StringIO()) as output:
                self.assertEqual(run(args), 0)
            diagnostic = json.loads(output.getvalue())
            self.assertTrue(diagnostic["match_diagnostics"])
            self.assertEqual(diagnostic["command"].count("LAMARIA_MATCH_DIAGNOSTICS=1"), 1)
            self.assertEqual(diagnostic["command"].count("LAMARIA_MATCH_DIAG_EVERY=1"), 1)
            without_logging = diagnostic["command"].copy()
            for value in ("LAMARIA_MATCH_DIAGNOSTICS=1", "LAMARIA_MATCH_DIAG_EVERY=1"):
                index = without_logging.index(value)
                self.assertEqual(without_logging[index-1], "-e")
                del without_logging[index-1:index+1]
            self.assertEqual(without_logging, plan["command"])


if __name__ == "__main__":
    unittest.main()
