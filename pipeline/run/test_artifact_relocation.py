"""Archived results remain immutable and cannot be confused with another run."""
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from artifact_paths import EXPERIMENT_ROOT, FORMER_EXPERIMENT_ROOTS, relocated_path, relocated_record
from lamaria_colmap_refinement import command


class RelocationContract(unittest.TestCase):
    def test_cached_success_survives_relocation_without_execution_or_rewrite(self):
        old = str(FORMER_EXPERIMENT_ROOTS[0] / 'example/model')
        new = str(EXPERIMENT_ROOT / 'example/model')
        with tempfile.TemporaryDirectory() as temporary:
            folder = Path(temporary)
            marker = folder/'triangulate.result.json'
            record = {'returncode': 0, 'command': ['colmap', '--input_path', old]}
            marker.write_text(json.dumps(record, indent=2)+'\n')
            original = marker.read_bytes()
            with patch('lamaria_colmap_refinement.subprocess.call') as execute:
                command(folder, 'triangulate', ['colmap', '--input_path', new])
                execute.assert_not_called()
            self.assertEqual(marker.read_bytes(), original)

    def test_scientific_hash_changes_and_distinct_runs_still_fail_equality(self):
        old = str(FORMER_EXPERIMENT_ROOTS[0] / 'baseline/points.bin')
        new = str(EXPERIMENT_ROOT / 'baseline/points.bin')
        original = {'files_sha256': {old: 'original_hash'}}
        same = {'files_sha256': {new: 'original_hash'}}
        changed = {'files_sha256': {new: 'changed_hash'}}
        self.assertEqual(relocated_record(original), same)
        self.assertNotEqual(relocated_record(original), changed)
        self.assertNotEqual(relocated_path(old), relocated_path(old.replace('/baseline/', '/candidate/')))
        self.assertIn(old, original['files_sha256'])

    def test_unrelated_inputs_and_similar_directory_names_are_untouched(self):
        for value in ('/media/raghav/HardDrive1/Mecka/lamaria/gt_dense.txt',
                      str(FORMER_EXPERIMENT_ROOTS[0])+'_other/test',
                      '/home/raghav/workspace/INSV_STITCHING/SLAM/src/a.cc'):
            self.assertEqual(relocated_path(value), Path(value))
        with self.assertRaisesRegex(ValueError, 'merge distinct'):
            relocated_record({str(FORMER_EXPERIMENT_ROOTS[0]/'a'): 1,
                              str(EXPERIMENT_ROOT/'a'): 2})


if __name__ == '__main__':
    unittest.main()
