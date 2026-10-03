"""Clock and state-history safeguards for cached experiment planning."""
import unittest

from lamaria_suite import choose_timestamps


class LamariaSuiteTest(unittest.TestCase):
    def setUp(self):
        self.start = 1_700_000_000_000_000_123
        self.times = [self.start + i * 1_000_000_000 for i in range(200)]
        self.manifest = {"startup_seconds": 30, "event_context_before_seconds": 20,
                         "event_context_after_seconds": 10}
        self.entry = {"events": [{"name": "loss", "start_ns": self.times[100],
                                   "end_ns": self.times[105], "kind": "map_loss"}]}

    def test_context_keeps_native_ns_but_prefix_replays_all_prior_inputs(self):
        context, context_note = choose_timestamps(self.times, "loss-context", self.manifest, self.entry)
        prefix, prefix_note = choose_timestamps(self.times, "loss-prefix", self.manifest, self.entry)
        self.assertEqual(context, self.times[80:116])
        self.assertEqual(prefix, self.times[:116])
        self.assertEqual(context[0] % 1_000_000_000, 123)
        self.assertIn("COLD START", context_note["interpretation"])
        self.assertFalse(context_note["starts_at_original_sequence_start"])
        self.assertTrue(prefix_note["starts_at_original_sequence_start"])
        self.assertFalse(context_note["loop_evaluation_allowed"])
        self.assertFalse(prefix_note["full_sequence_scoring_allowed"])

    def test_only_full_allows_benchmark_and_loop_evaluation(self):
        startup, note = choose_timestamps(self.times, "startup", self.manifest, self.entry)
        self.assertEqual(startup, self.times[:31])
        self.assertFalse(note["full_sequence_scoring_allowed"])
        full, note = choose_timestamps(self.times, "full", self.manifest, self.entry)
        self.assertEqual(full, self.times)
        self.assertTrue(note["full_sequence_scoring_allowed"])
        self.assertTrue(note["loop_evaluation_allowed"])

    def test_unknown_event_does_not_silently_select_another(self):
        with self.assertRaisesRegex(ValueError, "Unknown event"):
            choose_timestamps(self.times, "loss-prefix", self.manifest, self.entry, "wrong-event")


if __name__ == "__main__":
    unittest.main()
