import copy
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
from hardware.rb5_gopro.interactive import capture_poses, next_sample_id


class InteractiveCaptureTests(unittest.TestCase):
    def test_resume_after_highest_id_without_reusing_deleted_gaps(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            self.assertEqual(next_sample_id(root), 'pose001')
            (root/'pose001.json').write_text('existing')
            (root/'pose003.json').write_text('existing')
            self.assertEqual(next_sample_id(root), 'pose004')
            (root/'pose004.json.tmp').write_text('unfinished write')
            self.assertEqual(next_sample_id(root), 'pose005')

    def test_enter_controls_capture_and_failure_does_not_advance_id(self):
        config = json.loads(Path('hardware/rb5.gopro.json').read_text())
        config = copy.deepcopy(config)
        config['robot']['kinematics']['model_matches_robot'] = True
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            (root/'pose001.json').write_text('original')
            seen = []

            def record(config, sample_id, output, duration):
                seen.append(sample_id)
                if len(seen) == 1:
                    raise TimeoutError('No feedback')
                Path(output).write_text('new')

            with patch('builtins.input', side_effect=['typo', '', '', '', 'q']), \
                    patch('hardware.rb5_gopro.interactive.record_pose', side_effect=record):
                capture_poses(config, root)
            self.assertEqual(seen, ['pose002', 'pose002', 'pose003'])
            self.assertEqual((root/'pose001.json').read_text(), 'original')


if __name__ == '__main__':
    unittest.main()
