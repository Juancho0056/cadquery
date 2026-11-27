import runpy
import unittest
from pathlib import Path

build = runpy.run_path(str(Path(__file__).resolve().parents[1] / 'mechanical_patterns/RET-001/model.py'))['build']

class RetentionTests(unittest.TestCase):
    def test_clearances(self):
        for clearance in (0.2, 0.3, 0.4):
            with self.subTest(clearance=clearance):
                parts = build(clearance)
                for part in parts.values():
                    self.assertTrue(part.val().isValid())
                    self.assertGreater(part.val().Volume(), 0)
                for name in ('shell_a', 'shell_b'):
                    self.assertLess(parts['shaft'].intersect(parts[name]).val().Volume(), 1e-6)

if __name__ == '__main__':
    unittest.main()
