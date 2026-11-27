import runpy
import unittest
from pathlib import Path

build = runpy.run_path(str(Path(__file__).resolve().parents[1] / 'mechanical_patterns/ASM-001/model.py'))['build']


class SnapFitTests(unittest.TestCase):
    def test_assembled_geometry(self):
        for c in (0.2, 0.3, 0.4):
            with self.subTest(clearance=c):
                p = build(c)
                for part in p.values():
                    self.assertTrue(part.val().isValid())
                    self.assertEqual(len(part.solids().vals()), 1)
                self.assertLess(p['body'].intersect(p['lid']).val().Volume(), 1e-6)
                self.assertGreater(p['body'].intersect(p['lid'].translate((0,0,1))).val().Volume(), 0.1)

    def test_invalid_parameters(self):
        for kwargs in ({'clearance':0}, {'finger_thickness':0}, {'engagement':0}):
            with self.assertRaises(ValueError):
                build(**kwargs)


if __name__ == '__main__':
    unittest.main()
