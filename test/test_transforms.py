import unittest

import numpy as np

from vive_tracker.driver import _matrix_quaternion, _quaternion_matrix, _transform_matrix


class TransformTest(unittest.TestCase):
    def test_identity_quaternion(self):
        matrix = _quaternion_matrix((0.0, 0.0, 0.0, 1.0))
        np.testing.assert_allclose(matrix, np.eye(4))
        self.assertEqual(_matrix_quaternion(matrix), (0.0, 0.0, 0.0, 1.0))

    def test_translation_and_rotation_matrix(self):
        matrix = _transform_matrix((0.172, 0.0, -0.076), (0.0, 0.0, 90.0))
        np.testing.assert_allclose(matrix[:3, 3], (0.172, 0.0, -0.076))
        np.testing.assert_allclose(matrix[:3, :3] @ matrix[:3, :3].T, np.eye(3), atol=1e-7)


if __name__ == "__main__":
    unittest.main()
