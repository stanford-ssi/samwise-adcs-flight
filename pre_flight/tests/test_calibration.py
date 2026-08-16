import unittest

import numpy as np

from lib.calibration import (
    apply_magnetometer_calibration,
    fit_accelerometer_six_face,
    fit_gyro,
    fit_magnetometer,
)


class CalibrationTests(unittest.TestCase):
    def test_magnetometer_ellipsoid_recovery(self):
        rng = np.random.default_rng(7)
        directions = rng.normal(size=(3000, 3))
        directions /= np.linalg.norm(directions, axis=1, keepdims=True)
        center = np.array([7.0, -12.0, 4.5])
        transform = np.array(
            [
                [0.028, 0.001, -0.002],
                [0.001, 0.024, 0.0005],
                [-0.002, 0.0005, 0.031],
            ]
        )
        samples = center + (np.linalg.inv(transform) @ directions.T).T
        samples += rng.normal(scale=0.05, size=samples.shape)

        fit = fit_magnetometer(samples)
        corrected = apply_magnetometer_calibration(samples, fit.center, fit.transform)

        np.testing.assert_allclose(fit.center, center, atol=0.15)
        self.assertLess(np.std(np.linalg.norm(corrected, axis=1)), 0.01)
        self.assertTrue(fit.passed)

    def test_gyro_adds_residual_to_existing_offset(self):
        samples = np.tile([0.001, -0.002, 0.003], (100, 1))
        fit = fit_gyro(samples, np.array([0.01, 0.02, 0.03]))
        np.testing.assert_allclose(fit.new_offset, [0.011, 0.018, 0.033])
        self.assertFalse(fit.moving)

    def test_accelerometer_six_face(self):
        g = 0.00980665
        bias = np.array([0.0001, -0.0002, 0.00005])
        scale = np.array([1.01, 0.99, 1.02])
        means = {}
        for index, axis in enumerate("xyz"):
            plus = bias.copy()
            minus = bias.copy()
            plus[index] += scale[index] * g
            minus[index] -= scale[index] * g
            means[f"+{axis}"] = plus
            means[f"-{axis}"] = minus
        fit = fit_accelerometer_six_face(means)
        np.testing.assert_allclose(fit.bias, bias)
        np.testing.assert_allclose(fit.scale, scale)
        self.assertTrue(fit.passed)


if __name__ == "__main__":
    unittest.main()

