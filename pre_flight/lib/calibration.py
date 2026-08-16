"""Calibration math and quality checks for pre-flight sensor captures."""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np


@dataclass(frozen=True)
class MagnetometerCalibration:
    center: np.ndarray
    transform: np.ndarray
    corrected_radius_mean: float
    corrected_radius_cv: float
    octants: int
    span_ratio: float
    retained_samples: int
    total_samples: int

    @property
    def passed(self) -> bool:
        return self.octants >= 7 and self.span_ratio >= 0.45 and self.corrected_radius_cv <= 0.10


def _ellipsoid_fit(data: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    if data.ndim != 2 or data.shape[1] != 3 or len(data) < 20:
        raise ValueError("Magnetometer fit requires at least 20 three-axis samples")

    x, y, z = data.T
    design = np.column_stack(
        (x * x, y * y, z * z, 2 * x * y, 2 * x * z, 2 * y * z, 2 * x, 2 * y, 2 * z, np.ones_like(x))
    )
    _, _, vh = np.linalg.svd(design, full_matrices=False)
    coefficients = vh[-1]
    quadratic = np.array(
        [
            [coefficients[0], coefficients[3], coefficients[4]],
            [coefficients[3], coefficients[1], coefficients[5]],
            [coefficients[4], coefficients[5], coefficients[2]],
        ]
    )
    linear = coefficients[6:9]
    constant = coefficients[9]

    if np.trace(quadratic) < 0:
        quadratic = -quadratic
        linear = -linear
        constant = -constant

    center = -np.linalg.solve(quadratic, linear)
    scale = float(center @ quadratic @ center - constant)
    if scale <= 0:
        raise ValueError("Calibration points do not form a bounded ellipsoid")
    normalized_quadratic = quadratic / scale
    eigenvalues, eigenvectors = np.linalg.eigh(normalized_quadratic)
    if np.any(eigenvalues <= 0):
        raise ValueError("Calibration fit is not positive definite; collect more orientations")
    transform = eigenvectors @ np.diag(np.sqrt(eigenvalues)) @ eigenvectors.T
    return center, transform


def apply_magnetometer_calibration(
    data: np.ndarray, center: np.ndarray, transform: np.ndarray
) -> np.ndarray:
    return (transform @ (data - center).T).T


def fit_magnetometer(samples: np.ndarray) -> MagnetometerCalibration:
    data = np.asarray(samples, dtype=float)
    center, transform = _ellipsoid_fit(data)
    corrected = apply_magnetometer_calibration(data, center, transform)
    radii = np.linalg.norm(corrected, axis=1)

    median = np.median(radii)
    mad = np.median(np.abs(radii - median))
    if mad > 0:
        mask = np.abs(radii - median) <= 4.5 * 1.4826 * mad
    else:
        mask = np.ones(len(data), dtype=bool)
    if np.count_nonzero(mask) >= max(50, int(0.7 * len(data))):
        center, transform = _ellipsoid_fit(data[mask])
    else:
        mask = np.ones(len(data), dtype=bool)

    retained = data[mask]
    corrected = apply_magnetometer_calibration(retained, center, transform)
    radii = np.linalg.norm(corrected, axis=1)
    centered = retained - center
    signs = centered >= 0
    octant_ids = signs[:, 0].astype(int) * 4 + signs[:, 1].astype(int) * 2 + signs[:, 2].astype(int)
    spans = np.ptp(retained, axis=0)
    span_ratio = float(np.min(spans) / np.max(spans)) if np.max(spans) > 0 else 0.0

    return MagnetometerCalibration(
        center=center,
        transform=transform,
        corrected_radius_mean=float(np.mean(radii)),
        corrected_radius_cv=float(np.std(radii) / np.mean(radii)),
        octants=len(np.unique(octant_ids)),
        span_ratio=span_ratio,
        retained_samples=int(np.count_nonzero(mask)),
        total_samples=len(data),
    )


@dataclass(frozen=True)
class GyroCalibration:
    observed_mean: np.ndarray
    observed_std: np.ndarray
    existing_offset: np.ndarray

    @property
    def new_offset(self) -> np.ndarray:
        return self.existing_offset + self.observed_mean

    @property
    def moving(self) -> bool:
        return bool(np.any(self.observed_std > 0.01))


def fit_gyro(samples: np.ndarray, existing_offset: np.ndarray | None = None) -> GyroCalibration:
    data = np.asarray(samples, dtype=float)
    if data.ndim != 2 or data.shape[1] != 3 or len(data) < 20:
        raise ValueError("Gyro calibration requires at least 20 three-axis samples")
    return GyroCalibration(
        observed_mean=np.mean(data, axis=0),
        observed_std=np.std(data, axis=0),
        existing_offset=np.zeros(3) if existing_offset is None else np.asarray(existing_offset, dtype=float),
    )


@dataclass(frozen=True)
class AccelerometerCalibration:
    bias: np.ndarray
    scale: np.ndarray
    cross_axis_ratio: float

    @property
    def passed(self) -> bool:
        return bool(np.all(self.scale > 0) and np.all(np.abs(self.scale - 1.0) < 0.15) and self.cross_axis_ratio < 0.15)


def fit_accelerometer_six_face(face_means: dict[str, np.ndarray], gravity_km_s2: float = 0.00980665) -> AccelerometerCalibration:
    axes = "xyz"
    bias = np.zeros(3)
    scale = np.zeros(3)
    cross_axis = []
    for index, axis in enumerate(axes):
        plus = np.asarray(face_means[f"+{axis}"], dtype=float)
        minus = np.asarray(face_means[f"-{axis}"], dtype=float)
        bias[index] = 0.5 * (plus[index] + minus[index])
        scale[index] = (plus[index] - minus[index]) / (2.0 * gravity_km_s2)
        cross_indices = [item for item in range(3) if item != index]
        cross_axis.extend(np.abs(plus[cross_indices]) / gravity_km_s2)
        cross_axis.extend(np.abs(minus[cross_indices]) / gravity_km_s2)
    return AccelerometerCalibration(
        bias=bias,
        scale=scale,
        cross_axis_ratio=float(np.max(cross_axis)),
    )
