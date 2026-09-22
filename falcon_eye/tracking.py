"""
tracking.py
Lightweight target tracking and multi-sensor fusion.

Track: a simple alpha-beta (constant-velocity) filter over range, so a
single noisy/intermittent sensor stream produces a smoothed range/velocity
estimate instead of a raw per-cycle measurement. This is intentionally
simple (not a full Kalman filter) to keep the first prototype easy to
validate, per spec section 14's "start simple" guidance.

Fusion: combines LiDAR-like and radar measurements at a given time step
using confidence-weighted averaging when both report a detection, and
falls back to whichever sensor has a detection when only one does. This
directly supports the spec section 9 LiDAR-only vs Radar-only vs Fusion
comparison.
"""

from dataclasses import dataclass, field
from typing import Optional
import numpy as np

from .sensor import SensorMeasurement


@dataclass
class AlphaBetaTrack:
    alpha: float = 0.6
    beta: float = 0.3
    range_est: Optional[float] = None
    range_rate_est: float = 0.0
    angle_est: Optional[float] = None
    initialized: bool = False
    misses_in_a_row: int = 0
    max_coast_misses: int = 5  # keep predicting through short dropouts

    def update(self, meas: Optional[SensorMeasurement], dt: float):
        """meas=None or non-detected measurement -> coast (predict only)."""
        detected = meas is not None and meas.target_detected

        if not self.initialized:
            if detected:
                self.range_est = meas.estimated_range
                self.angle_est = meas.relative_angle
                self.range_rate_est = -(meas.relative_velocity or 0.0)
                self.initialized = True
                self.misses_in_a_row = 0
            return

        # Predict
        pred_range = self.range_est + self.range_rate_est * dt if self.range_est is not None else None

        if detected and pred_range is not None:
            residual = meas.estimated_range - pred_range
            self.range_est = pred_range + self.alpha * residual
            # Rate estimate: low-pass filter the sensor's own directly-measured
            # relative_velocity rather than differencing noisy range residuals
            # over a fixed dt. Differencing (beta*residual/dt) amplifies range
            # noise by 1/dt and produces large, spurious rate swings under
            # realistic sensor noise -- exactly the kind of instability that
            # can trip a hard TTC-based reflex threshold on a target that
            # isn't actually critical. The sensor's relative_velocity channel
            # has much lower noise than a range difference, so filtering it
            # directly is both simpler and materially more stable.
            measured_range_rate = -(meas.relative_velocity or 0.0)
            self.range_rate_est = (1.0 - self.beta) * self.range_rate_est + self.beta * measured_range_rate
            self.angle_est = meas.relative_angle
            self.misses_in_a_row = 0
        else:
            self.misses_in_a_row += 1
            if self.misses_in_a_row <= self.max_coast_misses:
                self.range_est = pred_range
            else:
                # lost track
                self.initialized = False
                self.range_est = None
                self.angle_est = None
                self.range_rate_est = 0.0

    def closing_velocity(self) -> float:
        return -self.range_rate_est

    def is_valid(self) -> bool:
        return self.initialized and self.range_est is not None


def fuse_measurements(
    lidar_meas: Optional[SensorMeasurement],
    radar_meas: Optional[SensorMeasurement],
) -> Optional[SensorMeasurement]:
    """Confidence-weighted fusion of two independent sensor measurements
    taken at (approximately) the same time step."""
    l_ok = lidar_meas is not None and lidar_meas.target_detected
    r_ok = radar_meas is not None and radar_meas.target_detected

    if not l_ok and not r_ok:
        return None

    if l_ok and not r_ok:
        return lidar_meas
    if r_ok and not l_ok:
        return radar_meas

    # Both report a detection: confidence-weighted combination.
    wl = max(lidar_meas.confidence, 1e-3)
    wr = max(radar_meas.confidence, 1e-3)
    total = wl + wr

    fused_range = (lidar_meas.estimated_range * wl + radar_meas.estimated_range * wr) / total
    fused_angle = (lidar_meas.relative_angle * wl + radar_meas.relative_angle * wr) / total
    fused_vel = (lidar_meas.relative_velocity * wl + radar_meas.relative_velocity * wr) / total
    fused_confidence = min(1.0, (wl + wr) / 2.0 + 0.15)  # agreement boosts confidence
    fused_false = lidar_meas.is_false_detection and radar_meas.is_false_detection

    return SensorMeasurement(
        target_detected=True,
        estimated_range=fused_range,
        relative_angle=fused_angle,
        relative_velocity=fused_vel,
        confidence=fused_confidence,
        is_false_detection=fused_false,
        latency=max(lidar_meas.latency, radar_meas.latency),
    )
