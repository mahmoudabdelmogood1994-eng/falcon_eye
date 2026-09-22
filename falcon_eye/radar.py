"""
radar.py
Simplified independent radar sensor model, used for the optional
sensor-fusion extension (spec section 9).

Radar is modeled with the complementary characteristic profile to the
LiDAR-like Falcon Eye sensor: coarser angular/range resolution and lower
peak detection probability, but essentially unaffected by fog (mmWave/
microwave radar propagation is not significantly attenuated by fog droplet
sizes, unlike optical wavelengths). This asymmetry is what makes fusion
potentially valuable and is intentional so the simulation can test the
sensor-fusion hypothesis honestly.
"""

from dataclasses import dataclass
from typing import Optional
import numpy as np

from .fog import FogScenario
from .sensor import SensorMeasurement


@dataclass
class RadarSensor:
    max_range_m: float = 12000.0
    field_of_view_rad: float = np.radians(50.0)
    measurement_frequency_hz: float = 5.0
    base_range_noise_std_m: float = 15.0
    base_angle_noise_std_rad: float = np.radians(1.5)
    base_detection_probability: float = 0.90
    base_false_detection_probability: float = 0.02
    processing_latency_s: float = 0.1
    sensor_failure_probability: float = 0.001
    # Radar is only weakly affected by fog (small residual sensitivity to
    # very dense fog / heavy precipitation captured via this coefficient).
    fog_sensitivity: float = 0.05

    rng: np.random.Generator = None

    def __post_init__(self):
        if self.rng is None:
            self.rng = np.random.default_rng()

    def dt(self) -> float:
        return 1.0 / self.measurement_frequency_hz

    def _in_fov(self, relative_angle: float) -> bool:
        return abs(relative_angle) <= (self.field_of_view_rad / 2.0)

    def measure(
        self,
        true_range: float,
        true_angle: float,
        true_relative_velocity: float,
        fog: FogScenario,
        extra_failure_probability: float = 0.0,
        extra_interference: float = 0.0,
    ) -> SensorMeasurement:
        total_failure_p = min(1.0, self.sensor_failure_probability + extra_failure_probability)
        if self.rng.random() < total_failure_p:
            return SensorMeasurement(
                target_detected=False, estimated_range=None, relative_angle=None,
                relative_velocity=None, confidence=0.0, sensor_failed=True,
                latency=self.processing_latency_s,
            )

        in_range = 0.0 < true_range <= self.max_range_m
        in_fov = self._in_fov(true_angle)

        # Fog causes only mild attenuation for radar wavelengths.
        fog_transmittance = 1.0 - self.fog_sensitivity * (1.0 - fog.two_way_transmittance(true_range))
        fog_transmittance = float(np.clip(fog_transmittance, 0.0, 1.0))

        if in_range and in_fov:
            range_falloff = max(0.0, 1.0 - (true_range / self.max_range_m) ** 2)
            p_detect = self.base_detection_probability * range_falloff * fog_transmittance
            p_detect *= (1.0 - min(0.9, extra_interference))
            p_detect = float(np.clip(p_detect, 0.0, 1.0))
        else:
            p_detect = 0.0

        detected = self.rng.random() < p_detect

        p_false = self.base_false_detection_probability * (1.0 + 2.0 * extra_interference)
        p_false = float(np.clip(p_false, 0.0, 0.5))
        false_detection = (not detected) and (self.rng.random() < p_false)

        if not detected and not false_detection:
            return SensorMeasurement(
                target_detected=False, estimated_range=None, relative_angle=None,
                relative_velocity=None, confidence=0.0,
                latency=self.processing_latency_s,
            )

        noise_scale = 1.0 + 2.0 * extra_interference
        range_noise_std = self.base_range_noise_std_m * noise_scale
        angle_noise_std = self.base_angle_noise_std_rad * noise_scale

        if false_detection:
            est_range = float(self.rng.uniform(0.0, self.max_range_m))
            est_angle = float(self.rng.uniform(-self.field_of_view_rad / 2,
                                                self.field_of_view_rad / 2))
            est_rel_vel = float(self.rng.uniform(-100.0, 100.0))
            confidence = float(self.rng.uniform(0.05, 0.3))
        else:
            est_range = max(0.0, float(true_range + self.rng.normal(0.0, range_noise_std)))
            est_angle = float(true_angle + self.rng.normal(0.0, angle_noise_std))
            vel_noise_std = 2.0 * noise_scale
            est_rel_vel = float(true_relative_velocity + self.rng.normal(0.0, vel_noise_std))
            confidence = float(np.clip(p_detect * fog_transmittance + 0.25, 0.0, 1.0))

        return SensorMeasurement(
            target_detected=True,
            estimated_range=est_range,
            relative_angle=est_angle,
            relative_velocity=est_rel_vel,
            confidence=confidence,
            is_false_detection=false_detection,
            latency=self.processing_latency_s,
        )
