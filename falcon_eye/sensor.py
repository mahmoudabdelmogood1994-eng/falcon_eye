"""
sensor.py
Falcon Eye forward-looking LiDAR-like sensor model (research/simulation
abstraction, not a real laser hardware implementation, per spec section 2).

The sensor is deliberately modeled at the "measurement statistics" level:
it does not simulate individual photons, but produces detection
probabilities, range/angle noise, false-return rates, latency and
intermittent hardware failure, all of which are configurable and all of
which respond to fog attenuation.
"""

from dataclasses import dataclass
from typing import Optional
import numpy as np

from .fog import FogScenario


@dataclass
class SensorMeasurement:
    target_detected: bool
    estimated_range: Optional[float]     # meters
    relative_angle: Optional[float]      # radians, 0 = boresight
    relative_velocity: Optional[float]   # m/s, closing positive
    confidence: float                    # 0..1
    is_false_detection: bool = False
    latency: float = 0.0                 # seconds, measurement-to-available delay
    sensor_failed: bool = False


@dataclass
class FalconEyeSensor:
    max_range_m: float = 9000.0          # maximum sensing range
    field_of_view_rad: float = np.radians(60.0)  # full FOV
    measurement_frequency_hz: float = 10.0
    range_resolution_m: float = 5.0
    base_range_noise_std_m: float = 3.0
    base_angle_noise_std_rad: float = np.radians(0.3)
    base_detection_probability: float = 0.98   # in clear air, in-range, in-FOV
    base_false_detection_probability: float = 0.01  # per measurement cycle
    processing_latency_s: float = 0.05
    sensor_failure_probability: float = 0.001   # per measurement cycle (dropout)

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
        """Produce one simulated measurement cycle.

        extra_failure_probability: additional injected hardware failure
            probability (failure-injection module hook).
        extra_interference: additional [0..1] scaling of noise/false-return
            rates, e.g. from data-link or EM interference injection.
        """

        # --- Hardware dropout (independent of fog) ---
        total_failure_p = min(1.0, self.sensor_failure_probability + extra_failure_probability)
        if self.rng.random() < total_failure_p:
            return SensorMeasurement(
                target_detected=False, estimated_range=None, relative_angle=None,
                relative_velocity=None, confidence=0.0, sensor_failed=True,
                latency=self.processing_latency_s,
            )

        # --- Geometric visibility gate ---
        in_range = 0.0 < true_range <= self.max_range_m
        in_fov = self._in_fov(true_angle)

        # --- Atmospheric transmittance from fog model ---
        transmittance = fog.two_way_transmittance(true_range) if in_range else 0.0

        # --- Detection probability model ---
        # Detection probability degrades with range (inverse-square-ish
        # signal falloff, approximated) and multiplies by atmospheric
        # transmittance. This is what prevents the sim from assuming fog
        # is "free" for LiDAR.
        if in_range and in_fov:
            range_falloff = (1.0 - (true_range / self.max_range_m) ** 2)
            range_falloff = max(0.0, range_falloff)
            p_detect = self.base_detection_probability * range_falloff * transmittance
            p_detect *= (1.0 - min(0.9, extra_interference))
            p_detect = float(np.clip(p_detect, 0.0, 1.0))
        else:
            p_detect = 0.0

        detected = self.rng.random() < p_detect

        # --- False detection (can occur even without a real target in
        # range/FOV; more likely as fog scatter and interference rise) ---
        p_false = self.base_false_detection_probability * fog.false_return_multiplier
        p_false *= (1.0 + 3.0 * extra_interference)
        p_false = float(np.clip(p_false, 0.0, 0.5))
        false_detection = (not detected) and (self.rng.random() < p_false)

        if not detected and not false_detection:
            return SensorMeasurement(
                target_detected=False, estimated_range=None, relative_angle=None,
                relative_velocity=None, confidence=0.0,
                latency=self.processing_latency_s,
            )

        # --- Noise model, inflated by fog scattering and interference ---
        noise_scale = fog.noise_multiplier * (1.0 + 2.0 * extra_interference)
        range_noise_std = max(self.range_resolution_m / 2.0,
                               self.base_range_noise_std_m * noise_scale)
        angle_noise_std = self.base_angle_noise_std_rad * noise_scale

        if false_detection:
            # False returns are effectively random within sensing envelope
            est_range = float(self.rng.uniform(0.0, self.max_range_m))
            est_angle = float(self.rng.uniform(-self.field_of_view_rad / 2,
                                                self.field_of_view_rad / 2))
            est_rel_vel = float(self.rng.uniform(-100.0, 100.0))
            confidence = float(self.rng.uniform(0.05, 0.35))
        else:
            est_range = float(true_range + self.rng.normal(0.0, range_noise_std))
            est_range = max(0.0, est_range)
            est_angle = float(true_angle + self.rng.normal(0.0, angle_noise_std))
            vel_noise_std = 1.0 * noise_scale
            est_rel_vel = float(true_relative_velocity + self.rng.normal(0.0, vel_noise_std))
            # Confidence combines detection probability and transmittance
            confidence = float(np.clip(p_detect * transmittance + 0.2, 0.0, 1.0))

        return SensorMeasurement(
            target_detected=True,
            estimated_range=est_range,
            relative_angle=est_angle,
            relative_velocity=est_rel_vel,
            confidence=confidence,
            is_false_detection=false_detection,
            latency=self.processing_latency_s,
        )
