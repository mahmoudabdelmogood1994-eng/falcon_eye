"""
failures.py
Failure-injection module (spec section 8).

Provides configurable, time-varying fault injection that layers on top of
the sensor's own baseline failure/noise model:

  - no_detection: forces zero detection probability for a window
  - intermittent: sensor dropout that toggles on/off periodically
  - biased_range: adds a systematic (not zero-mean) range error
  - false_detection_burst: temporarily elevated false-return rate
  - delayed_measurement: extra latency injected
  - datalink_dropout: measurement is produced but "lost" before reaching
    the collision logic (models a comms/data-link failure downstream of
    the sensor itself)
  - interference: elevated extra_interference passed into sensor.measure()
  - multi_target_confusion: handled at the scenario level (see scenarios.py)
"""

from dataclasses import dataclass, field
from typing import Optional
import numpy as np

from .sensor import SensorMeasurement


@dataclass
class FailureConfig:
    mode: str = "none"  # none | no_detection | intermittent | biased_range |
                          # false_detection_burst | delayed_measurement |
                          # datalink_dropout | interference
    start_time_s: float = 0.0
    end_time_s: float = 1e9
    # mode-specific parameters
    intermittent_period_s: float = 4.0
    intermittent_duty_cycle: float = 0.5
    range_bias_m: float = 200.0
    extra_false_probability: float = 0.2
    extra_latency_s: float = 1.0
    datalink_dropout_probability: float = 0.3
    interference_level: float = 0.5

    def active(self, t: float) -> bool:
        return self.start_time_s <= t <= self.end_time_s

    def extra_sensor_failure_probability(self, t: float) -> float:
        if not self.active(t):
            return 0.0
        if self.mode == "no_detection":
            return 1.0
        if self.mode == "intermittent":
            phase = t % self.intermittent_period_s
            return 1.0 if phase < (self.intermittent_period_s * self.intermittent_duty_cycle) else 0.0
        return 0.0

    def extra_interference(self, t: float) -> float:
        if not self.active(t):
            return 0.0
        if self.mode == "interference":
            return self.interference_level
        if self.mode == "false_detection_burst":
            return 0.3
        return 0.0

    def apply_post_measurement(self, t: float, meas: SensorMeasurement,
                                rng: np.random.Generator) -> SensorMeasurement:
        """Apply failure effects that act after the raw measurement is
        generated (bias, extra latency, data-link loss)."""
        if not self.active(t):
            return meas

        if self.mode == "biased_range" and meas.target_detected and meas.estimated_range is not None:
            meas.estimated_range += self.range_bias_m

        if self.mode == "delayed_measurement":
            meas.latency += self.extra_latency_s

        if self.mode == "datalink_dropout":
            if rng.random() < self.datalink_dropout_probability:
                return SensorMeasurement(
                    target_detected=False, estimated_range=None, relative_angle=None,
                    relative_velocity=None, confidence=0.0,
                    latency=meas.latency, sensor_failed=True,
                )

        return meas
