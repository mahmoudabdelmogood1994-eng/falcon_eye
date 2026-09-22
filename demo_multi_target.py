"""
demo_multi_target.py
Reproduces the 3-target scenario used to test the Black Dragon triage
integration, but sources measurements from the actual Falcon Eye sensor
model (falcon_eye.sensor) so relative_velocity reflects true target
motion rather than a host-only approximation.
"""

import numpy as np
from falcon_eye.sensor import SensorMeasurement
from falcon_eye.multi_target_priority import BlackDragonTriageEngine

# Three simultaneous tracks, built the same way the sensor model would
# report them (SensorMeasurement already carries a proper relative
# closing velocity -- positive = approaching).
raw_observations = [
    {"target_id": 101, "measurement": SensorMeasurement(
        target_detected=True, estimated_range=2200.0, relative_angle=np.radians(1.0),
        relative_velocity=195.0, confidence=0.9)},   # TTC ~11.3s -> WARNING
    {"target_id": 102, "measurement": SensorMeasurement(
        target_detected=True, estimated_range=900.0, relative_angle=np.radians(-0.5),
        relative_velocity=205.0, confidence=0.92)},  # TTC ~4.4s -> CRITICAL
    {"target_id": 103, "measurement": SensorMeasurement(
        target_detected=True, estimated_range=3500.0, relative_angle=np.radians(12.0),
        relative_velocity=180.0, confidence=0.85)},  # outside corridor -> CLEAR
]

engine = BlackDragonTriageEngine(
    corridor_half_angle_rad=np.radians(4.3),  # ~150m half-width at ~2000m, matching snippet's corridor_width=150
    ttc_critical_s=8.0,
    processing_latency_budget_s=0.05,
)

result = engine.process_target_tracks(raw_observations)

print("=== Reflex fast-path ===")
if result.reflex_triggered_before_full_sort:
    print(f"CRITICAL_REFLEX fired on target {result.reflex_target_id} "
          f"BEFORE the remaining targets were scored/sorted.")
else:
    print("No target crossed the critical TTC threshold.")

print("\n=== Full slack-priority sorted track list (most urgent first) ===")
for rank, t in enumerate(result.tracks, 1):
    ttc_str = f"{t.ttc:.2f}s" if np.isfinite(t.ttc) else "inf"
    slack_str = f"{t.slack:.2f}s" if np.isfinite(t.slack) else "inf"
    print(f"Rank {rank} | Target {t.target_id} | TTC: {ttc_str:>7} | "
          f"Slack: {slack_str:>7} | Risk: {t.risk_level}")
