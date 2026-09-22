"""
run_demo.py
Runs the Falcon Eye first-prototype demonstration described in spec
section 14:

  1. Milestone 1: single clear-air head-on scenario (baseline pipeline proof)
  2. Fog progression: same geometry under clear -> light -> moderate -> dense fog
  3. Failure-injection scenarios (spec section 8)
  4. Monte Carlo comparison: LiDAR-only vs Radar-only vs Fusion, across fog
     densities (spec sections 9-11)
  5. Writes all figures + a comparison table + a technical report to
     outputs/
"""

import json
import numpy as np
import pandas as pd

from falcon_eye.simulation import make_head_on_scenario, SimulationConfig, run_scenario
from falcon_eye.failures import FailureConfig
from falcon_eye.collision import CollisionRiskConfig
from falcon_eye.monte_carlo import MonteCarloConfig, run_monte_carlo
from falcon_eye.metrics import extract_outcome, aggregate
from falcon_eye.plots import plot_scenario_trace, plot_mode_fog_comparison

OUT = "outputs"
FIG = f"{OUT}/figures"

# ---------------------------------------------------------------------
# 1. Milestone 1: baseline clear-air detection -> track -> TTC -> warning
# ---------------------------------------------------------------------
print("=== Milestone 1: baseline clear-air scenario ===")
host, target = make_head_on_scenario(initial_range_m=6000, closing_speed_mps=180)
cfg = SimulationConfig(duration_s=90, dt_s=0.1, fog_key="clear", mode="lidar", seed=1)
result = run_scenario(host, target, cfg)
print(f"First detection: t={result.first_detection_time:.1f}s, "
      f"range={result.first_detection_range:.0f}m")
print(f"Warning issued: t={result.warning_time:.1f}s" if result.warning_time else "No warning issued")
print(f"Projected collision point: t={result.collision_time:.1f}s" if result.collision_time else "No collision")
plot_scenario_trace(result, "Milestone 1 — Clear air, head-on, LiDAR only",
                     f"{FIG}/milestone1_clear_air.png")

# ---------------------------------------------------------------------
# 2. Fog progression on identical geometry
# ---------------------------------------------------------------------
print("\n=== Fog progression (identical geometry, LiDAR only) ===")
fog_summary = []
for fog_key in ["clear", "light", "moderate", "dense"]:
    host, target = make_head_on_scenario(initial_range_m=6000, closing_speed_mps=180)
    cfg = SimulationConfig(duration_s=90, dt_s=0.1, fog_key=fog_key, mode="lidar", seed=1)
    r = run_scenario(host, target, cfg)
    fog_summary.append({
        "fog": fog_key,
        "first_detection_time_s": r.first_detection_time,
        "first_detection_range_m": r.first_detection_range,
        "warning_time_s": r.warning_time,
        "collision_time_s": r.collision_time,
    })
    plot_scenario_trace(r, f"Fog progression — {fog_key} (LiDAR only)",
                         f"{FIG}/fog_{fog_key}_lidar.png")
    print(fog_key, fog_summary[-1])

pd.DataFrame(fog_summary).to_csv(f"{OUT}/fog_progression_table.csv", index=False)

# ---------------------------------------------------------------------
# 3. Failure-injection scenarios
# ---------------------------------------------------------------------
print("\n=== Failure-injection scenarios ===")
failure_scenarios = {
    "no_detection": FailureConfig(mode="no_detection", start_time_s=0, end_time_s=1e9),
    "intermittent": FailureConfig(mode="intermittent", start_time_s=0, end_time_s=1e9,
                                   intermittent_period_s=4.0, intermittent_duty_cycle=0.6),
    "biased_range": FailureConfig(mode="biased_range", start_time_s=0, end_time_s=1e9, range_bias_m=400.0),
    "false_detection_burst": FailureConfig(mode="false_detection_burst", start_time_s=10, end_time_s=40),
    "delayed_measurement": FailureConfig(mode="delayed_measurement", start_time_s=0, end_time_s=1e9, extra_latency_s=1.5),
    "datalink_dropout": FailureConfig(mode="datalink_dropout", start_time_s=0, end_time_s=1e9, datalink_dropout_probability=0.4),
    "interference": FailureConfig(mode="interference", start_time_s=0, end_time_s=1e9, interference_level=0.6),
}

failure_summary = []
for name, fc in failure_scenarios.items():
    host, target = make_head_on_scenario(initial_range_m=6000, closing_speed_mps=180)
    cfg = SimulationConfig(duration_s=90, dt_s=0.1, fog_key="clear", mode="lidar", failure=fc, seed=3)
    r = run_scenario(host, target, cfg)
    outcome = extract_outcome(r)
    failure_summary.append({
        "failure_mode": name,
        "detected": outcome.detected,
        "warning_issued": r.warning_time is not None,
        "warning_lead_time_s": outcome.warning_lead_time_s,
    })
    print(name, failure_summary[-1])

pd.DataFrame(failure_summary).to_csv(f"{OUT}/failure_injection_table.csv", index=False)

# ---------------------------------------------------------------------
# 4. Monte Carlo comparison: LiDAR vs Radar vs Fusion across fog density
# ---------------------------------------------------------------------
print("\n=== Monte Carlo comparison: LiDAR vs Radar vs Fusion ===")
mc_results = {}
for mode in ["lidar", "radar", "fusion"]:
    mc_cfg = MonteCarloConfig(
        n_trials=200, mode=mode,
        fog_keys=["clear", "light", "moderate", "dense"],
        duration_s=70, dt_s=0.15, seed=100,
    )
    out = run_monte_carlo(mc_cfg)
    mc_results[mode] = {fog_key: agg for fog_key, (outcomes, agg) in out.items()}
    print(f"-- mode={mode} done --")
    for fog_key, agg in mc_results[mode].items():
        print(fog_key, agg)

# Comparison table
rows = []
for mode, by_fog in mc_results.items():
    for fog_key, agg in by_fog.items():
        rows.append({
            "mode": mode, "fog": fog_key,
            "n_trials": agg.n_trials, "n_dangerous": agg.n_dangerous,
            "detection_rate": agg.detection_rate,
            "missed_detection_rate": agg.missed_detection_rate,
            "false_alarm_rate": agg.false_alarm_rate,
            "mean_detection_range_m": agg.mean_detection_range_m,
            "mean_detection_time_s": agg.mean_detection_time_s,
            "mean_warning_lead_time_s": agg.mean_warning_lead_time_s,
            "mean_abs_ttc_error_s": agg.mean_abs_ttc_error_s,
            "availability": agg.availability,
        })
comparison_df = pd.DataFrame(rows)
comparison_df.to_csv(f"{OUT}/monte_carlo_comparison_table.csv", index=False)
print("\nComparison table:")
print(comparison_df.to_string(index=False))

# Plots
plot_mode_fog_comparison(mc_results, "detection_rate", "Detection rate (dangerous scenarios)",
                          "Detection Rate vs Fog Density by Sensing Mode",
                          f"{FIG}/detection_rate_comparison.png")
plot_mode_fog_comparison(mc_results, "false_alarm_rate", "False alarm rate",
                          "False Alarm Rate vs Fog Density by Sensing Mode",
                          f"{FIG}/false_alarm_rate_comparison.png")
plot_mode_fog_comparison(mc_results, "availability", "Availability (fraction tracked)",
                          "Availability vs Fog Density by Sensing Mode",
                          f"{FIG}/availability_comparison.png")

print("\nAll outputs written to outputs/")
