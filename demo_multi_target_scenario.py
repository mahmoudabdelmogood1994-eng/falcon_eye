"""
demo_multi_target_scenario.py
Full end-to-end multi-target simulation: three approaching targets with
staggered geometry, tracked simultaneously and triaged every timestep by
BlackDragonTriageEngine (slack-priority sort + reflex fast-path), wired
directly into the simulation loop via multi_target_simulation.py.
"""

import numpy as np
from falcon_eye.aircraft import Aircraft, AircraftState
from falcon_eye.multi_target_simulation import MultiTargetSimulationConfig, run_multi_target_scenario
from falcon_eye.plots import plot_multi_target_trace

host = Aircraft(
    name="Aircraft A (host)", role="host",
    state=AircraftState(x=0.0, y=0.0, z=3000.0, heading=0.0, speed=90.0, size=15.0),
)

# Three targets:
#   T0 - far, slow-closing, in corridor: should stay ADVISORY/WARNING, never critical
#   T1 - closer, fast-closing head-on: should hit CRITICAL_REFLEX partway through
#   T2 - off-corridor crossing traffic: should stay CLEAR/ADVISORY, never dominate priority
targets = [
    Aircraft(name="B0", role="target", state=AircraftState(
        x=7000.0, y=0.0, z=3000.0, heading=np.pi, speed=40.0, size=15.0)),
    Aircraft(name="B1", role="target", state=AircraftState(
        x=3000.0, y=50.0, z=3000.0, heading=np.pi, speed=140.0, size=15.0)),
    Aircraft(name="B2", role="target", state=AircraftState(
        x=4000.0, y=900.0, z=3000.0, heading=np.pi, speed=90.0, size=15.0)),
]

config = MultiTargetSimulationConfig(
    duration_s=45.0, dt_s=0.1, fog_key="clear", mode="fusion",
    ttc_critical_s=8.0, seed=7,
)

result = run_multi_target_scenario(host, targets, config)

print("=== Reflex events (first-time triggers, deduplicated) ===")
if result.reflex_events:
    for rt, rid in result.reflex_events:
        print(f"t={rt:5.1f}s -> CRITICAL_REFLEX fired for target {rid}")
else:
    print("No reflex events triggered.")

print("\n=== Final risk level per target ===")
for tid in result.target_ids:
    print(f"T{tid}: final risk = {result.risk_level[tid][-1]}, "
          f"final true range = {result.true_range[tid][-1]:.0f} m")

print("\n=== Priority hand-offs — RAW slack sort (unfiltered) ===")
last = None
raw_switches = 0
for t, tid in zip(result.time, result.top_priority_target):
    if tid != last:
        if last is not None:
            raw_switches += 1
        print(f"t={t:5.1f}s -> top priority becomes T{tid}")
        last = tid

print("\n=== Priority hand-offs — HYSTERESIS-STABILIZED ===")
last = None
stable_switches = 0
for t, tid in zip(result.time, result.stable_priority_target):
    if tid != last:
        if last is not None:
            stable_switches += 1
        print(f"t={t:5.1f}s -> stable priority becomes T{tid}")
        last = tid

print(f"\nRaw switches: {raw_switches}  |  Stabilized switches: {stable_switches}  "
      f"|  Reduction: {100*(1 - stable_switches/max(raw_switches,1)):.0f}%")

plot_multi_target_trace(result, "Multi-Target Triage — 3 simultaneous tracks (fusion mode)",
                         "outputs/figures/multi_target_triage.png")
print("\nSaved outputs/figures/multi_target_triage.png")
