"""
multi_target_simulation.py
Extends the single-target simulation engine (simulation.py) to N
simultaneous targets, with per-target tracking (tracking.AlphaBetaTrack)
and per-timestep triage via BlackDragonTriageEngine.

This directly exercises spec section 8's "Multiple targets" failure/
stress scenario: several tracks competing for the sensor's and the
collision-logic's attention at once, with the reflex fast-path deciding
which one gets an immediate hardware-level response.
"""

from dataclasses import dataclass, field
from typing import Dict, List, Optional
import numpy as np

from .aircraft import Aircraft
from .sensor import FalconEyeSensor, SensorMeasurement
from .radar import RadarSensor
from .fog import get_fog_scenario
from .failures import FailureConfig
from .tracking import AlphaBetaTrack, fuse_measurements
from .multi_target_priority import BlackDragonTriageEngine, PriorityHysteresis
from .simulation import relative_geometry


@dataclass
class MultiTargetSimulationConfig:
    duration_s: float = 60.0
    dt_s: float = 0.1
    fog_key: str = "clear"
    mode: str = "lidar"  # "lidar" | "radar" | "fusion"
    failure: FailureConfig = field(default_factory=FailureConfig)
    corridor_half_angle_rad: float = np.radians(5.0)
    ttc_critical_s: float = 8.0
    ttc_warning_multiplier: float = 2.0
    processing_latency_budget_s: float = 0.05
    priority_hold_time_s: float = 1.0
    seed: Optional[int] = None


@dataclass
class MultiTargetSimulationResult:
    time: np.ndarray
    target_ids: List[int]
    true_range: Dict[int, np.ndarray]
    tracked_range: Dict[int, np.ndarray]
    risk_level: Dict[int, List[str]]         # per target, per timestep
    reflex_events: List[tuple]               # (time, target_id) — reflex fired, first time per event
    top_priority_target: List[Optional[int]]  # per timestep, raw slack-sort #1 (unfiltered, can flicker)
    stable_priority_target: List[Optional[int]]  # per timestep, hysteresis-stabilized designation


def run_multi_target_scenario(
    host: Aircraft, targets: List[Aircraft], config: MultiTargetSimulationConfig
) -> MultiTargetSimulationResult:
    rng = np.random.default_rng(config.seed)
    fog = get_fog_scenario(config.fog_key)

    lidar = FalconEyeSensor(rng=np.random.default_rng(rng.integers(1e9)))
    radar = RadarSensor(rng=np.random.default_rng(rng.integers(1e9)))

    engine = BlackDragonTriageEngine(
        corridor_half_angle_rad=config.corridor_half_angle_rad,
        ttc_critical_s=config.ttc_critical_s,
        ttc_warning_multiplier=config.ttc_warning_multiplier,
        processing_latency_budget_s=config.processing_latency_budget_s,
    )
    hysteresis = PriorityHysteresis(hold_time_s=config.priority_hold_time_s)

    target_ids = [t.state.__hash__() if False else i for i, t in enumerate(targets)]
    # use stable explicit ids via enumerate index (targets carry no id field)
    target_ids = list(range(len(targets)))

    tracks: Dict[int, AlphaBetaTrack] = {tid: AlphaBetaTrack() for tid in target_ids}
    lidar_acc: Dict[int, float] = {tid: 0.0 for tid in target_ids}
    radar_acc: Dict[int, float] = {tid: 0.0 for tid in target_ids}

    n_steps = int(config.duration_s / config.dt_s)
    time = np.zeros(n_steps)
    true_range = {tid: np.zeros(n_steps) for tid in target_ids}
    tracked_range = {tid: np.full(n_steps, np.nan) for tid in target_ids}
    risk_level: Dict[int, List[str]] = {tid: [] for tid in target_ids}
    reflex_events: List[tuple] = []
    top_priority_target: List[Optional[int]] = []
    stable_priority_target: List[Optional[int]] = []

    last_reflex_target = None  # avoid re-logging the same continuous reflex condition every step
    t = 0.0

    for i in range(n_steps):
        obs_list = []

        for tid, target in zip(target_ids, targets):
            tr, ta, closing = relative_geometry(host, target)
            true_range[tid][i] = tr

            extra_fail_p = config.failure.extra_sensor_failure_probability(t)
            extra_interf = config.failure.extra_interference(t)

            lidar_acc[tid] += config.dt_s
            radar_acc[tid] += config.dt_s
            lidar_meas = None
            radar_meas = None

            if config.mode in ("lidar", "fusion") and lidar_acc[tid] >= lidar.dt():
                lidar_acc[tid] = 0.0
                lidar_meas = lidar.measure(tr, ta, closing, fog, extra_fail_p, extra_interf)
                lidar_meas = config.failure.apply_post_measurement(t, lidar_meas, rng)

            if config.mode in ("radar", "fusion") and radar_acc[tid] >= radar.dt():
                radar_acc[tid] = 0.0
                radar_meas = radar.measure(tr, ta, closing, fog, extra_fail_p, extra_interf)
                radar_meas = config.failure.apply_post_measurement(t, radar_meas, rng)

            if config.mode == "lidar":
                meas_for_track = lidar_meas
            elif config.mode == "radar":
                meas_for_track = radar_meas
            else:
                meas_for_track = fuse_measurements(lidar_meas, radar_meas) \
                    if (lidar_meas is not None or radar_meas is not None) else None

            track = tracks[tid]
            track.update(meas_for_track, config.dt_s)
            tracked_range[tid][i] = track.range_est if track.is_valid() else np.nan

            if track.is_valid():
                synth_meas = SensorMeasurement(
                    target_detected=True,
                    estimated_range=track.range_est,
                    relative_angle=track.angle_est,
                    relative_velocity=track.closing_velocity(),
                    confidence=0.8,
                )
                obs_list.append({"target_id": tid, "measurement": synth_meas})

        # --- Triage: reflex fast-path + full slack-priority sort, every step ---
        triage = engine.process_target_tracks(obs_list)

        if triage.reflex_target_id is not None and triage.reflex_target_id != last_reflex_target:
            reflex_events.append((t, triage.reflex_target_id))
        last_reflex_target = triage.reflex_target_id

        top_priority_target.append(triage.tracks[0].target_id if triage.tracks else None)
        stable_priority_target.append(hysteresis.update(t, triage.tracks, triage.reflex_target_id))

        track_risk_by_id = {trk.target_id: trk.risk_level for trk in triage.tracks}
        for tid in target_ids:
            risk_level[tid].append(track_risk_by_id.get(tid, "CLEAR"))

        time[i] = t
        host.step(config.dt_s)
        for target in targets:
            target.step(config.dt_s)
        t += config.dt_s

    return MultiTargetSimulationResult(
        time=time, target_ids=target_ids, true_range=true_range, tracked_range=tracked_range,
        risk_level=risk_level, reflex_events=reflex_events, top_priority_target=top_priority_target,
        stable_priority_target=stable_priority_target,
    )
