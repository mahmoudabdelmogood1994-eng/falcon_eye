"""
metrics.py
Extracts the performance metrics required by spec section 11 from one or
many SimulationResult objects.
"""

from dataclasses import dataclass
from typing import List, Optional
import numpy as np

from .simulation import SimulationResult


@dataclass
class ScenarioOutcome:
    # A scenario is "dangerous" if the true trajectory would have produced
    # a collision (min true range below combined aircraft size) absent any
    # avoidance action.
    is_dangerous: bool
    detected: bool                     # any true (non-false) detection occurred
    detection_range_m: Optional[float]
    detection_time_s: Optional[float]
    warning_time_s: Optional[float]
    collision_time_s: Optional[float]
    warning_lead_time_s: Optional[float]
    ttc_estimation_error_s: Optional[float]
    false_alarm: bool                  # warning issued with no dangerous target
    min_true_range_m: float


def extract_outcome(result: SimulationResult, danger_range_m: float = 500.0) -> ScenarioOutcome:
    is_dangerous = result.min_true_range <= danger_range_m
    detected = result.first_detection_time is not None

    warning_lead = None
    if result.warning_time is not None and result.collision_time is not None:
        warning_lead = result.collision_time - result.warning_time

    ttc_error = None
    if result.warning_time is not None:
        idx = np.searchsorted(result.time, result.warning_time)
        idx = min(idx, len(result.time) - 1)
        true_ttc_at_warning = result.true_ttc[idx]
        tracked_range_at_warning = result.tracked_range[idx]
        if not np.isnan(true_ttc_at_warning) and not np.isnan(tracked_range_at_warning):
            # Recompute estimated TTC at that step from tracked range /
            # true closing velocity proxy (approx via finite difference).
            if idx > 0 and not np.isnan(result.tracked_range[idx - 1]):
                dt = result.time[idx] - result.time[idx - 1]
                d_range = result.tracked_range[idx - 1] - result.tracked_range[idx]
                est_closing = d_range / dt if dt > 0 else 0.0
                if est_closing > 1e-3:
                    est_ttc = tracked_range_at_warning / est_closing
                    ttc_error = est_ttc - true_ttc_at_warning

    false_alarm = (result.warning_time is not None) and (not is_dangerous)

    return ScenarioOutcome(
        is_dangerous=is_dangerous,
        detected=detected,
        detection_range_m=result.first_detection_range,
        detection_time_s=result.first_detection_time,
        warning_time_s=result.warning_time,
        collision_time_s=result.collision_time,
        warning_lead_time_s=warning_lead,
        ttc_estimation_error_s=ttc_error,
        false_alarm=false_alarm,
        min_true_range_m=result.min_true_range,
    )


@dataclass
class AggregateMetrics:
    n_trials: int
    n_dangerous: int
    detection_rate: float               # of dangerous scenarios, fraction detected
    missed_detection_rate: float
    false_alarm_rate: float             # of non-dangerous scenarios, fraction with warning
    mean_detection_range_m: Optional[float]
    mean_detection_time_s: Optional[float]
    mean_warning_lead_time_s: Optional[float]
    mean_abs_ttc_error_s: Optional[float]
    availability: float                 # fraction of dangerous scenarios with any track


def aggregate(outcomes: List[ScenarioOutcome]) -> AggregateMetrics:
    n = len(outcomes)
    dangerous = [o for o in outcomes if o.is_dangerous]
    non_dangerous = [o for o in outcomes if not o.is_dangerous]

    n_dangerous = len(dangerous)
    detected_dangerous = [o for o in dangerous if o.detected]
    detection_rate = len(detected_dangerous) / n_dangerous if n_dangerous else float("nan")
    missed_rate = 1.0 - detection_rate if n_dangerous else float("nan")

    false_alarms = [o for o in non_dangerous if o.false_alarm]
    false_alarm_rate = len(false_alarms) / len(non_dangerous) if non_dangerous else float("nan")

    det_ranges = [o.detection_range_m for o in dangerous if o.detection_range_m is not None]
    det_times = [o.detection_time_s for o in dangerous if o.detection_time_s is not None]
    leads = [o.warning_lead_time_s for o in dangerous if o.warning_lead_time_s is not None]
    ttc_errs = [abs(o.ttc_estimation_error_s) for o in dangerous if o.ttc_estimation_error_s is not None]

    availability = len(detected_dangerous) / n_dangerous if n_dangerous else float("nan")

    return AggregateMetrics(
        n_trials=n,
        n_dangerous=n_dangerous,
        detection_rate=detection_rate,
        missed_detection_rate=missed_rate,
        false_alarm_rate=false_alarm_rate,
        mean_detection_range_m=float(np.mean(det_ranges)) if det_ranges else None,
        mean_detection_time_s=float(np.mean(det_times)) if det_times else None,
        mean_warning_lead_time_s=float(np.mean(leads)) if leads else None,
        mean_abs_ttc_error_s=float(np.mean(ttc_errs)) if ttc_errs else None,
        availability=availability,
    )
