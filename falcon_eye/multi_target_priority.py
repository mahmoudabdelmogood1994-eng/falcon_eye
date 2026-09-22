"""
multi_target_priority.py
Multi-target track prioritization for Falcon Eye, adapting two ideas from
the Black Dragon OGC 2026 shipyard-packing solver:

  - Squirrel (slack-priority sort): process/report the most time-critical
    target first, using slack = TTC - processing_latency_budget as the
    priority key (ascending -> most urgent first). Directly analogous to
    Black Dragon's slack = due_date - release_time - processing_time sort
    used to decide which block gets the best remaining option first.

  - Reflex fast-path: if any target is inside a hard "critical" TTC
    threshold, surface it immediately for an emergency hardware trigger
    rather than waiting for full multi-target scoring/sorting to finish.
    Analogous to Black Dragon's Reflex Layer, which detects constraint
    violations in real time and triggers an emergency repack rather than
    completing the full pass first.

Two correctness issues in the original prototype snippet are fixed here:

  1. Closing velocity must come from the sensor's actual relative-velocity
     measurement (which reflects both aircraft's motion), not from
     projecting only the host's own velocity onto the line of sight. The
     latter silently assumes every target is stationary.
  2. The reflex fast-path must actually short-circuit before full-list
     processing for the critical target, not just `break` out of a loop
     that runs after every target has already been fully processed.
"""

from dataclasses import dataclass
from typing import List, Optional
import numpy as np

from .sensor import SensorMeasurement
from .collision import CollisionRiskConfig, RiskLevel


@dataclass
class TargetTrack:
    target_id: int
    estimated_range: float
    relative_angle: float
    closing_velocity: float   # positive = approaching (from sensor measurement, not re-derived)
    ttc: float                 # seconds; math.inf if not closing / not applicable
    slack: float                # Black-Dragon-style slack = ttc - processing_latency_budget
    risk_level: str             # "CRITICAL_REFLEX" | "WARNING" | "ADVISORY" | "CLEAR"


@dataclass
class MultiTargetTriageResult:
    tracks: List[TargetTrack]           # full track list, slack-sorted ascending (most urgent first)
    reflex_target_id: Optional[int]     # target that triggered the immediate hardware override, if any
    reflex_triggered_before_full_sort: bool  # True: reflex fired on first pass, before scoring the rest


class PriorityHysteresis:
    """
    Damps rapid switching of the *displayed/designated* top-priority target
    when two targets have close slack values (e.g. two aircraft at similar
    TTC), without ever delaying a genuine escalation.

    Asymmetric by design, and deliberately the opposite asymmetry from a
    typical resource-tier switcher: here, moving attention TO a more urgent
    target is immediate (never withhold a real escalation), while reverting
    to a LESS urgent designation is damped by a minimum hold time, since
    that direction is the one where sensor/track noise can cause meaningless
    flicker between two similarly-urgent targets. A genuine CRITICAL_REFLEX
    always bypasses the hold entirely, the same way a resource switcher's
    emergency-fallback trigger would bypass its own hysteresis -- hysteresis
    protects against noise-driven churn, never against a real emergency.
    """

    def __init__(self, hold_time_s: float = 1.0, escalation_slack_margin_s: float = 0.0):
        self.hold_time_s = hold_time_s
        self.escalation_slack_margin_s = escalation_slack_margin_s
        self._designated_id: Optional[int] = None
        self._designated_slack: float = float("inf")
        self._pending_id: Optional[int] = None
        self._pending_since_s: Optional[float] = None

    def update(self, t: float, tracks: List[TargetTrack], reflex_target_id: Optional[int]) -> Optional[int]:
        """Call once per triage cycle with the current time and the fresh,
        slack-sorted track list. Returns the hysteresis-stabilized
        designated top-priority target id (or None if no tracks)."""
        if not tracks:
            self._designated_id = None
            self._pending_id = None
            self._pending_since_s = None
            return None

        proposed = tracks[0]

        # Reflex always wins immediately -- never held back by hysteresis.
        if reflex_target_id is not None:
            self._designated_id = reflex_target_id
            self._designated_slack = next(
                (tr.slack for tr in tracks if tr.target_id == reflex_target_id), proposed.slack
            )
            self._pending_id = None
            self._pending_since_s = None
            return self._designated_id

        if self._designated_id is None:
            self._designated_id = proposed.target_id
            self._designated_slack = proposed.slack
            return self._designated_id

        if proposed.target_id == self._designated_id:
            self._designated_slack = proposed.slack
            self._pending_id = None
            self._pending_since_s = None
            return self._designated_id

        is_escalation = proposed.slack <= (self._designated_slack - self.escalation_slack_margin_s)

        if is_escalation:
            # More urgent than the current designee -- switch immediately.
            self._designated_id = proposed.target_id
            self._designated_slack = proposed.slack
            self._pending_id = None
            self._pending_since_s = None
            return self._designated_id

        # De-escalation candidate: require it to persist for hold_time_s.
        if self._pending_id != proposed.target_id:
            self._pending_id = proposed.target_id
            self._pending_since_s = t
        elif (t - self._pending_since_s) >= self.hold_time_s:
            self._designated_id = proposed.target_id
            self._designated_slack = proposed.slack
            self._pending_id = None
            self._pending_since_s = None

        return self._designated_id


class BlackDragonTriageEngine:
    """
    Slack-priority multi-target triage for Falcon Eye.

    Unlike a single-target collision check, a real forward sensor can
    report several simultaneous tracks (spec section 8: "Multiple
    targets" is an explicit failure/stress scenario). This engine decides
    *which* target to act on first and *how fast* a critical one can
    trigger a hardware-level response, without waiting on lower-priority
    targets to be scored.
    """

    def __init__(
        self,
        corridor_half_angle_rad: float = np.radians(5.0),
        ttc_critical_s: float = 8.0,
        ttc_warning_multiplier: float = 2.0,
        processing_latency_budget_s: float = 0.05,
    ):
        self.corridor_half_angle_rad = corridor_half_angle_rad
        self.ttc_critical_s = ttc_critical_s
        self.ttc_warning_s = ttc_critical_s * ttc_warning_multiplier
        self.processing_latency_budget_s = processing_latency_budget_s

    def _build_track(self, target_id: int, meas: SensorMeasurement) -> TargetTrack:
        r = meas.estimated_range
        theta = meas.relative_angle
        closing = meas.relative_velocity if meas.relative_velocity is not None else 0.0

        in_corridor = abs(theta) <= self.corridor_half_angle_rad
        if in_corridor and closing > 1e-3:
            ttc = r / closing
        else:
            ttc = float("inf")

        slack = ttc - self.processing_latency_budget_s if np.isfinite(ttc) else float("inf")

        if in_corridor and ttc <= self.ttc_critical_s:
            risk = "CRITICAL_REFLEX"
        elif in_corridor and ttc <= self.ttc_warning_s:
            risk = "WARNING"
        elif in_corridor:
            risk = "ADVISORY"
        else:
            risk = "CLEAR"

        return TargetTrack(
            target_id=target_id, estimated_range=r, relative_angle=theta,
            closing_velocity=closing, ttc=ttc, slack=slack, risk_level=risk,
        )

    def process_target_tracks(
        self, raw_observations: List[dict]
    ) -> MultiTargetTriageResult:
        """raw_observations: list of {"target_id": int, "measurement": SensorMeasurement}."""

        # --- Reflex fast-path: scan for a critical target FIRST, before any
        # sorting or full-list scoring, so a hard real-time deadline is not
        # blocked behind lower-priority targets. This is the fix for the
        # original snippet's no-op early exit. ---
        reflex_target_id = None
        for obs in raw_observations:
            meas: SensorMeasurement = obs["measurement"]
            if not meas.target_detected:
                continue
            r, theta = meas.estimated_range, meas.relative_angle
            closing = meas.relative_velocity if meas.relative_velocity is not None else 0.0
            in_corridor = abs(theta) <= self.corridor_half_angle_rad
            if in_corridor and closing > 1e-3:
                ttc = r / closing
                if ttc <= self.ttc_critical_s:
                    reflex_target_id = obs["target_id"]
                    break  # genuinely stop here -- do not score remaining targets yet

        # --- Full triage pass (runs regardless, for logging/tracking
        # continuity, but the reflex signal above has already fired if
        # applicable and does not wait on this). ---
        tracks = []
        for obs in raw_observations:
            meas: SensorMeasurement = obs["measurement"]
            if not meas.target_detected:
                continue
            tracks.append(self._build_track(obs["target_id"], meas))

        # Squirrel-style slack-priority sort: most urgent (lowest slack) first.
        tracks.sort(key=lambda t: t.slack)

        return MultiTargetTriageResult(
            tracks=tracks,
            reflex_target_id=reflex_target_id,
            reflex_triggered_before_full_sort=reflex_target_id is not None,
        )
