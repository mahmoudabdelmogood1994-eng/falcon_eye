"""
collision.py
Collision-risk assessment and warning logic (spec section 7).

Implements:
  Target detected
    -> inside projected flight corridor?
    -> closing velocity > threshold?
    -> TTC below warning threshold?
    -> generate warning

All thresholds are configurable via CollisionRiskConfig.
"""

from dataclasses import dataclass
from enum import Enum
from typing import Optional
import numpy as np


class RiskLevel(str, Enum):
    NONE = "none"
    ADVISORY = "advisory"
    WARNING = "warning"


@dataclass
class CollisionRiskConfig:
    corridor_half_angle_rad: float = np.radians(5.0)  # "inside flight path" cone
    closing_velocity_threshold_mps: float = 5.0
    ttc_warning_threshold_s: float = 25.0
    ttc_advisory_threshold_s: float = 45.0


@dataclass
class RiskAssessment:
    in_corridor: bool
    closing_velocity: float
    ttc: Optional[float]
    risk_level: RiskLevel


def assess_risk(
    estimated_range: Optional[float],
    relative_angle: Optional[float],
    relative_velocity: Optional[float],
    config: CollisionRiskConfig,
) -> RiskAssessment:
    """relative_velocity is defined as closing speed: positive = approaching."""
    if estimated_range is None or relative_angle is None or relative_velocity is None:
        return RiskAssessment(False, 0.0, None, RiskLevel.NONE)

    in_corridor = abs(relative_angle) <= config.corridor_half_angle_rad
    closing = relative_velocity

    if closing > 1e-3:
        ttc = estimated_range / closing
    else:
        ttc = None  # not closing -> no meaningful TTC

    risk = RiskLevel.NONE
    if in_corridor and closing > config.closing_velocity_threshold_mps and ttc is not None:
        if ttc <= config.ttc_warning_threshold_s:
            risk = RiskLevel.WARNING
        elif ttc <= config.ttc_advisory_threshold_s:
            risk = RiskLevel.ADVISORY

    return RiskAssessment(in_corridor, closing, ttc, risk)
