"""
monte_carlo.py
Monte Carlo experiment harness (spec section 10).

Randomizes target distance, relative velocity, approach angle, fog
density, sensor noise/failure/latency, and produces aggregate metrics
(spec section 11) for a given sensing mode ("lidar" | "radar" | "fusion").
"""

from dataclasses import dataclass, field
from typing import List, Optional
import numpy as np

from .aircraft import Aircraft, AircraftState
from .simulation import SimulationConfig, run_scenario
from .collision import CollisionRiskConfig
from .failures import FailureConfig
from .metrics import extract_outcome, aggregate, AggregateMetrics, ScenarioOutcome


@dataclass
class MonteCarloConfig:
    n_trials: int = 300
    mode: str = "lidar"
    fog_keys: List[str] = field(default_factory=lambda: ["clear", "light", "moderate", "dense"])
    range_bounds_m: tuple = (1500.0, 9000.0)
    closing_speed_bounds_mps: tuple = (40.0, 300.0)
    lateral_offset_bounds_m: tuple = (-1200.0, 1200.0)
    duration_s: float = 90.0
    dt_s: float = 0.1
    failure_mode_pool: List[str] = field(default_factory=lambda: ["none"])
    seed: int = 42


def run_monte_carlo(config: MonteCarloConfig):
    """Returns dict: fog_key -> (List[ScenarioOutcome], AggregateMetrics)"""
    rng = np.random.default_rng(config.seed)
    results_by_fog = {}

    for fog_key in config.fog_keys:
        outcomes: List[ScenarioOutcome] = []
        for trial in range(config.n_trials):
            initial_range = rng.uniform(*config.range_bounds_m)
            closing_speed = rng.uniform(*config.closing_speed_bounds_mps)
            lateral_offset = rng.uniform(*config.lateral_offset_bounds_m)
            failure_mode = config.failure_mode_pool[rng.integers(len(config.failure_mode_pool))]

            host = Aircraft(
                name="A", role="host",
                state=AircraftState(x=0.0, y=0.0, z=3000.0, heading=0.0, speed=90.0, size=15.0),
            )
            target = Aircraft(
                name="B", role="target",
                state=AircraftState(
                    x=initial_range, y=lateral_offset, z=3000.0,
                    heading=np.pi, speed=closing_speed - 90.0, size=15.0,
                ),
            )

            failure = FailureConfig(mode=failure_mode, start_time_s=0.0, end_time_s=1e9) \
                if failure_mode != "none" else FailureConfig(mode="none")

            sim_config = SimulationConfig(
                duration_s=config.duration_s, dt_s=config.dt_s,
                fog_key=fog_key, mode=config.mode, failure=failure,
                collision_config=CollisionRiskConfig(),
                seed=int(rng.integers(1_000_000_000)),
            )

            result = run_scenario(host, target, sim_config)
            outcomes.append(extract_outcome(result))

        results_by_fog[fog_key] = (outcomes, aggregate(outcomes))

    return results_by_fog
