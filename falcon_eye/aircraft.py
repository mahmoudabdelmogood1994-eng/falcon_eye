"""
aircraft.py
Configurable kinematic model for the host aircraft (Falcon Eye carrier)
and target aircraft / obstacles.

All units: meters, seconds, radians unless noted otherwise.
"""

from dataclasses import dataclass, field
import numpy as np


@dataclass
class AircraftState:
    """Kinematic state of an aircraft or obstacle in a simplified 3D frame.

    x: forward/east position (m)
    y: lateral/north position (m)
    z: altitude (m)
    heading: direction of travel in the x-y plane, radians, 0 = +x axis
    speed: forward speed magnitude (m/s)
    vertical_speed: climb/descend rate (m/s)
    """
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    heading: float = 0.0
    speed: float = 0.0
    vertical_speed: float = 0.0
    heading_rate: float = 0.0       # rad/s, for turning targets
    acceleration: float = 0.0        # m/s^2 along heading

    # Physical / detectability properties
    size: float = 10.0               # characteristic dimension (m), used as
                                      # a simple proxy for radar/optical
                                      # cross-section -- bigger = easier to
                                      # detect, not physically exact RCS.

    def velocity_vector(self) -> np.ndarray:
        vx = self.speed * np.cos(self.heading)
        vy = self.speed * np.sin(self.heading)
        vz = self.vertical_speed
        return np.array([vx, vy, vz])

    def position_vector(self) -> np.ndarray:
        return np.array([self.x, self.y, self.z])

    def step(self, dt: float):
        """Advance kinematic state forward by dt seconds (in place)."""
        self.speed = max(0.0, self.speed + self.acceleration * dt)
        self.heading += self.heading_rate * dt
        v = self.velocity_vector()
        self.x += v[0] * dt
        self.y += v[1] * dt
        self.z += v[2] * dt


@dataclass
class Aircraft:
    """Wraps an AircraftState with an identifier and a role (host/target)."""
    name: str
    state: AircraftState
    role: str = "target"  # "host" or "target"
    history: list = field(default_factory=list)

    def step(self, dt: float):
        self.state.step(dt)
        self.history.append(self.state.position_vector().copy())

    def position(self) -> np.ndarray:
        return self.state.position_vector()

    def velocity(self) -> np.ndarray:
        return self.state.velocity_vector()
