"""
fog.py
Environmental visibility degradation model for the Falcon Eye LiDAR-like
sensor.

Key research caveat (per spec section 6): dense fog is NOT assumed to be
"seen through" by LiDAR. Instead we model physically-motivated attenuation
of the optical return signal using a Beer-Lambert extinction law, and let
that attenuation drive down detection probability / usable range while
driving up noise and false-return rates. This lets the simulation honestly
answer "does the sensor still help in fog?" rather than assuming it does.
"""

from dataclasses import dataclass
import numpy as np


@dataclass
class FogScenario:
    name: str
    # Meteorological visibility, i.e. distance at which a large dark
    # object is just distinguishable (m). This is the standard variable
    # used to derive Beer-Lambert extinction coefficients.
    visibility_m: float
    # Extra multiplicative noise-inflation and false-return factors,
    # calibrated qualitatively (fog scatters returns -> more spurious hits).
    noise_multiplier: float
    false_return_multiplier: float

    def extinction_coefficient(self) -> float:
        """Koschmieder approximation: sigma = 3.912 / visibility.
        Standard relation used in atmospheric optics / LiDAR range
        equations to relate meteorological visibility to the volume
        extinction coefficient (units: 1/m)."""
        return 3.912 / max(self.visibility_m, 1.0)

    def two_way_transmittance(self, range_m: float) -> float:
        """Beer-Lambert two-way (out-and-back) atmospheric transmittance
        of the laser pulse at a given range. Value in [0, 1]."""
        sigma = self.extinction_coefficient()
        return float(np.exp(-2.0 * sigma * range_m))


# Predefined scenarios per spec section 6
FOG_SCENARIOS = {
    "clear":    FogScenario("Clear air",     visibility_m=20000.0, noise_multiplier=1.0, false_return_multiplier=1.0),
    "light":    FogScenario("Light fog",     visibility_m=2000.0,  noise_multiplier=1.5, false_return_multiplier=1.8),
    "moderate": FogScenario("Moderate fog",  visibility_m=500.0,   noise_multiplier=2.5, false_return_multiplier=3.5),
    "dense":    FogScenario("Dense fog",     visibility_m=100.0,   noise_multiplier=5.0, false_return_multiplier=8.0),
}


def get_fog_scenario(key: str) -> FogScenario:
    if key not in FOG_SCENARIOS:
        raise ValueError(f"Unknown fog scenario '{key}'. Options: {list(FOG_SCENARIOS)}")
    return FOG_SCENARIOS[key]
