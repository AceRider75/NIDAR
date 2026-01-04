from dataclasses import dataclass, field
from utils import haversine_dist
from typing import Tuple

@dataclass
class GeoFence:
    """Geofencing configuration"""
    home_lat: float = 0.0
    home_lon: float = 0.0
    max_radius: float = 500.0  # meters
    min_altitude: float = 0.5
    max_altitude: float = 50.0
    enabled: bool = True

    def is_within_bounds(self, lat: float, lon: float, alt: float) -> Tuple[bool, str]:
        """Check if position is within geofence"""
        if not self.enabled:
            return True, "Geofence disabled"

        # Check altitude
        if alt < self.min_altitude:
            return False, f"Altitude too low: {alt:.2f}m < {self.min_altitude}m"
        if alt > self.max_altitude:
            return False, f"Altitude too high: {alt:.2f}m > {self.max_altitude}m"

        # Check horizontal distance
        dist = haversine_dist(self.home_lat, self.home_lon, lat, lon)
        if dist > self.max_radius:
            return False, f"Outside geofence: {dist:.2f}m > {self.max_radius}m"

        return True, "Within bounds"
