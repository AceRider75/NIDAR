
from dataclasses import dataclass
from typing import Dict, Any

@dataclass
class Telemetry:
    """Telemetry data structure"""
    timestamp: float = 0.0
    lat: float = 0.0
    lon: float = 0.0
    alt: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    xacc: float = 0.0
    yacc: float = 0.0
    battery: int = -1
    flight_mode: str = "UNKNOWN"
    armed: bool = False
    home_lat: float = 0.0
    home_lon: float = 0.0
    home_alt: float = 0.0

    def to_dict(self) -> Dict[str, Any]:
        return {
            'lat': self.lat,
            'lon': self.lon,
            'alt': self.alt,
            'vx': self.vx,
            'vy': self.vy,
            'vz': self.vz,
            'roll': self.roll,
            'pitch': self.pitch,
            'yaw': self.yaw,
            'battery': self.battery,
            'mode': self.flight_mode,
            'armed': self.armed,
            'home_lat': self.home_lat,
            'home_lon': self.home_lon,
            'home_alt': self.home_alt
        }
