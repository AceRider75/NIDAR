from dataclasses import dataclass, field
from typing import Dict
from enum import Enum
from core.radio_comm import RadioComm

def default_telemetry() -> Dict[str, float]:
    return {            
            "lat": 0.0,
            "lon": 0.0,
            "alt": 0.0,
            "vx": 0.0,
            "vy": 0.0,
            "vz": 0.0,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
            "xacc": 0.0,
            "yacc": 0.0
            }

@dataclass
class DroneState:
    name: str
    password: str
    radio: RadioComm
    lat0: float = field(default = 0.0)
    lon0: float = field(default = 0.0)
    started: bool = False
    status: str = field(default = "")
    battery: int = field(default = 0)
    log: str = ""
    telemetry: Dict[str, float] = field(default_factory = default_telemetry)

class DroneName(Enum):
    Scanner = 0
    Sprayer = 1