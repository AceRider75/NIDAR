from dataclasses import dataclass, field
from typing import Dict, List, Optional
import threading
from enum import Enum


class DroneName(Enum):
    """Enum for drone identifiers."""
    Scanner = 0
    Sprayer = 1


@dataclass
class DroneState:
    name: str
    password: str
    radio: any
    
    status: str = "Idle"
    battery: int = -1
    log: str = ""
    telemetry: Dict[str, float] = field(default_factory=lambda: {
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
        "yacc": 0.0,
        "zacc": 0.0,
        "battery": 0.0
    })
    
    lat0: Optional[float] = None
    lon0: Optional[float] = None
    started: bool = False
    
    # Yellow spots storage
    yellow_spots: Dict[str, List[Dict]] = field(default_factory=dict)
    yellow_spots_lock: threading.Lock = field(default_factory=threading.Lock)