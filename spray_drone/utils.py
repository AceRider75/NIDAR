import logging
import math
from enum import Enum, auto
from typing import List, Tuple
from dataclasses import dataclass, field
import time
import json
import os
import csv

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
LOG_DIR = os.path.join(BASE_DIR, "data", "logs")
LOG_FILE = os.path.join(LOG_DIR, "logs_live.csv")

class DroneState(Enum):
    """Finite state machine states"""
    DISCONNECTED = auto()
    CONNECTED = auto()
    IDLE = auto()
    ARMING = auto()
    ARMED = auto()
    TAKING_OFF = auto()
    MISSION_ACTIVE = auto()
    MISSION_PAUSED = auto()
    RETURNING_HOME = auto()
    LANDING = auto()
    EMERGENCY = auto()
    ERROR = auto()


class FlightMode(Enum):
    """MAVLink flight modes"""
    STABILIZE = "STABILIZE"
    GUIDED = "GUIDED"
    LAND = "LAND"
    RTL = "RTL"
    AUTO = "AUTO"
    LOITER = "LOITER"


@dataclass
class Waypoint:
    """Waypoint structure"""
    lat: float
    lon: float
    alt: float
    radius: float = 5.0
    timestamp: float = field(default_factory=time.time)
    validated: bool = False


def setup_logger(name: str, log_file: str, level: int = logging.INFO) -> logging.Logger:
    """Setup structured logging"""
    logger = logging.getLogger(name)
    logger.setLevel(level)

    # File handler
    fh = logging.FileHandler(log_file)
    fh.setLevel(level)

    # Console handler
    ch = logging.StreamHandler()
    ch.setLevel(level)

    # Formatter
    formatter = logging.Formatter(
        '%(asctime)s - %(name)s - [%(levelname)s] - %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S'
    )
    fh.setFormatter(formatter)
    ch.setFormatter(formatter)

    logger.addHandler(fh)
    logger.addHandler(ch)

    return logger


def haversine_dist(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Calculate distance between two GPS coordinates in meters"""
    R = 6371000  # Earth radius in meters
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)

    a = math.sin(dphi/2)**2 + math.cos(phi1) * \
        math.cos(phi2) * math.sin(dlambda/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))

    return R * c


def json_to_dict(json_string: str) -> dict:
    try:
        data = json.loads(json_string)
        return data
    except json.JSONDecodeError as e:
        # Silently ignore JSON parsing errors (common with noisy serial data)
        return None

def dict_to_json(data_dict: dict, indent=4) -> str:
    try:
        json_string = json.dumps(data_dict, indent=indent)
        return json_string
    except (TypeError, ValueError) as e:
        print("Cannot convert dictionary to JSON:", e)
        return None

def log_message(device: str, message: str) -> str:

    os.makedirs(os.path.dirname(LOG_FILE), exist_ok=True)

    with open(LOG_FILE, "a+", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([time.time(), device, message])
    
    logged_message = f"{time.time()} | [{device}]{message}"
    print(logged_message)

    return logged_message
