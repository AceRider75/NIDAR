import threading
import time
import queue
from enum import Enum, auto
from dataclasses import dataclass, field
from typing import Optional, Tuple, List, Dict, Any
from pymavlink import mavutil
import math
from datetime import datetime
import logging


@dataclass
class DroneConfig:
    """Central configuration for drone operations"""
    # Connection
    connection_string: str = '127.0.0.1:14551'
    connection_timeout: int = 30
    heartbeat_timeout: int = 5

    # Safety limits
    max_altitude: float = 50.0  # meters
    min_altitude: float = 0.5
    max_speed: float = 10.0  # m/s
    geofence_radius: float = 500.0  # meters from home

    # Mission parameters
    waypoint_radius: float = 5.0  # meters
    altitude_tolerance: float = 1.0
    default_altitude: float = 3.0
    mission_timeout: int = 1800  # 30 minutes

    # Retry parameters
    max_arm_retries: int = 3
    max_takeoff_retries: int = 3
    max_mode_change_retries: int = 3
    command_ack_timeout: int = 3

    # Telemetry
    telemetry_rate_hz: int = 20
    state_update_rate_hz: int = 10

    # Logging
    log_file: str = "drone_flight.log"
    log_level: int = logging.INFO
