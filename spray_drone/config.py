from dataclasses import dataclass, field
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
    
    # Geofence configuration
    geofence_mode: str = "polygon"  # "polygon" or "radius"
    geofence_radius: float = 500.0  # meters from home (used if mode="radius")
    kml_file: str = "data/JU.kml"  # Path to KML file (used if mode="polygon")
    polygon_name: str = "Field"  # Name of polygon in KML
    
    # Mission parameters
    waypoint_radius: float = 5.0  # meters
    altitude_tolerance: float = 1.0
    default_altitude: float = 3.0
    mission_timeout: int = 1800  # 30 minutes
    optimize_waypoint_order: bool = True  # Use nearest-neighbor optimization
    
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

