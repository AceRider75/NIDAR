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
    radius: float = 5.0  # meters
    validated: bool = False
    spray_enabled: bool = True  # Enable spraying at this waypoint
    spray_duration: float = 10.0  # Spray duration in seconds
    
    def __str__(self):
        return f"WP({self.lat:.6f}, {self.lon:.6f}, {self.alt:.1f}m, " \
               f"r={self.radius}m, spray={'ON' if self.spray_enabled else 'OFF'})"


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

def generate_spiral_waypoints(center_lat: float, center_lon: float, center_alt: float, 
                             radius: float = 1.0, num_points: int = 12) -> List[Waypoint]:
    """
    Generate waypoints in a smooth outward spiral pattern for continuous movement
    Creates a spiral that starts from near the center and spirals outward smoothly
    
    Args:
        center_lat: Center latitude
        center_lon: Center longitude 
        center_alt: Center altitude
        radius: Maximum spiral radius in meters
        num_points: Number of waypoints in the spiral (more points = smoother movement)
        
    Returns:
        List of Waypoint objects forming a smooth outward spiral pattern
    """
    waypoints = []
    
    # Convert radius to degrees approximately
    lat_deg_per_meter = 1.0 / 111320.0
    lon_deg_per_meter = 1.0 / (111320.0 * math.cos(math.radians(center_lat)))
    
    # Generate smooth spiral points
    for i in range(num_points):
        # Create smooth outward spiral
        # More rotations for better coverage: 2-3 full rotations
        angle = 3 * 2 * math.pi * i / num_points  # 3 full rotations
        
        # Spiral radius increases smoothly from small to max radius
        # Start from 10% of radius to avoid center clustering
        min_radius = radius * 0.1
        current_radius = min_radius + (radius - min_radius) * (i / (num_points - 1))
        
        # Calculate offset in meters
        delta_lat_m = current_radius * math.cos(angle)
        delta_lon_m = current_radius * math.sin(angle)
        
        # Convert to lat/lon
        new_lat = center_lat + (delta_lat_m * lat_deg_per_meter)
        new_lon = center_lon + (delta_lon_m * lon_deg_per_meter)
        
        waypoint = Waypoint(
            lat=new_lat,
            lon=new_lon,
            alt=center_alt,
            radius=1.0,  # Small radius for continuous movement
            spray_enabled=True,
            spray_duration=0.0,  # Not used for continuous spray
            validated=False  # Will be validated later
        )
        
        waypoints.append(waypoint)
    
    return waypoints


def test_spiral_pattern(center_lat: float = 40.7128, center_lon: float = -74.0060, 
                       radius: float = 1.0, num_points: int = 8):
    """
    Test function to visualize spiral pattern generation
    Prints the spiral waypoints for debugging
    """
    print(f"\nGenerating spiral pattern:")
    print(f"Center: ({center_lat:.6f}, {center_lon:.6f})")
    print(f"Radius: {radius}m, Points: {num_points}")
    print("-" * 60)
    
    waypoints = generate_spiral_waypoints(center_lat, center_lon, 10.0, radius, num_points)
    
    for i, wp in enumerate(waypoints):
        dist_from_center = haversine_dist(center_lat, center_lon, wp.lat, wp.lon)
        print(f"Point {i:2d}: ({wp.lat:.6f}, {wp.lon:.6f}) - Distance: {dist_from_center:.2f}m")
    
    print(f"\nTotal waypoints: {len(waypoints)}")
    return waypoints


def log_message(device: str, message: str) -> str:

    os.makedirs(os.path.dirname(LOG_FILE), exist_ok=True)

    with open(LOG_FILE, "a+", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([time.time(), device, message])
    
    logged_message = f"{time.time()} | [{device}]{message}"
    print(logged_message)

    return logged_message
