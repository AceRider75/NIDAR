import os
import csv
import logging
from datetime import datetime
from pathlib import Path
from typing import Optional, Dict, Any
import threading
import time

class MissionLogger:
    """
    Unified mission logging system that manages all logs for a single mission.
    Creates one folder per mission containing:
    - controller.log (flight control, state transitions, commands)
    - telemetry.csv (GPS, attitude, sensor data)
    - spot_tracker.log (detected spots with coordinates)
    - video/ (folder for mission videos)
    """
    
    def __init__(self, base_logs_dir: str = "/home/sajjad/NIDAR/scan_drone/logs"):
        """
        Initialize mission logger (NOT singleton - created per mission)
        
        Args:
            base_logs_dir: Base directory for all missions
        """
        self.base_logs_dir = base_logs_dir
        self.mission_dir = None
        self.mission_id = None
        self.active = False
        
        # File paths
        self.controller_log_file = None
        self.telemetry_csv_file = None
        self.spot_log_file = None
        self.video_dir = None
        
        # Loggers
        self.controller_logger = None
        self.spot_logger = None
        
        # CSV writer
        self.telemetry_csv_writer = None
        self.telemetry_csv_handle = None
        self._csv_lock = threading.Lock()
        
    def start_mission(self) -> str:
        """
        Start a new mission and create logging structure.
            
        Returns:
            Mission directory path
        """
        # Create mission ID from timestamp
        self.mission_id = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.mission_dir = os.path.join(self.base_logs_dir, f"mission_{self.mission_id}")
        
        # Create directories
        os.makedirs(self.mission_dir, exist_ok=True)
        self.video_dir = os.path.join(self.mission_dir, "video")
        os.makedirs(self.video_dir, exist_ok=True)
        
        # Setup file paths
        self.controller_log_file = os.path.join(self.mission_dir, "controller.log")
        self.telemetry_csv_file = os.path.join(self.mission_dir, "telemetry.csv")
        self.spot_log_file = os.path.join(self.mission_dir, "spot_tracker.log")
        
        # Initialize controller logger
        self.controller_logger = logging.getLogger(f"MissionController_{self.mission_id}")
        self.controller_logger.setLevel(logging.INFO)
        self.controller_logger.handlers.clear()
        
        controller_handler = logging.FileHandler(self.controller_log_file, mode='w')
        controller_handler.setFormatter(
            logging.Formatter('%(asctime)s | %(levelname)s | %(message)s')
        )
        self.controller_logger.addHandler(controller_handler)
        
        # Initialize spot tracker logger
        self.spot_logger = logging.getLogger(f"MissionSpot_{self.mission_id}")
        self.spot_logger.setLevel(logging.INFO)
        self.spot_logger.handlers.clear()
        
        spot_handler = logging.FileHandler(self.spot_log_file, mode='w')
        spot_handler.setFormatter(logging.Formatter('%(message)s'))
        self.spot_logger.addHandler(spot_handler)
        
        # Initialize telemetry CSV
        self._init_telemetry_csv()
        
        self.active = True
        self.controller_logger.info(f"Mission started: {self.mission_id}")
        self.controller_logger.info(f"Mission directory: {self.mission_dir}")
        
        return self.mission_dir
    
    def _init_telemetry_csv(self):
        """Initialize telemetry CSV file with headers"""
        with self._csv_lock:
            self.telemetry_csv_handle = open(self.telemetry_csv_file, 'w', newline='')
            self.telemetry_csv_writer = csv.writer(self.telemetry_csv_handle)
            
            # Write headers
            self.telemetry_csv_writer.writerow([
                'timestamp', 'datetime', 'lat', 'lon', 'alt',
                'roll', 'pitch', 'yaw', 'vx', 'vy', 'vz',
                'xacc', 'yacc', 'battery', 'flight_mode', 'armed', 'state'
            ])
            self.telemetry_csv_handle.flush()
    
    def log_controller(self, level: str, message: str):
        """
        Log controller events (state changes, commands, errors)
        
        Args:
            level: Log level (INFO, WARNING, ERROR, CRITICAL)
            message: Log message
        """
        if not self.active or not self.controller_logger:
            return
        
        level_map = {
            'INFO': self.controller_logger.info,
            'WARNING': self.controller_logger.warning,
            'ERROR': self.controller_logger.error,
            'CRITICAL': self.controller_logger.critical,
            'DEBUG': self.controller_logger.debug
        }
        
        log_func = level_map.get(level.upper(), self.controller_logger.info)
        log_func(message)
    
    def log_telemetry(self, telemetry: Dict[str, Any], state: str):
        """
        Log telemetry data to CSV
        
        Args:
            telemetry: Dictionary with telemetry data
            state: Current drone state
        """
        if not self.active or not self.telemetry_csv_writer:
            return
        
        with self._csv_lock:
            try:
                self.telemetry_csv_writer.writerow([
                    telemetry.get('timestamp', time.time()),
                    datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3],
                    telemetry.get('lat', 0),
                    telemetry.get('lon', 0),
                    telemetry.get('alt', 0),
                    telemetry.get('roll', 0),
                    telemetry.get('pitch', 0),
                    telemetry.get('yaw', 0),
                    telemetry.get('vx', 0),
                    telemetry.get('vy', 0),
                    telemetry.get('vz', 0),
                    telemetry.get('xacc', 0),
                    telemetry.get('yacc', 0),
                    telemetry.get('battery', 0),
                    telemetry.get('flight_mode', ''),
                    telemetry.get('armed', False),
                    state
                ])
                self.telemetry_csv_handle.flush()
            except Exception as e:
                print(f"Failed to log telemetry: {e}")
    
    def log_spot(self, spot_id: int, drone_coords: tuple, spot_coords: tuple):
        """
        Log detected spot data
        
        Args:
            spot_id: Spot ID
            drone_coords: (lat, lon, alt) of drone
            spot_coords: (lat, lon) of spot
        """
        if not self.active or not self.spot_logger:
            return
        
        timestamp = datetime.utcnow().strftime("%Y-%m-%d %H:%M:%S")
        
        entry = (
            f"{timestamp} | spot_id={spot_id} | "
            f"drone_lat={drone_coords[0]:.7f}, drone_lon={drone_coords[1]:.7f}, "
            f"drone_alt={drone_coords[2]:.2f} | "
            f"spot_lat={spot_coords[0]:.7f}, spot_lon={spot_coords[1]:.7f}"
        )
        
        self.spot_logger.info(entry)
    
    def get_video_path(self, filename: str) -> str:
        """
        Get path for video file in mission directory
        
        Args:
            filename: Video filename
            
        Returns:
            Full path to video file
        """
        if not self.active or not self.video_dir:
            return None
        return os.path.join(self.video_dir, filename)
    
    def end_mission(self):
        """Close all log files and finalize mission"""
        if not self.active:
            return
            
        if self.controller_logger:
            self.controller_logger.info(f"Mission ended: {self.mission_id}")
        
        # Close telemetry CSV
        with self._csv_lock:
            if self.telemetry_csv_handle:
                self.telemetry_csv_handle.close()
                self.telemetry_csv_handle = None
                self.telemetry_csv_writer = None
        
        # Close logger handlers
        if self.controller_logger:
            for handler in self.controller_logger.handlers:
                handler.close()
            self.controller_logger.handlers.clear()
        
        if self.spot_logger:
            for handler in self.spot_logger.handlers:
                handler.close()
            self.spot_logger.handlers.clear()
        
        self.active = False
        print(f"Mission logs saved to: {self.mission_dir}")
    
    def is_active(self) -> bool:
        """Check if mission logging is active"""
        return self.active