import threading
import time
import queue
import csv
import os
from enum import Enum, auto
from dataclasses import dataclass, field
from typing import Optional, Tuple, List, Dict, Any
from pymavlink import mavutil
import math
from datetime import datetime
import logging

from config import DroneConfig
from utils import setup_logger, haversine_dist, DroneState, FlightMode, Waypoint
from telemetry import Telemetry
from geofence import GeoFence
from mission_plan import MissionPlanner

class DroneController:
    """
    Main drone controller with state machine architecture

    Architecture:
    - State machine for clear state transitions
    - Thread-safe operations using locks and queues
    - Heartbeat monitoring for connection health
    - Command queue for ordered execution
    - Separate threads for telemetry, heartbeat, and mission control
    - Single message reader pattern (only telemetry thread reads MAVLink)
    """

    def __init__(self, config: DroneConfig = None):
        self.config = config or DroneConfig()
        self.logger = setup_logger(
            'DroneController', self.config.log_file, self.config.log_level)

        # Initialize mission planner
        self.mission_planner: Optional[MissionPlanner] = None
        if self.config.geofence_mode == "polygon" and self.config.kml_file:
            try:
                self.mission_planner = MissionPlanner(
                    kml=self.config.kml_file,
                    polygon_name=self.config.polygon_name
                )
                self.logger.info(
                    f"Mission planner initialized with KML: {self.config.kml_file}")
            except Exception as e:
                self.logger.error(f"Failed to load KML file: {e}")
                self.mission_planner = None

        # Connection
        self.connection: Optional[mavutil.mavlink_connection] = None
        self.connected = threading.Event()
        self.connection_lock = threading.Lock()

        # State management
        self.state = DroneState.DISCONNECTED
        self.state_lock = threading.Lock()
        self.state_history: List[Tuple[float, DroneState]] = []

        # Telemetry
        self.telemetry = Telemetry()
        self.telemetry_lock = threading.Lock()

        # Geofencing
        self.geofence = GeoFence(
            mode=self.config.geofence_mode,
            max_radius=self.config.geofence_radius,
            min_altitude=self.config.min_altitude,
            max_altitude=self.config.max_altitude,
            mission_planner=self.mission_planner
        )

        # Mission management
        self.waypoints: List[Waypoint] = []
        self.current_waypoint_index = -1
        self.mission_lock = threading.Lock()
        self.mission_active = threading.Event()
        self.mission_start_time: float = 0.0

        # ACK tracking (for single reader pattern)
        self.last_command_ack: Optional[Any] = None
        self.command_ack_lock = threading.Lock()
        self.ack_received_event = threading.Event()
        
        # System status tracking (for armable checking)
        self.system_status = mavutil.mavlink.MAV_STATE_UNINIT
        self.system_status_lock = threading.Lock()

        # Command queue for ordered execution
        self.command_queue = queue.Queue(maxsize=100)

        # Threads
        self.running = threading.Event()
        self.threads: List[threading.Thread] = []

        # Health monitoring
        self.last_heartbeat = 0.0
        self.heartbeat_lock = threading.Lock()
        self.health_ok = threading.Event()

        # Emergency flags
        self.emergency_land = threading.Event()
        self.emergency_rtl = threading.Event()

        # Log buffer for UI
        self.log_buffer: List[str] = []
        self.log_buffer_lock = threading.Lock()

        # Telemetry CSV logger
        self.telemetry_log_dir = self.config.telemetry_log_dir
        self.telemetry_log_file = None
        self.telemetry_log_lock = threading.Lock()

        self.logger.info("DroneController initialized")
        self.logger.info(f"Geofence mode: {self.config.geofence_mode}")

    # ==========================================================================
    # CONNECTION MANAGEMENT
    # ==========================================================================

    def connect(self) -> bool:
        """
        Establish connection to flight controller with retry
        Ensures data is actually flowing before returning
        """
        self.logger.info(f"Connecting to {self.config.connection_string}")
        self._add_log(f"Connecting to {self.config.connection_string}")

        for attempt in range(3):
            try:
                # Step 1: Create connection
                self.logger.info(f"Connection attempt {attempt + 1}/3...")
                with self.connection_lock:
                    self.connection = mavutil.mavlink_connection(
                        self.config.connection_string,
                        baud=57600
                    )

                # Step 2: Wait for heartbeat (blocks until received)
                self.logger.info("Waiting for heartbeat...")
                self.connection.wait_heartbeat(timeout=self.config.connection_timeout)
                
                self.logger.info(
                    f"Heartbeat from system {self.connection.target_system}, "
                    f"component {self.connection.target_component}"
                )
                self._add_log(f"Connected to system {self.connection.target_system}")

                # Step 3: Request data streams
                self.logger.info("Requesting data streams...")
                self._request_data_streams()
                
                # Step 4: CRITICAL - Wait for data to actually start flowing
                self.logger.info("Verifying data stream...")
                if not self._verify_data_stream(timeout=10):
                    self.logger.warning("Data stream verification failed, retrying...")
                    with self.connection_lock:
                        if self.connection:
                            try:
                                self.connection.close()
                            except:
                                pass
                        self.connection = None
                    time.sleep(2)
                    continue

                self.logger.info("Waiting for data streams to initialize...")
                time.sleep(3)  # Give streams time to start

                
                # Step 5: Initialize tracking variables
                self.connected.set()
                self._change_state(DroneState.CONNECTED)
                self.last_heartbeat = time.time()
                self.health_ok.set()
                
                # Initialize system status
                with self.system_status_lock:
                    self.system_status = mavutil.mavlink.MAV_STATE_UNINIT

                # Initialize telemetry log NOW (after connection verified)
                self._init_telemetry_log()

                self.logger.info("✓ Connection established and verified")
                return True

            except Exception as e:
                self.logger.error(f"Connection attempt {attempt + 1} failed: {e}")
                with self.connection_lock:
                    if self.connection:
                        try:
                            self.connection.close()
                        except:
                            pass
                    self.connection = None
                time.sleep(2)

        self.logger.error("Failed to connect after 3 attempts")
        self._add_log("ERROR: Failed to connect after 3 attempts")
        return False

    def _request_data_streams(self):
        """Request telemetry data streams from flight controller"""
        with self.connection_lock:
            if not self.connection:
                return
            
            # Request all data streams at specified rate
            self.connection.mav.request_data_stream_send(
                self.connection.target_system,
                self.connection.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_ALL,
                self.config.telemetry_rate_hz,
                1  # Start streaming
            )
            
            # Also request individual important streams to be sure
            # This is redundant but ensures we get critical data
            for stream_id in [
                mavutil.mavlink.MAV_DATA_STREAM_POSITION,
                mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,  # ATTITUDE
                mavutil.mavlink.MAV_DATA_STREAM_RAW_SENSORS,
            ]:
                self.connection.mav.request_data_stream_send(
                    self.connection.target_system,
                    self.connection.target_component,
                    stream_id,
                    self.config.telemetry_rate_hz,
                    1
                )
        
        self.logger.debug(f"Data streams requested at {self.config.telemetry_rate_hz} Hz")

    def _verify_data_stream(self, timeout: float = 10) -> bool:
        """
        CRITICAL: Verify that telemetry data is actually flowing
        This prevents the telemetry loop from starting before data arrives
        
        Returns:
            True if GPS and attitude data received within timeout
        """
        self.logger.info("Verifying data stream reception...")
        
        start = time.time()
        received_messages = {
            'HEARTBEAT': False,
            'GLOBAL_POSITION_INT': False,
            'ATTITUDE': False,
            'SYS_STATUS': False
        }
        
        with self.connection_lock:
            conn = self.connection
        
        while time.time() - start < timeout:
            try:
                # Read messages synchronously during verification
                msg = conn.recv_match(blocking=True, timeout=1.0)
                
                if msg:
                    msg_type = msg.get_type()
                    
                    if msg_type in received_messages:
                        received_messages[msg_type] = True
                        self.logger.debug(f"✓ Received {msg_type}")
                    
                    # Check if we got all critical messages
                    if all(received_messages.values()):
                        self.logger.info("✓ All critical data streams verified")
                        return True
                    
            except Exception as e:
                self.logger.error(f"Error during data stream verification: {e}")
                return False
        
        # Report what we got
        missing = [k for k, v in received_messages.items() if not v]
        if missing:
            self.logger.warning(f"Missing message types: {missing}")
        
        # Consider it OK if we at least got GPS data
        if received_messages['GLOBAL_POSITION_INT']:
            self.logger.info("✓ GPS data verified (partial)")
            return True
        
        self.logger.error("✗ Data stream verification failed")
        return False

    # ==========================================================================
    # STATE MACHINE
    # ==========================================================================

    def _change_state(self, new_state: DroneState):
        """Thread-safe state transition with logging"""
        with self.state_lock:
            old_state = self.state
            self.state = new_state
            self.state_history.append((time.time(), new_state))

            # Keep only last 100 state changes
            if len(self.state_history) > 100:
                self.state_history = self.state_history[-100:]

        self.logger.info(
            f"State transition: {old_state.name} -> {new_state.name}")
        self._add_log(f"State: {new_state.name}")

    def get_state(self) -> DroneState:
        """Get current state"""
        with self.state_lock:
            return self.state

    def _add_log(self, message: str):
        """Add message to log buffer for UI"""
        with self.log_buffer_lock:
            timestamp = datetime.now().strftime("%H:%M:%S")
            self.log_buffer.append(f"[{timestamp}] {message}")
            # Keep only last 100 logs
            if len(self.log_buffer) > 100:
                self.log_buffer = self.log_buffer[-100:]

    # ==========================================================================
    # THREAD MANAGEMENT
    # ==========================================================================

    def start(self):
        """Start all background threads"""
        if not self.connected.is_set():
            self.logger.error("Cannot start - not connected")
            return False

        self.running.set()

        # Telemetry thread
        t1 = threading.Thread(target=self._telemetry_loop,
                              name="Telemetry", daemon=True)
        t1.start()
        self.threads.append(t1)

        # Heartbeat monitor thread
        t2 = threading.Thread(target=self._heartbeat_monitor,
                              name="Heartbeat", daemon=True)
        t2.start()
        self.threads.append(t2)

        # Command processor thread
        t3 = threading.Thread(target=self._command_processor,
                              name="CommandProc", daemon=True)
        t3.start()
        self.threads.append(t3)

        # Mission controller thread
        t4 = threading.Thread(target=self._mission_controller,
                              name="MissionCtrl", daemon=True)
        t4.start()
        self.threads.append(t4)

        self.logger.info("All threads started")
        self._change_state(DroneState.IDLE)
        return True

    def stop(self):
        """Stop all threads gracefully"""
        self.logger.info("Stopping controller...")
        self._add_log("Shutting down...")
        self.running.clear()
        self.mission_active.clear()

        # Wait for threads to finish
        for thread in self.threads:
            thread.join(timeout=2.0)

        # Close connection
        with self.connection_lock:
            if self.connection:
                self.connection.close()

        self.logger.info("Controller stopped")

    # ==========================================================================
    # TELEMETRY LOGGING
    # ==========================================================================

    def _init_telemetry_log(self):
        """Initialize telemetry CSV log file with headers"""
        try:
            # Ensure directory exists
            if self.telemetry_log_dir and not os.path.exists(self.telemetry_log_dir):
                os.makedirs(self.telemetry_log_dir)
            
            # Write CSV headers
            log_file_name = f"telemetry_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
            self.telemetry_log_file = os.path.join(self.telemetry_log_dir, log_file_name)

            with open(self.telemetry_log_file, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    'timestamp', 'datetime', 'lat', 'lon', 'alt',
                    'roll', 'pitch', 'yaw', 'vx', 'vy', 'vz',
                    'xacc', 'yacc', 'battery', 'flight_mode', 'armed', 'state'
                ])
            self.logger.info(f"Telemetry log initialized: {self.telemetry_log_file}")
        except Exception as e:
            self.logger.error(f"Failed to initialize telemetry log: {e}")

    def _log_telemetry(self):
        """Log current telemetry to CSV file"""
        if not self.telemetry_log_file:
            return
            
        try:
            with self.telemetry_log_lock:
                with open(self.telemetry_log_file, 'a', newline='') as f:
                    writer = csv.writer(f)
                    with self.telemetry_lock:
                        writer.writerow([
                            self.telemetry.timestamp,
                            datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3],
                            self.telemetry.lat,
                            self.telemetry.lon,
                            self.telemetry.alt,
                            self.telemetry.roll,
                            self.telemetry.pitch,
                            self.telemetry.yaw,
                            self.telemetry.vx,
                            self.telemetry.vy,
                            self.telemetry.vz,
                            self.telemetry.xacc,
                            self.telemetry.yacc,
                            self.telemetry.battery,
                            self.telemetry.flight_mode,
                            self.telemetry.armed,
                            self.state.name
                        ])
        except Exception as e:
            self.logger.error(f"Failed to log telemetry: {e}")

    # ==========================================================================
    # TELEMETRY THREAD - ONLY PLACE THAT READS FROM MAVLINK
    # ==========================================================================

    def _telemetry_loop(self):
        """
        Background thread for receiving telemetry
        CRITICAL: ONLY place that reads from MAVLink connection (single reader pattern)
        """
        self.logger.info("Telemetry loop started")
        
        # Statistics
        msg_count = 0
        msg_types_received = {}
        last_log_time = time.time()
        last_debug_time = time.time()
        last_csv_time = time.time()
        
        # Track if we're receiving data
        first_message_received = False

        while self.running.is_set():
            try:
                # Get connection
                with self.connection_lock:
                    conn = self.connection
                
                if not conn:
                    if not first_message_received:
                        self.logger.warning("No connection in telemetry loop")
                    time.sleep(0.1)
                    continue

                # ============================================================
                # ONLY PLACE IN ENTIRE CODEBASE THAT CALLS recv_match()
                # ============================================================
                msg = conn.recv_match(blocking=True, timeout=1.0)
                
                if not msg:
                    # Log if we're not getting messages
                    now = time.time()
                    if now - last_log_time > 5.0:
                        self.logger.warning("No messages received in 5 seconds")
                        last_log_time = now
                    continue
                
                # First message received
                if not first_message_received:
                    self.logger.info("✓ First telemetry message received")
                    first_message_received = True

                msg_type = msg.get_type()
                msg_count += 1
                now = time.time()
                
                # Track message types
                msg_types_received[msg_type] = msg_types_received.get(msg_type, 0) + 1

                # Process based on message type
                if msg_type == "SYS_STATUS":
                    with self.telemetry_lock:
                        self.telemetry.timestamp = now
                        self.telemetry.battery = msg.battery_remaining

                elif msg_type == "GLOBAL_POSITION_INT":
                    with self.telemetry_lock:
                        self.telemetry.timestamp = now
                        self.telemetry.lat = msg.lat / 1e7
                        self.telemetry.lon = msg.lon / 1e7
                        self.telemetry.alt = msg.relative_alt / 1000.0
                        self.telemetry.vx = msg.vx / 100.0
                        self.telemetry.vy = msg.vy / 100.0
                        self.telemetry.vz = msg.vz / 100.0

                elif msg_type == "ATTITUDE":
                    with self.telemetry_lock:
                        self.telemetry.timestamp = now
                        self.telemetry.roll = math.degrees(msg.roll)
                        self.telemetry.pitch = math.degrees(msg.pitch)
                        self.telemetry.yaw = math.degrees(msg.yaw)

                elif msg_type == "RAW_IMU":
                    with self.telemetry_lock:
                        self.telemetry.timestamp = now
                        self.telemetry.xacc = msg.xacc / 1000.0
                        self.telemetry.yacc = msg.yacc / 1000.0

                elif msg_type == "HEARTBEAT":
                    # Update heartbeat timestamp
                    with self.heartbeat_lock:
                        self.last_heartbeat = now
                    
                    # Update system status for armable checking
                    with self.system_status_lock:
                        self.system_status = msg.system_status
                    
                    # Update telemetry
                    with self.telemetry_lock:
                        self.telemetry.timestamp = now
                        self.telemetry.flight_mode = conn.flightmode
                        self.telemetry.armed = bool(
                            msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                        )

                elif msg_type == "COMMAND_ACK":
                    # Store for waiting threads (single reader pattern)
                    with self.command_ack_lock:
                        self.last_command_ack = msg
                        self.ack_received_event.set()
                    
                    self.logger.debug(f"ACK: cmd={msg.command}, result={msg.result}")

                # Log to CSV at fixed rate (10 Hz)
                if now - last_csv_time >= 0.1:
                    self._log_telemetry()
                    last_csv_time = now

                # Debug output every 10 seconds
                if now - last_debug_time >= 10.0:
                    with self.telemetry_lock:
                        self.logger.info(
                            f"Telemetry: {msg_count} msgs | "
                            f"GPS: ({self.telemetry.lat:.6f}, {self.telemetry.lon:.6f}, {self.telemetry.alt:.2f}m) | "
                            f"Battery: {self.telemetry.battery}% | "
                            f"Mode: {self.telemetry.flight_mode}"
                        )
                    
                    # Show message type distribution
                    top_types = sorted(msg_types_received.items(), key=lambda x: x[1], reverse=True)[:5]
                    self.logger.debug(f"Top message types: {dict(top_types)}")
                    
                    last_debug_time = now

            except Exception as e:
                self.logger.exception(f"Telemetry loop error: {e}")
                time.sleep(0.1)

        self.logger.info(f"Telemetry loop stopped. Total messages: {msg_count}")
        self.logger.info(f"Message type distribution: {dict(sorted(msg_types_received.items()))}")

    # ==========================================================================
    # HEARTBEAT MONITOR
    # ==========================================================================

    def _heartbeat_monitor(self):
        """Monitor connection health via heartbeat"""
        self.logger.info("Heartbeat monitor started")

        while self.running.is_set():
            try:
                with self.heartbeat_lock:
                    time_since_heartbeat = time.time() - self.last_heartbeat

                if time_since_heartbeat > self.config.heartbeat_timeout:
                    self.logger.warning(
                        f"No heartbeat for {time_since_heartbeat:.1f}s")
                    self.health_ok.clear()

                    # Trigger emergency if in flight
                    if self.get_state() in [DroneState.MISSION_ACTIVE, DroneState.TAKING_OFF]:
                        self.logger.critical(
                            "Connection lost during flight - emergency RTL")
                        self._add_log(
                            "CRITICAL: Connection lost - emergency RTL")
                        self.emergency_rtl.set()
                else:
                    if not self.health_ok.is_set():
                        self.logger.info("Heartbeat restored")
                        self._add_log("Connection restored")
                        self.health_ok.set()

                time.sleep(1.0)

            except Exception as e:
                self.logger.error(f"Heartbeat monitor error: {e}")
                time.sleep(1.0)

        self.logger.info("Heartbeat monitor stopped")

    # ==========================================================================
    # COMMAND PROCESSOR
    # ==========================================================================

    def _command_processor(self):
        """Process commands from queue"""
        self.logger.info("Command processor started")

        while self.running.is_set():
            try:
                # Get command with timeout
                try:
                    cmd_func, args, kwargs = self.command_queue.get(
                        timeout=0.5)
                except queue.Empty:
                    continue

                # Execute command
                try:
                    cmd_func(*args, **kwargs)
                except Exception as e:
                    self.logger.error(f"Command execution failed: {e}")
                    self._add_log(f"ERROR: {e}")
                finally:
                    self.command_queue.task_done()

            except Exception as e:
                self.logger.error(f"Command processor error: {e}")
                time.sleep(0.1)

        self.logger.info("Command processor stopped")

    def queue_command(self, cmd_func, *args, **kwargs) -> bool:
        """Add command to execution queue"""
        try:
            self.command_queue.put((cmd_func, args, kwargs), block=False)
            return True
        except queue.Full:
            self.logger.error("Command queue full")
            return False

    # ==========================================================================
    # MISSION CONTROLLER
    # ==========================================================================

    def _mission_controller(self):
        """Main mission control loop"""
        self.logger.info("Mission controller started")

        while self.running.is_set():
            try:
                if not self.mission_active.is_set():
                    time.sleep(0.1)
                    continue

                # Check for emergency conditions
                if self.emergency_rtl.is_set():
                    self.logger.critical("Emergency RTL triggered")
                    self.mission_active.clear()
                    self._execute_rtl()
                    self.emergency_rtl.clear()
                    continue

                if self.emergency_land.is_set():
                    self.logger.critical("Emergency land triggered")
                    self._execute_land()
                    self.emergency_land.clear()
                    continue

                # Check mission timeout
                if time.time() - self.mission_start_time > self.config.mission_timeout:
                    self.logger.warning("Mission timeout - returning home")
                    self._add_log("Mission timeout - RTL")
                    self._execute_rtl()
                    continue

                # Check geofence continuously during mission
                with self.telemetry_lock:
                    current_lat = self.telemetry.lat
                    current_lon = self.telemetry.lon
                    current_alt = self.telemetry.alt

                valid, msg = self.geofence.is_within_bounds(
                    current_lat, current_lon, current_alt)
                if not valid:
                    self.logger.error(f"Geofence violation: {msg}")
                    self._add_log(f"GEOFENCE VIOLATION: {msg}")
                    self.emergency_rtl.set()
                    continue

                # Process waypoints
                with self.mission_lock:
                    if self.current_waypoint_index >= len(self.waypoints):
                        self.logger.info("Mission complete")
                        self._add_log("Mission complete")
                        self.mission_active.clear()
                        continue

                    waypoint = self.waypoints[self.current_waypoint_index]

                # Check if waypoint reached
                with self.telemetry_lock:
                    current_pos = (self.telemetry.lat,
                                   self.telemetry.lon, self.telemetry.alt)

                dist_h = haversine_dist(
                    current_pos[0], current_pos[1], waypoint.lat, waypoint.lon)
                dist_v = abs(current_pos[2] - waypoint.alt)

                if dist_h <= waypoint.radius and dist_v <= self.config.altitude_tolerance:
                    self.logger.info(
                        f"Waypoint {self.current_waypoint_index + 1}/{len(self.waypoints)} reached")
                    self._add_log(
                        f"Waypoint {self.current_waypoint_index + 1}/{len(self.waypoints)} reached")

                    with self.mission_lock:
                        self.current_waypoint_index += 1

                    # Send next waypoint
                    if self.current_waypoint_index < len(self.waypoints):
                        next_wp = self.waypoints[self.current_waypoint_index]
                        self._send_waypoint(next_wp)
                    else:
                        # Mission complete
                        self.logger.info("All waypoints reached")
                        self._add_log("All waypoints reached - landing")
                        self.mission_active.clear()
                        self._execute_land()

                time.sleep(0.1)

            except Exception as e:
                self.logger.error(f"Mission controller error: {e}")
                time.sleep(0.5)

        self.logger.info("Mission controller stopped")

    # ==========================================================================
    # HIGH-LEVEL COMMANDS - MISSION PLANNING
    # ==========================================================================

    def load_mission_from_points(
        self,
        points: List[Tuple[float, float, float]],
        validate_geofence: bool = True
    ) -> bool:
        """
        Load mission from list of waypoint coordinates

        Args:
            points: List of (lat, lon, alt) tuples
            validate_geofence: Whether to validate against geofence before loading

        Returns:
            True if mission loaded successfully
        """
        if self.config.geofence_mode == "polygon" and self.mission_planner:
            # Use mission planner to validate and optimize
            self.logger.info(f"Loading mission with {len(points)} points")

            # Extract lat/lon pairs (remove altitude for validation)
            lat_lon_points = [(lat, lon) for lat, lon, alt in points]

            # Get current position for optimization
            with self.telemetry_lock:
                start_lat = self.telemetry.lat
                start_lon = self.telemetry.lon
                self.logger.info(f"Starting position: lat={start_lat:.6f}, lon={start_lon:.6f}")

            # Validate and optimize using mission planner
            if self.config.optimize_waypoint_order:
                optimized_points = self.mission_planner.generate_mission_from_points(
                    lat_lon_points,
                    start_lat=start_lat,
                    start_lon=start_lon
                )
            else:
                # Just validate without optimization
                optimized_points = []
                for lat, lon in lat_lon_points:
                    if self.mission_planner.is_point_inside(lat, lon):
                        optimized_points.append((lat, lon))
                    else:
                        self.logger.warning(
                            f"Point outside polygon: ({lat:.6f}, {lon:.6f})")

            if not optimized_points:
                self.logger.error(
                    "No valid waypoints after geofence validation")
                self._add_log("ERROR: No valid waypoints inside geofence")
                return False

            # Add altitude back to optimized points
            waypoints_with_alt = []
            for lat, lon in optimized_points:
                # Find matching altitude from original points
                # Use first point's altitude if not found (fallback)
                alt = self.config.default_altitude
                for orig_lat, orig_lon, orig_alt in points:
                    if abs(orig_lat - lat) < 1e-6 and abs(orig_lon - lon) < 1e-6:
                        alt = orig_alt
                        break
                waypoints_with_alt.append((lat, lon, alt))

            self.logger.info(
                f"Mission validated: {len(waypoints_with_alt)} waypoints")
            self._add_log(
                f"Mission loaded: {len(waypoints_with_alt)} waypoints")

            # Convert to Waypoint objects
            with self.mission_lock:
                self.waypoints.clear()
                for lat, lon, alt in waypoints_with_alt:
                    wp = Waypoint(
                        lat=lat,
                        lon=lon,
                        alt=alt,
                        radius=self.config.waypoint_radius,
                        validated=True
                    )
                    self.waypoints.append(wp)

                self.current_waypoint_index = 0

            return True

        else:
            # Radius mode or no mission planner - just validate each point
            with self.mission_lock:
                self.waypoints.clear()

                for lat, lon, alt in points:
                    if validate_geofence:
                        valid, msg = self.geofence.is_within_bounds(
                            lat, lon, alt)
                        if not valid:
                            self.logger.warning(f"Waypoint rejected: {msg}")
                            continue

                    wp = Waypoint(
                        lat=lat,
                        lon=lon,
                        alt=alt,
                        radius=self.config.waypoint_radius,
                        validated=validate_geofence
                    )
                    self.waypoints.append(wp)

                self.current_waypoint_index = 0

            if not self.waypoints:
                self.logger.error("No valid waypoints loaded")
                return False

            self.logger.info(
                f"Mission loaded: {len(self.waypoints)} waypoints")
            self._add_log(f"Mission loaded: {len(self.waypoints)} waypoints")
            return True

    def add_return_home_waypoint(self) -> bool:
        """Add return-to-home waypoint at the end of mission"""
        if self.geofence.mode == "radius":
            home_lat = self.geofence.home_lat
            home_lon = self.geofence.home_lon
        else:
            # For polygon mode, use first waypoint or current position
            with self.mission_lock:
                if self.waypoints:
                    home_lat = self.waypoints[0].lat
                    home_lon = self.waypoints[0].lon
                else:
                    with self.telemetry_lock:
                        home_lat = self.telemetry.lat
                        home_lon = self.telemetry.lon
        
        with self.mission_lock:
            home_wp = Waypoint(
                lat=home_lat,
                lon=home_lon,
                alt=self.config.default_altitude,
                radius=self.config.waypoint_radius,
                validated=True
            )
            self.waypoints.append(home_wp)
        
        self.logger.info(f"Return home waypoint added: ({home_lat:.6f}, {home_lon:.6f})")
        return True

    # ==========================================================================
    # HIGH-LEVEL COMMANDS - FLIGHT OPERATIONS
    # ==========================================================================
    
    def arm_and_takeoff(self, altitude: float = None) -> bool:
        """Arm and takeoff sequence"""
        altitude = altitude or self.config.default_altitude
        
        self.logger.info(f"Starting arm and takeoff to {altitude}m")
        self._add_log(f"Arming and taking off to {altitude}m")
        
        # Check if already in appropriate state
        if self.get_state() not in [DroneState.IDLE, DroneState.CONNECTED]:
            self.logger.error(f"Cannot arm from state {self.get_state().name}")
            return False
        
        # Set home position for geofence
        with self.telemetry_lock:
            if self.geofence.mode == "radius":
                self.geofence.home_lat = self.telemetry.lat
                self.geofence.home_lon = self.telemetry.lon
                self.logger.info(f"Home set: {self.geofence.home_lat:.6f}, {self.geofence.home_lon:.6f}")
            else:
                self.logger.info("Using polygon geofence from KML")
        
        # Change to GUIDED mode
        if not self._set_mode(FlightMode.GUIDED):
            return False
        
        # Wait for armable
        self._change_state(DroneState.ARMING)
        if not self._wait_for_armable(timeout=30):
            self.logger.error("Drone not armable")
            self._add_log("ERROR: Drone not armable")
            self._change_state(DroneState.ERROR)
            return False
        
        # Arm
        if not self._arm():
            self._change_state(DroneState.ERROR)
            return False
        
        self._change_state(DroneState.ARMED)
        
        # Takeoff
        self._change_state(DroneState.TAKING_OFF)
        if not self._takeoff(altitude):
            self._change_state(DroneState.ERROR)
            return False
        
        # Wait for altitude
        if not self._wait_for_altitude(altitude, timeout=60):
            self.logger.error("Takeoff altitude not reached")
            self._add_log("WARNING: Takeoff altitude not fully reached")
            # Don't fail completely, might be close enough
        
        self.logger.info("Arm and takeoff complete")
        self._add_log("Takeoff complete")
        self._change_state(DroneState.ARMED)
        return True
    
    def start_mission(self) -> bool:
        """Start loaded waypoint mission"""
        if self.get_state() != DroneState.ARMED:
            self.logger.error("Drone must be armed and at altitude")
            return False
        
        with self.mission_lock:
            if not self.waypoints:
                self.logger.error("No waypoints loaded")
                self._add_log("ERROR: No waypoints loaded")
                return False
            
            num_waypoints = len(self.waypoints)
        
        self.logger.info(f"Starting mission with {num_waypoints} waypoints")
        self._add_log(f"Starting mission: {num_waypoints} waypoints")
        
        # Start mission
        self.mission_start_time = time.time()
        self.mission_active.set()
        self._change_state(DroneState.MISSION_ACTIVE)
        
        # Send first waypoint
        if self.waypoints:
            self._send_waypoint(self.waypoints[0])
        
        return True
    
    def land(self) -> bool:
        """Land at current position"""
        self.logger.info("Landing initiated")
        self._add_log("Landing...")
        self.mission_active.clear()
        return self._execute_land()
    
    def return_to_launch(self) -> bool:
        """Return to launch point and land"""
        self.logger.info("RTL initiated")
        self._add_log("Returning to launch...")
        self.mission_active.clear()
        return self._execute_rtl()
    
    def emergency_stop(self):
        """Emergency landing"""
        self.logger.critical("EMERGENCY STOP")
        self._add_log("EMERGENCY STOP")
        self.emergency_land.set()
    
    def pause_mission(self):
        """Pause current mission"""
        if self.get_state() == DroneState.MISSION_ACTIVE:
            self.mission_active.clear()
            self._set_mode(FlightMode.LOITER)
            self._change_state(DroneState.MISSION_PAUSED)
            self.logger.info("Mission paused")
            self._add_log("Mission paused")
    
    def resume_mission(self):
        """Resume paused mission"""
        if self.get_state() == DroneState.MISSION_PAUSED:
            self._set_mode(FlightMode.GUIDED)
            self.mission_active.set()
            self._change_state(DroneState.MISSION_ACTIVE)
            self.logger.info("Mission resumed")
            self._add_log("Mission resumed")

    # ==========================================================================
    # LOW-LEVEL COMMANDS
    # ==========================================================================

    def _wait_for_armable(self, timeout: float = 30) -> bool:
        """
        Wait for drone to become armable
        Uses shared state (single reader pattern - no recv_match)
        """
        start = time.time()
        self.logger.info("Waiting for drone to become armable...")
        
        while time.time() - start < timeout:
            # Read from shared state updated by telemetry loop
            with self.system_status_lock:
                status = self.system_status
            
            if status == mavutil.mavlink.MAV_STATE_STANDBY:
                self.logger.info("✓ Drone is armable")
                return True
            
            # Log progress every 5 seconds
            elapsed = int(time.time() - start)
            if elapsed % 5 == 0 and elapsed > 0:
                self.logger.info(f"Still waiting for armable... ({elapsed}s, status={status})")
            
            time.sleep(0.5)
        
        self.logger.error(f"Armable timeout after {timeout}s")
        return False
    
    def _arm(self) -> bool:
        """Arm the drone with retry"""
        for attempt in range(self.config.max_arm_retries):
            self.logger.info(f"Arming attempt {attempt + 1}/{self.config.max_arm_retries}")
            
            with self.connection_lock:
                self.connection.mav.command_long_send(
                    self.connection.target_system,
                    self.connection.target_component,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                    0, 1, 0, 0, 0, 0, 0, 0
                )
            
            ack = self._wait_for_ack(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM)
            
            if ack and ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                self.logger.info("Armed successfully")
                self._add_log("Armed")
                return True
            
            if ack:
                self.logger.warning(f"Arm rejected: {ack.result}")
            
            time.sleep(1)
        
        self.logger.error("Failed to arm after retries")
        self._add_log("ERROR: Failed to arm")
        return False
    
    def _takeoff(self, altitude: float) -> bool:
        """Execute takeoff command with retry"""
        for attempt in range(self.config.max_takeoff_retries):
            self.logger.info(f"Takeoff attempt {attempt + 1}/{self.config.max_takeoff_retries}")
            
            with self.connection_lock:
                self.connection.mav.command_long_send(
                    self.connection.target_system,
                    self.connection.target_component,
                    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                    0, 0, 0, 0, float('nan'), float('nan'), float('nan'), altitude
                )
            
            ack = self._wait_for_ack(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF)
            
            if ack and ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                self.logger.info("Takeoff command accepted")
                self._add_log("Climbing...")
                return True
            
            if ack:
                self.logger.warning(f"Takeoff rejected: {ack.result}")
            
            time.sleep(2)
        
        self.logger.error("Failed to takeoff after retries")
        self._add_log("ERROR: Takeoff failed")
        return False
    
    def _wait_for_altitude(self, target_alt: float, timeout: float = 60) -> bool:
        """Wait for drone to reach target altitude"""
        start = time.time()
        
        while time.time() - start < timeout:
            with self.telemetry_lock:
                current_alt = self.telemetry.alt
            
            if abs(current_alt - target_alt) <= self.config.altitude_tolerance:
                self.logger.info(f"Target altitude reached: {current_alt:.2f}m")
                return True
            
            time.sleep(0.5)
        
        with self.telemetry_lock:
            current_alt = self.telemetry.alt
        
        self.logger.error(f"Altitude timeout. Current: {current_alt:.2f}m, Target: {target_alt:.2f}m")
        return False
    
    def _send_waypoint(self, waypoint: Waypoint):
        """Send waypoint to flight controller"""
        self.logger.info(f"Sending waypoint: ({waypoint.lat:.6f}, {waypoint.lon:.6f}, {waypoint.alt:.2f})")
        
        # Validate geofence one more time before sending
        valid, msg = self.geofence.is_within_bounds(waypoint.lat, waypoint.lon, waypoint.alt)
        if not valid:
            self.logger.error(f"Waypoint violates geofence: {msg}")
            self._add_log(f"ERROR: Waypoint violates geofence")
            self.emergency_rtl.set()
            return
        
        with self.connection_lock:
            self.connection.mav.set_position_target_global_int_send(
                0,
                self.connection.target_system,
                self.connection.target_component,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                0b110111111000,
                int(waypoint.lat * 1e7),
                int(waypoint.lon * 1e7),
                waypoint.alt,
                0, 0, 0, 0, 0, 0, 0, 0
            )
    
    def _set_mode(self, mode: FlightMode) -> bool:
        """Set flight mode with retry"""
        with self.connection_lock:
            mapping = self.connection.mode_mapping()
        
        if mode.value not in mapping:
            self.logger.error(f"Mode {mode.value} not supported")
            return False
        
        for attempt in range(self.config.max_mode_change_retries):
            self.logger.info(f"Setting mode to {mode.value} (attempt {attempt + 1})")
            
            with self.connection_lock:
                self.connection.mav.set_mode_send(
                    self.connection.target_system,
                    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                    mapping[mode.value]
                )
            
            # Wait and verify mode change
            time.sleep(1)
            with self.telemetry_lock:
                current_mode = self.telemetry.flight_mode
            
            if current_mode == mode.value:
                self.logger.info(f"Mode changed to {mode.value}")
                return True
            
            time.sleep(1)
        
        self.logger.error(f"Failed to change mode to {mode.value}")
        return False
    
    def _execute_land(self) -> bool:
        """Execute landing sequence"""
        self._change_state(DroneState.LANDING)
        
        if self._set_mode(FlightMode.LAND):
            self.logger.info("Landing in progress")
            return True
        else:
            self.logger.error("Failed to enter LAND mode")
            return False
    
    def _execute_rtl(self) -> bool:
        """Execute return to launch"""
        self._change_state(DroneState.RETURNING_HOME)
        
        if self._set_mode(FlightMode.RTL):
            self.logger.info("RTL in progress")
            return True
        else:
            self.logger.error("Failed to enter RTL mode")
            return False
        
    def _wait_for_ack(self, command: int, timeout: float = None) -> Optional[Any]:
        """
        Wait for command acknowledgement
        Uses Event and shared state (single reader pattern - no recv_match)
        """
        timeout = timeout or self.config.command_ack_timeout
        
        # Clear previous state
        with self.command_ack_lock:
            self.last_command_ack = None
            self.ack_received_event.clear()
        
        start = time.time()
        while time.time() - start < timeout:
            # Wait for event (set by telemetry loop)
            if self.ack_received_event.wait(timeout=0.1):
                with self.command_ack_lock:
                    ack = self.last_command_ack
                    if ack and ack.command == command:
                        self.logger.debug(f"✓ ACK received for command {command}")
                        return ack
                    # Wrong command, keep waiting
                    self.ack_received_event.clear()
        
        self.logger.warning(f"✗ ACK timeout for command {command}")
        return None

    # ==========================================================================
    # PUBLIC API
    # ==========================================================================
    
    def get_telemetry(self) -> Dict[str, Any]:
        """Get current telemetry data"""
        with self.telemetry_lock:
            return self.telemetry.to_dict()
    
    def get_status(self) -> Dict[str, Any]:
        """Get comprehensive status"""
        with self.telemetry_lock:
            telem = self.telemetry.to_dict()
        
        with self.mission_lock:
            mission_info = {
                'active': self.mission_active.is_set(),
                'waypoint_index': self.current_waypoint_index,
                'total_waypoints': len(self.waypoints),
                'progress': f"{self.current_waypoint_index}/{len(self.waypoints)}"
            }
        
        with self.log_buffer_lock:
            recent_logs = list(self.log_buffer[-20:])  # Last 20 logs
        
        return {
            'state': self.get_state().name,
            'connected': self.connected.is_set(),
            'healthy': self.health_ok.is_set(),
            'telemetry': telem,
            'mission': mission_info,
            'geofence': {
                'mode': self.geofence.mode,
                'enabled': self.geofence.enabled,
                'home': (self.geofence.home_lat, self.geofence.home_lon) if self.geofence.mode == "radius" else None,
                'radius': self.geofence.max_radius if self.geofence.mode == "radius" else None,
                'polygon_points': len(self.geofence.polygon_points) if self.geofence.mode == "polygon" else None
            },
            'logs': recent_logs
        }
    
    def set_geofence_mode(self, mode: str):
        """Change geofence mode (polygon or radius)"""
        if mode not in ["polygon", "radius"]:
            self.logger.error(f"Invalid geofence mode: {mode}")
            return False
        
        self.geofence.mode = mode
        self.logger.info(f"Geofence mode set to: {mode}")
        self._add_log(f"Geofence mode: {mode}")
        return True