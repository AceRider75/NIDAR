from controller import DroneController
from radio_comm import RadioComm
import time
import os
from config import DroneConfig
# from utils import log_message
from utils import setup_logger
from decimal import Decimal, InvalidOperation

DRONE_NAME = "Sprayer"
PASSWORD = "vihang@2025"

class DroneManager:
    def __init__(self):
        self.radio = RadioComm(port="/dev/ttyAMA0")
        self.radio.start()
        # Buffer for framed waypoint transfer
        self._wp_transfer_active = False
        self._wp_buffer = []
        self._wp_expected_count = 0
        self._wp_altitude = None

        # NEW: Configure controller with KML geofencing
        BASE_DIR = os.path.dirname(os.path.abspath(__file__))
        KML_PATH = os.path.join(BASE_DIR, "data", "JUs.kml")
        
        config = DroneConfig(
            connection_string='/dev/ttyACM0',  # Change to '/dev/ttyACM0' for real hardware
            geofence_mode="polygon",  # or "radius"
            kml_file=KML_PATH,
            polygon_name="Field",
            max_altitude=30.0,
            geofence_radius=500.0,
            waypoint_radius=3.0,
            optimize_waypoint_order=True,
            default_altitude=3.0
        )
        
        self.controller = DroneController(config)
        
        # Connect and start
        if not self.controller.connect():
            print("ERROR: Failed to connect to flight controller")
            return
        
        self.controller.start()
        
        self._last_tx = time.time()

    # -------------------------------------------------
    # COMMAND HANDLING - UPDATED
    # -------------------------------------------------
    def _parse_coord(self, v):
        """
        Parse coordinate values that may arrive as strings to avoid scientific notation issues.
        Returns float without altering the value beyond float precision.
        """
        if isinstance(v, (int, float)):
            return float(v)
        if isinstance(v, str):
            try:
                # Use Decimal to parse fixed-point strings exactly, then convert to float
                return float(Decimal(v))
            except (InvalidOperation, ValueError):
                return float('nan')
        return float('nan')

    def _normalize_waypoint(self, wp):
        """
        Normalize a waypoint to [lat, lon, alt] floats without reformatting.
        """
        if not isinstance(wp, (list, tuple)) or len(wp) < 2:
            return None
        lat = self._parse_coord(wp[0])
        lon = self._parse_coord(wp[1])
        alt = self._parse_coord(wp[2] if len(wp) > 2 else self.controller.config.default_altitude)
        if any(map(lambda x: x != x, [lat, lon, alt])):  # NaN check
            return None
        return [lat, lon, alt]

    def handle_command(self, packet):
        cmd = packet.get("command")
        params = packet.get("params", {})
        
        if cmd == "START":
            waypoints = params.get("waypoints", [])
            altitude = params.get("altitude", 3.0)

            # If framed transfer buffered waypoints exist, prefer them
            if self._wp_transfer_active and self._wp_buffer:
                waypoints = self._wp_buffer.copy()
                # Use provided altitude if any from framing
                if isinstance(self._wp_altitude, (int, float)):
                    altitude = self._wp_altitude

            if waypoints:
                normalized = []
                for wp in waypoints:
                    n = self._normalize_waypoint(wp)
                    if n:
                        normalized.append(n)
                if normalized:
                    self.controller.queue_command(
                        self.controller.load_mission_from_points,
                        normalized,
                        True
                    )
                    self.controller.queue_command(
                        self.controller.add_return_home_waypoint
                    )

            self.controller.queue_command(
                self.controller.arm_and_takeoff,
                altitude
            )

            if waypoints:
                self.controller.queue_command(
                    self.controller.start_mission
                )

            # Clear buffer after starting
            self._wp_transfer_active = False
            self._wp_buffer.clear()
            self._wp_expected_count = 0
            self._wp_altitude = None

        elif cmd == "LAND":
            self.controller.queue_command(self.controller.land)

        elif cmd == "RTL":
            self.controller.queue_command(self.controller.return_to_launch)

        elif cmd == "PAUSE":
            self.controller.pause_mission()
        
        elif cmd == "RESUME":
            self.controller.resume_mission()
        
        elif cmd == "EMERGENCY":
            self.controller.emergency_stop()

        elif cmd == "SET_MODE":
            mode = params.get('mode')
            # Mode handled internally

        # ---- Framed waypoint transfer handling ----
        elif cmd == "LOAD_WAYPOINTS_START":
            # Begin buffered transfer
            self._wp_transfer_active = True
            self._wp_buffer.clear()
            self._wp_expected_count = int(params.get("count", 0) or 0)
            alt_str = params.get("altitude")
            self._wp_altitude = self._parse_coord(alt_str) if alt_str is not None else None
            # Optional: acknowledge start (if RadioComm supports)
            # self.radio.send_packet({"name": DRONE_NAME, "password": PASSWORD, "ack": "LOAD_WAYPOINTS_START"})

        elif cmd == "LOAD_WAYPOINTS_BATCH":
            if self._wp_transfer_active:
                batch = params.get("waypoints", [])
                for wp in batch:
                    n = self._normalize_waypoint(wp)
                    if n:
                        self._wp_buffer.append(n)
            # Optional: ack per batch
            # self.radio.send_packet({"name": DRONE_NAME, "password": PASSWORD, "ack": "LOAD_WAYPOINTS_BATCH", "received": len(batch)})

        elif cmd == "LOAD_WAYPOINTS_END":
            if self._wp_transfer_active:
                # Validate count if provided
                if self._wp_expected_count and len(self._wp_buffer) != self._wp_expected_count:
                    # You can log or handle mismatch; still proceed
                    pass
                # Load mission immediately upon end
                if self._wp_buffer:
                    self.controller.queue_command(
                        self.controller.load_mission_from_points,
                        self._wp_buffer.copy(),
                        True
                    )
                    self.controller.queue_command(
                        self.controller.add_return_home_waypoint
                    )
                # Clear transfer state
                self._wp_transfer_active = False
                self._wp_expected_count = 0

        elif cmd == "LOAD_WAYPOINTS":
            # Legacy single-command transfer
            waypoints = params.get('waypoints', [])
            normalized = []
            for wp in waypoints:
                n = self._normalize_waypoint(wp)
                if n:
                    normalized.append(n)
            if normalized:
                self.controller.queue_command(
                    self.controller.load_mission_from_points,
                    normalized,
                    True
                )

        else:
            print(f"Unknown Command: {cmd}")
            # log_message("RPi", f"Unknown Command: {cmd}\n")
    
    # -------------------------------------------------
    # TELEMETRY TX - UPDATED
    # -------------------------------------------------
    def send_telemetry(self):
        status = self.controller.get_status()
        
        packet = {
            "name": DRONE_NAME,
            "password": PASSWORD,
            "state": status['state'],
            "battery": status['telemetry']['battery'],
            "telemetry": status['telemetry'],
            "mission": status['mission'],
            "geofence": status['geofence'],
            "healthy": status['healthy'],
            "log": "\n".join(status['logs'][-10:])  # Last 10 log messages
        }
        self.radio.send_packet(packet)

    # -------------------------------------------------
    # MAIN LOOP
    # -------------------------------------------------
    def run(self):
        while self.controller.running.is_set():
            cmd = self.radio.get_latest_command()
            if cmd:
                self.handle_command(cmd)

            if time.time() - self._last_tx > 0.5:  # 2 Hz
                self.send_telemetry()
                self._last_tx = time.time()
            
            time.sleep(0.01)
        
        self.controller.stop()

if __name__ == "__main__":
    drone = DroneManager()
    drone.run()