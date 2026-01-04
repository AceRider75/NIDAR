from controller import DroneController
from radio_comm import RadioComm
import time
import os
from config import DroneConfig
# from utils import log
DRONE_NAME = "Sprayer"
PASSWORD = "vihang@2025"

class DroneManager:
    def __init__(self):
        self.radio = RadioComm(port="/dev/ttyAMA0")
        self.radio.start()

        # NEW: Configure controller with KML geofencing
        BASE_DIR = os.path.dirname(os.path.abspath(__file__))
        KML_PATH = os.path.join(BASE_DIR, "data", "JUs.kml")
        
        config = DroneConfig(
            connection_string='127.0.0.1:14551',  # Change to '/dev/ttyACM0' for real hardware
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
    def handle_command(self, packet):
        cmd = packet.get("command")
        params = packet.get("params", {})
        
        # log_message("RPi", f"Received command: {cmd}\n")

        if cmd == "START":
            # NEW: Load mission from params, then start
            waypoints = params.get("waypoints", [])
            altitude = params.get("altitude", 3.0)
            
            if waypoints:
                # Waypoints should be list of [lat, lon, alt]
                self.controller.queue_command(
                    self.controller.load_mission_from_points,
                    waypoints,
                    True  # validate_geofence
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
            # Mode changes are handled internally by state machine
            log_message("RPi", f"Mode change requests handled by state machine\n")

        elif cmd == "LOAD_WAYPOINTS":
            waypoints = params.get('waypoints', [])
            self.controller.queue_command(
                self.controller.load_mission_from_points,
                waypoints,
                True
            )

        else:
            log_message("RPi", f"Unknown Command: {cmd}\n")
    
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