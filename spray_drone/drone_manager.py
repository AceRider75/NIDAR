from controller import DroneController
from radio_comm import RadioComm
import time
import os
from config import DroneConfig
# from utils import log_message
from utils import setup_logger

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
            connection_string='/dev/ttyACM1',  # Change to '/dev/ttyACM0' for real hardware
            geofence_mode="polygon",  # or "radius"
            kml_file=KML_PATH,
            polygon_name="Field",
            max_altitude=30.0,
            geofence_radius=500.0,
            waypoint_radius=3.0,
            optimize_waypoint_order=True,
            default_altitude=4.0
        )
        
        self.controller = DroneController(config)
        
        # Connect and start
        if not self.controller.connect():
            print("ERROR: Failed to connect to flight controller")
            return
        
        self.controller.start()
        
        self._last_tx = time.time()

        # Buffer for incremental waypoints
        self._pending_waypoints = []
        self._expected_waypoints = 0
        self._pending_altitude = 4.0

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
            altitude = params.get("altitude", 4.0)
            
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
            # log_message("RPi", f"Mode change requests handled by state machine\n")

        elif cmd == "LOAD_WAYPOINTS":
            waypoints = params.get('waypoints', [])
            self.controller.queue_command(
                self.controller.load_mission_from_points,
                waypoints,
                True
            )

        # Incremental waypoint protocol
        elif cmd == "START_WAYPOINTS":
            self._expected_waypoints = int(params.get("expected_count", 0))
            self._pending_altitude = float(params.get("altitude", 4.0))
            self._pending_waypoints = []
            print(f"[WP] START_WAYPOINTS received: expecting {self._expected_waypoints} waypoints")

        elif cmd == "ADD_WAYPOINT":
            lat = params.get("lat")
            lon = params.get("lon")
            alt = params.get("alt", getattr(self, '_pending_altitude', 4.0))
            idx = params.get("index", len(getattr(self, '_pending_waypoints', [])) + 1)
            
            # Initialize buffer if START_WAYPOINTS was missed
            if not hasattr(self, '_pending_waypoints'):
                self._pending_waypoints = []
            if not hasattr(self, '_expected_waypoints'):
                self._expected_waypoints = 0
                
            if lat is not None and lon is not None:
                try:
                    self._pending_waypoints.append([float(lat), float(lon), float(alt)])
                    print(f"[WP] ADD_WAYPOINT {idx}: ({lat}, {lon}, {alt}) - total buffered: {len(self._pending_waypoints)}")
                except (ValueError, TypeError):
                    print(f"[WP] Ignoring malformed waypoint: {params}")

        elif cmd == "END_WAYPOINTS":
            expected = int(params.get("expected_count", self._expected_waypoints))
            received = len(self._pending_waypoints) if hasattr(self, '_pending_waypoints') else 0
            
            print(f"[WP] END_WAYPOINTS: expected={expected}, received={received}")
            
            # If START was missed but we have waypoints, use received count
            if self._expected_waypoints == 0 and received > 0:
                print(f"[WP] START_WAYPOINTS was missed, using received count: {received}")
                self._expected_waypoints = received
            
            # Check if we have enough waypoints (allow some tolerance for corruption)
            min_required = max(1, expected - 2)  # Allow up to 2 missing waypoints
            
            if received < min_required:
                print(f"[WP] Received {received} of {expected}, need at least {min_required}, aborting mission load")
                # Reset buffer
                self._pending_waypoints = []
                self._expected_waypoints = 0
                return
            
            if received != expected:
                print(f"[WP] Warning: received {received} waypoints, expected {expected}, loading anyway")

            # Load mission from buffered waypoints and add return-to-home
            print(f"[WP] Loading mission with {received} waypoints")
            self.controller.queue_command(
                self.controller.load_mission_from_points,
                self._pending_waypoints,
                True
            )
            self.controller.queue_command(
                self.controller.add_return_home_waypoint
            )

            # Reset buffer
            self._pending_waypoints = []
            self._expected_waypoints = 0

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
        print("\n[DroneManager] Entering main loop...")
        print("[DroneManager] Waiting for commands from GCS...")
        
        # SIMULATION: Auto-start if in SITL mode
        if "127.0.0.1" in self.controller.config.connection_string:
            print("[SIMULATOR] SITL connection detected.")
            print("[SIMULATOR] Simulating START command in 5 seconds...")
            
            def simulate_start():
                time.sleep(5)
                print("\n" + "="*40)
                print("[SIMULATOR] INJECTING START COMMAND")
                print("="*40 + "\n")
                
                # Mock a packet from GCS
                start_packet = {
                    "command": "START",
                    "params": {
                        "altitude": 4.0
                    }
                }
                self.handle_command(start_packet)
            
            threading.Thread(target=simulate_start, daemon=True).start()
        
        try:
            while self.controller.running.is_set():
                # Handle incoming commands from GCS
                cmd = self.radio.get_latest_command()
                if cmd:
                    self.handle_command(cmd)

                # Send telemetry at fixed rate (2 Hz)
                if time.time() - self._last_tx > 0.5:
                    self.send_telemetry()
                    self._last_tx = time.time()
                
                time.sleep(0.01)
                
        except KeyboardInterrupt:
            print("\n[DroneManager] Keyboard Interrupt detected (Ctrl+C)")
            print("[DroneManager] !!! TRIGGERING EMERGENCY LANDING !!!")
            
            try:
                # Construct a LAND command packet to ensure safe termination
                land_packet = {
                    "command": "LAND",
                    "params": {}
                }
                self.handle_command(land_packet)
                
                # Give the system a moment to process/send the command
                time.sleep(1.0)
            except Exception as e:
                print(f"[DroneManager] Failed to trigger emergency landing: {e}")

        except Exception as e:
            print(f"\n[DroneManager] ERROR in main loop: {e}")
            import traceback
            traceback.print_exc()
        finally:
            self.stop()


if __name__ == "__main__":
    drone = DroneManager()
    drone.run()