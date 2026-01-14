from controller import DroneController
from radio_comm import RadioComm
import time
import os
import subprocess
import signal
import socket
import threading
import json
from config import DroneConfig
from utils import setup_logger

DRONE_NAME = "Scanner"
PASSWORD = "vihang@2025"

class SpotTrackerSocketServer:
    """Socket server for bidirectional communication with SpotTracker."""
    
    def __init__(self, host: str = "127.0.0.1", port: int = 5005):
        self.host = host
        self.port = port
        self.server_socket = None
        self.client_socket = None
        self.running = False
        self.buffer = ""
        self.latest_spots = []
        self._lock = threading.Lock()
        self.logger = setup_logger("SpotSocket", "spot_socket.log")

    def start(self):
        """Start the socket server."""
        self.running = True
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind((self.host, self.port))
        self.server_socket.listen(1)
        self.server_socket.settimeout(1.0)

        self.accept_thread = threading.Thread(target=self._accept_connections, daemon=True)
        self.accept_thread.start()
        
        self.logger.info(f"Server listening on {self.host}:{self.port}")
        print(f"[DroneManager] Socket server started on {self.host}:{self.port}")

    def _accept_connections(self):
        """Accept incoming connections from SpotTracker."""
        while self.running:
            try:
                client_sock, addr = self.server_socket.accept()
                client_sock.settimeout(0.05)

                with self._lock:
                    if self.client_socket:
                        try:
                            self.client_socket.close()
                        except:
                            pass
                    self.client_socket = client_sock
                    self.buffer = ""

                self.logger.info(f"SpotTracker connected from {addr}")
                print(f"[DroneManager] SpotTracker connected from {addr}")

                recv_thread = threading.Thread(target=self._receive_loop, daemon=True)
                recv_thread.start()

            except socket.timeout:
                continue
            except Exception as e:
                if self.running:
                    self.logger.error(f"Accept error: {e}")

    def _receive_loop(self):
        """Receive spot data from SpotTracker."""
        while self.running:
            with self._lock:
                sock = self.client_socket

            if not sock:
                time.sleep(0.1)
                continue

            try:
                data = sock.recv(4096).decode('utf-8')
                if not data:
                    self.logger.info("SpotTracker disconnected")
                    print("[DroneManager] SpotTracker disconnected")
                    with self._lock:
                        self.client_socket = None
                    break

                self.buffer += data

                while '\n' in self.buffer:
                    line, self.buffer = self.buffer.split('\n', 1)
                    line = line.strip()
                    if line:
                        try:
                            msg = json.loads(line)
                            msg_type = msg.get("type")
                            
                            if msg_type == "register":
                                # Handle registration from spot_tracker
                                client_name = msg.get("name")
                                self.logger.info(f"Client registered: {client_name}")
                                print(f"[DroneManager] Client registered: {client_name}")
                                
                            elif msg_type == "yellow_spots":
                                # Receive spot data from SpotTracker
                                with self._lock:
                                    self.latest_spots = msg.get("spots", [])
                                
                                num_spots = len(self.latest_spots)
                                if num_spots > 0:
                                    self.logger.debug(f"Received {num_spots} yellow spots")
                                    # Only print occasionally to avoid spam
                                    if num_spots > 0 and int(time.time()) % 5 == 0:
                                        print(f"[DroneManager] Tracking {num_spots} yellow spots")
                        
                        except json.JSONDecodeError as e:
                            self.logger.error(f"JSON decode error: {e}")

            except socket.timeout:
                continue
            except Exception as e:
                self.logger.error(f"Receive error: {e}")
                with self._lock:
                    self.client_socket = None
                break

    def send_telemetry(self, telemetry: dict):
        """Send telemetry data to SpotTracker."""
        with self._lock:
            sock = self.client_socket

        if not sock:
            return False

        try:
            msg = {
                "type": "telemetry",
                "timestamp": time.time(),
                "telemetry": {
                    "lat": telemetry.get("lat", 0.0),
                    "lon": telemetry.get("lon", 0.0),
                    "alt": telemetry.get("alt", 0.0),
                    "yaw": telemetry.get("yaw", 0.0),
                    "roll": telemetry.get("roll", 0.0),
                    "pitch": telemetry.get("pitch", 0.0)
                }
            }
            data = json.dumps(msg).encode('utf-8') + b'\n'
            sock.sendall(data)
            return True
        except Exception as e:
            self.logger.error(f"Send telemetry failed: {e}")
            with self._lock:
                self.client_socket = None
            return False

    def get_latest_spots(self) -> list:
        """Get the latest detected spots."""
        with self._lock:
            return self.latest_spots.copy()

    def is_connected(self) -> bool:
        """Check if SpotTracker is connected."""
        with self._lock:
            return self.client_socket is not None

    def stop(self):
        """Stop the socket server."""
        self.running = False
        with self._lock:
            if self.client_socket:
                try:
                    self.client_socket.close()
                except:
                    pass
        if self.server_socket:
            try:
                self.server_socket.close()
            except:
                pass
        self.logger.info("Server stopped")
        print("[DroneManager] Socket server stopped")


class DroneManager:
    def __init__(self):
        print("=" * 60)
        print("DRONE MANAGER - Initializing")
        print("=" * 60)
        
        # Initialize radio communication
        print("[DroneManager] Starting radio communication...")
        self.radio = RadioComm(port="/dev/ttyAMA0")
        self.radio.start()

        # Setup controller configuration
        BASE_DIR = os.path.dirname(os.path.abspath(__file__))
        KML_PATH = os.path.join(BASE_DIR, "data", "GBUsoccer.kml")
        
        print(f"[DroneManager] Loading KML from: {KML_PATH}")
        
        config = DroneConfig(
            connection_string='127.0.0.1:14551',
            geofence_mode="polygon",
            kml_file=KML_PATH,
            polygon_name="MultiGeometry",
            max_altitude=30.0,
            geofence_radius=500.0,
            waypoint_radius=3.0,
            optimize_waypoint_order=True,
            default_altitude=3.0
        )
        
        # Initialize controller
        print("[DroneManager] Initializing drone controller...")
        self.controller = DroneController(config)
        
        print("[DroneManager] Connecting to flight controller...")
        if not self.controller.connect():
            print("ERROR: Failed to connect to flight controller")
            raise RuntimeError("Flight controller connection failed")
        
        print("[DroneManager] Starting controller threads...")
        self.controller.start()
        
        # Start socket server for SpotTracker communication
        print("[DroneManager] Starting socket server for SpotTracker...")
        self.spot_socket = SpotTrackerSocketServer(host="127.0.0.1", port=5005)
        self.spot_socket.start()
        
        # Start SpotTracker subprocess (uncomment to auto-start)
        self.spot_tracker_process = None
        self._start_spot_tracker()
        
        self.detected_spots = []
        self._last_tx = time.time()
        self._last_spot_check = time.time()
        
        print("=" * 60)
        print("DRONE MANAGER - Ready")
        print("=" * 60)
        print(f"  Flight Controller: Connected")
        print(f"  Socket Server: Port 5005")
        print(f"  SpotTracker: {'Starting...' if self.spot_tracker_process else 'Manual start required'}")
        print(f"  Radio: Active")
        print("=" * 60)

    def _start_spot_tracker(self):
        """Start the SpotTracker as a separate process."""
        try:
            spot_tracker_path = os.path.join(
                os.path.dirname(__file__), 
                "image_processing",
                "spot_tracker.py"
            )
            
            # Check if file exists
            if not os.path.exists(spot_tracker_path):
                print(f"[DroneManager] WARNING: spot_tracker.py not found at {spot_tracker_path}")
                print(f"[DroneManager] Please start spot_tracker.py manually:")
                print(f"    python3 {spot_tracker_path}")
                return
            
            print(f"[DroneManager] Starting SpotTracker from: {spot_tracker_path}")
            
            # Use system Python (not venv) for spot_tracker
            python_exe = "/usr/bin/python3"
            if not os.path.exists(python_exe):
                python_exe = "python3"  # fallback to PATH
            
            self.spot_tracker_process = subprocess.Popen(
                [python_exe, spot_tracker_path],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                universal_newlines=True,
                preexec_fn=os.setsid
            )
            
            print(f"[DroneManager] SpotTracker started (PID: {self.spot_tracker_process.pid})")
            
            # Start thread to monitor spot_tracker output
            monitor_thread = threading.Thread(
                target=self._monitor_spot_tracker_output,
                daemon=True
            )
            monitor_thread.start()
            
        except Exception as e:
            print(f"[DroneManager] Failed to start SpotTracker: {e}")
            print(f"[DroneManager] You can start it manually with:")
            print(f"    python3 spot_tracker.py")
            self.spot_tracker_process = None

    def _monitor_spot_tracker_output(self):
        """Monitor SpotTracker subprocess output."""
        if not self.spot_tracker_process:
            return
        
        try:
            for line in self.spot_tracker_process.stdout:
                print(f"[SPOT_TRACKER] {line.rstrip()}")
        except Exception as e:
            print(f"[DroneManager] SpotTracker monitor error: {e}")
        
        # Process ended
        print("[DroneManager] SpotTracker process ended")

    def _stop_spot_tracker(self):
        """Stop the SpotTracker process gracefully."""
        if self.spot_tracker_process:
            try:
                print("[DroneManager] Stopping SpotTracker...")
                os.killpg(os.getpgid(self.spot_tracker_process.pid), signal.SIGTERM)
                try:
                    self.spot_tracker_process.wait(timeout=5)
                    print("[DroneManager] SpotTracker stopped gracefully")
                except subprocess.TimeoutExpired:
                    print("[DroneManager] SpotTracker not responding, force killing...")
                    os.killpg(os.getpgid(self.spot_tracker_process.pid), signal.SIGKILL)
                    print("[DroneManager] SpotTracker force killed")
            except Exception as e:
                print(f"[DroneManager] Error stopping SpotTracker: {e}")

    # -------------------------------------------------
    # COMMAND HANDLING - UPDATED
    # -------------------------------------------------
    def handle_command(self, packet):
        cmd = packet.get("command")
        params = packet.get("params", {})
        
        print(f"[DroneManager] Received command: {cmd}")

        if cmd == "START":
            # Get mission parameters from command (or use defaults)
            waypoints = params.get("waypoints", [])
            altitude = params.get("altitude", 3.0)
            
            # If no waypoints provided, generate spiral pattern from KML
            if not waypoints and self.controller.mission_planner:
                print("[DroneManager] No waypoints provided. Generating spiral mission from KML...")
                # Use current position as start for optimization
                curr_lat = self.controller.telemetry.lat
                curr_lon = self.controller.telemetry.lon
                
                points_2d = self.controller.mission_planner.generate_mission_from_points(
                    start_lat=curr_lat if curr_lat != 0 else None,
                    start_lon=curr_lon if curr_lon != 0 else None
                )
                
                # Convert to waypoints with altitude
                waypoints = [[lat, lon, altitude] for lat, lon in points_2d]
                print(f"[DroneManager] Generated {len(waypoints)} waypoints from KML")
                print("Generated Mission Plan:")
                for i, wp in enumerate(waypoints):
                    print(f"  WP {i+1}: Lat={wp[0]:.6f}, Lon={wp[1]:.6f}, Alt={wp[2]:.1f}m")

            print(f"[DroneManager] START command received")
            print(f"[DroneManager] Waypoints: {len(waypoints)}, Altitude: {altitude}m")
             
            # Load mission waypoints if provided (validates against KML polygon)
            if waypoints:
                print("[DroneManager] Loading mission waypoints...")
                self.controller.queue_command(
                    self.controller.load_mission_from_points,
                    waypoints,
                    True  # validate_geofence
                )
            
            # Arm and takeoff
            print(f"[DroneManager] Queuing arm and takeoff to {altitude}m...")
            self.controller.queue_command(
                self.controller.arm_and_takeoff,
                altitude
            )
            
            # Start mission if waypoints were provided
            if waypoints:
                print("[DroneManager] Queuing mission start...")
                self.controller.queue_command(
                    self.controller.start_mission
                )
            
            # Note: Mission monitoring happens in send_telemetry() 
            # and mission completion triggers return_to_home_and_land via state machine

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
            print(f"[DroneManager] Mode change requests handled by state machine")

        elif cmd == "LOAD_WAYPOINTS":
            waypoints = params.get('waypoints', [])
            self.controller.queue_command(
                self.controller.load_mission_from_points,
                waypoints,
                True
            )

        else:
            print(f"[DroneManager] Unknown Command: {cmd}")
    
    # -------------------------------------------------
    # TELEMETRY TX - UPDATED
    # -------------------------------------------------
    def send_telemetry(self):
        status = self.controller.get_status()
        
        # Send telemetry to SpotTracker via socket
        telemetry_sent = self.spot_socket.send_telemetry(status['telemetry'])
        
        # Get detected spots from SpotTracker
        self.detected_spots = self.spot_socket.get_latest_spots()
        
        # Check SpotTracker connection status periodically
        now = time.time()
        if now - self._last_spot_check > 10:
            if self.spot_socket.is_connected():
                if telemetry_sent:
                    # Connection OK and data flowing
                    pass
                else:
                    print("[DroneManager] WARNING: SpotTracker connected but telemetry send failed")
            else:
                print("[DroneManager] WARNING: SpotTracker not connected")
            self._last_spot_check = now
        
        # Send to GCS
        packet = {
            "name": DRONE_NAME,
            "password": PASSWORD,
            "state": status['state'],
            "battery": status['telemetry']['battery'],
            "telemetry": status['telemetry'],
            "mission": status['mission'],
            "geofence": status['geofence'],
            "healthy": status['healthy'],
            "yellow_spots": self.detected_spots,  # Include spot data
            "spot_tracker_connected": self.spot_socket.is_connected(),  # Connection status
            "log": "\n".join(status['logs'][-10:])
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
                        "altitude": 3.0
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
            print("\n[DroneManager] Shutdown requested by user")
        except Exception as e:
            print(f"\n[DroneManager] ERROR in main loop: {e}")
            import traceback
            traceback.print_exc()
        finally:
            self.stop()
    
    def stop(self):
        """Clean shutdown of all components."""
        print("\n" + "=" * 60)
        print("DRONE MANAGER - Shutting Down")
        print("=" * 60)
        
        self._stop_spot_tracker()
        
        print("[DroneManager] Stopping socket server...")
        self.spot_socket.stop()
        
        print("[DroneManager] Stopping controller...")
        self.controller.stop()
        
        print("[DroneManager] Stopping radio...")
        self.radio.stop()
        
        print("=" * 60)
        print("DRONE MANAGER - Shutdown Complete")
        print("=" * 60)


def main():
    """Main entry point with signal handling."""
    drone = None
    
    def signal_handler(sig, frame):
        print(f"\n[DroneManager] Received signal {sig}")
        if drone:
            drone.stop()
        import sys
        sys.exit(0)
    
    # Setup signal handlers
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    try:
        drone = DroneManager()
        drone.run()
    except Exception as e:
        print(f"\n[DroneManager] FATAL ERROR: {e}")
        import traceback
        traceback.print_exc()
        if drone:
            drone.stop()
        return 1
    
    return 0


if __name__ == "__main__":
    exit(main())