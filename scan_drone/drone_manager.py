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
        self.logger = setup_logger("SpotSocket")

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
                data = sock.recv(1024).decode('utf-8')
                if not data:
                    self.logger.info("SpotTracker disconnected")
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
                            if msg.get("type") == "yellow_spots":
                                with self._lock:
                                    self.latest_spots = msg.get("spots", [])
                        except json.JSONDecodeError:
                            pass

            except socket.timeout:
                continue
            except Exception as e:
                self.logger.error(f"Receive error: {e}")
                break

    def send_telemetry(self, telemetry: dict):
        """Send telemetry data to SpotTracker."""
        with self._lock:
            sock = self.client_socket

        if not sock:
            return

        try:
            msg = {
                "type": "telemetry",
                "telemetry": telemetry
            }
            data = json.dumps(msg).encode('utf-8') + b'\n'
            sock.sendall(data)
        except Exception as e:
            self.logger.error(f"Send telemetry failed: {e}")
            with self._lock:
                self.client_socket = None

    def get_latest_spots(self) -> list:
        """Get the latest detected spots."""
        with self._lock:
            return self.latest_spots.copy()

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


class DroneManager:
    def __init__(self):
        self.radio = RadioComm(port="/dev/ttyAMA0")
        self.radio.start()

        BASE_DIR = os.path.dirname(os.path.abspath(__file__))
        KML_PATH = os.path.join(BASE_DIR, "data", "JUs.kml")
        
        config = DroneConfig(
            connection_string='/dev/ttyACM0',
            geofence_mode="polygon",
            kml_file=KML_PATH,
            polygon_name="Field",
            max_altitude=30.0,
            geofence_radius=500.0,
            waypoint_radius=3.0,
            optimize_waypoint_order=True,
            default_altitude=3.0
        )
        
        self.controller = DroneController(config)
        
        if not self.controller.connect():
            print("ERROR: Failed to connect to flight controller")
            return
        
        self.controller.start()
        
        # Start socket server for SpotTracker communication
        self.spot_socket = SpotTrackerSocketServer(host="127.0.0.1", port=5005)
        self.spot_socket.start()
        
        # Start SpotTracker subprocess
        self.spot_tracker_process = None
        self._start_spot_tracker()
        
        self.detected_spots = []
        self._last_tx = time.time()

    def _start_spot_tracker(self):
        """Start the SpotTracker as a separate process."""
        try:
            spot_tracker_path = os.path.join(
                os.path.dirname(__file__), 
                "image_processing", 
                "spot_tracker.py"
            )
            
            self.spot_tracker_process = subprocess.Popen(
                ["python3", spot_tracker_path],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid
            )
            
            print(f"[DroneManager] SpotTracker started (PID: {self.spot_tracker_process.pid})")
            
        except Exception as e:
            print(f"[DroneManager] Failed to start SpotTracker: {e}")
            self.spot_tracker_process = None

    def _stop_spot_tracker(self):
        """Stop the SpotTracker process gracefully."""
        if self.spot_tracker_process:
            try:
                os.killpg(os.getpgid(self.spot_tracker_process.pid), signal.SIGTERM)
                try:
                    self.spot_tracker_process.wait(timeout=5)
                    print("[DroneManager] SpotTracker stopped gracefully")
                except subprocess.TimeoutExpired:
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
            # log_message("RPi", f"Mode change requests handled by state machine\n")

        elif cmd == "LOAD_WAYPOINTS":
            waypoints = params.get('waypoints', [])
            self.controller.queue_command(
                self.controller.load_mission_from_points,
                waypoints,
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
        
        # Send telemetry to SpotTracker via socket
        self.spot_socket.send_telemetry(status['telemetry'])
        
        # Get detected spots from SpotTracker
        self.detected_spots = self.spot_socket.get_latest_spots()
        
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
            "log": "\n".join(status['logs'][-10:])
        }
        self.radio.send_packet(packet)

    # -------------------------------------------------
    # MAIN LOOP
    # -------------------------------------------------
    def run(self):
        try:
            while self.controller.running.is_set():
                cmd = self.radio.get_latest_command()
                if cmd:
                    self.handle_command(cmd)

                if time.time() - self._last_tx > 0.5:
                    self.send_telemetry()
                    self._last_tx = time.time()
                
                time.sleep(0.01)
        except KeyboardInterrupt:
            print("\n[DroneManager] Shutdown requested")
        finally:
            self.stop()
    
    def stop(self):
        """Clean shutdown of all components."""
        print("[DroneManager] Stopping components...")
        self._stop_spot_tracker()
        self.spot_socket.stop()
        self.controller.stop()
        self.radio.stop()
        print("[DroneManager] Shutdown complete")

if __name__ == "__main__":
    drone = DroneManager()
    drone.run()