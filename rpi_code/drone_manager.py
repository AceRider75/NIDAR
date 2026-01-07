import time
import socket
import threading
import json
from radio_comm import RadioComm
from controller import Controller
from utils import log_message

DRONE_NAME = "Sprayer"
PASSWORD = "vihang@2025"


class SpotTrackerSocketServer:
    """
    Socket server for communicating with the SpotTracker.
    Handles:
    - Sending telemetry data to SpotTracker
    - Receiving detected spot data from SpotTracker
    """

    def __init__(self, host: str = "127.0.0.1", port: int = 5005):
        self.host = host
        self.port = port
        self.server_socket = None
        self.client_socket = None
        self.client_address = None
        self.running = False
        self.buffer = ""
        self.latest_spots = []  # Store received spot data
        self._lock = threading.Lock()

    def start(self):
        """Start the socket server in a background thread."""
        self.running = True
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(
            socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind((self.host, self.port))
        self.server_socket.listen(1)
        # Allow periodic checks for shutdown
        self.server_socket.settimeout(1.0)

        # Start accept thread
        self.accept_thread = threading.Thread(
            target=self._accept_connections, daemon=True)
        self.accept_thread.start()

        print(
            f"[SpotTrackerSocket] Server listening on {self.host}:{self.port}")

    def _accept_connections(self):
        """Accept incoming connections from SpotTracker."""
        while self.running:
            try:
                client_sock, addr = self.server_socket.accept()
                client_sock.settimeout(0.05)  # Non-blocking for receive

                with self._lock:
                    # Close existing connection if any
                    if self.client_socket:
                        try:
                            self.client_socket.close()
                        except:
                            pass
                    self.client_socket = client_sock
                    self.client_address = addr
                    self.buffer = ""

                print(f"[SpotTrackerSocket] SpotTracker connected from {addr}")

                # Start receive thread for this client
                recv_thread = threading.Thread(
                    target=self._receive_loop, daemon=True)
                recv_thread.start()

            except socket.timeout:
                continue
            except Exception as e:
                if self.running:
                    print(f"[SpotTrackerSocket] Accept error: {e}")

    def _receive_loop(self):
        """Receive data from connected SpotTracker client."""
        while self.running:
            with self._lock:
                sock = self.client_socket

            if not sock:
                time.sleep(0.1)
                continue

            try:
                data = sock.recv(1024).decode('utf-8')
                if not data:
                    # Client disconnected
                    print("[SpotTrackerSocket] SpotTracker disconnected")
                    with self._lock:
                        self.client_socket = None
                    break

                self.buffer += data

                # Process complete messages (newline delimited JSON)
                while '\n' in self.buffer:
                    line, self.buffer = self.buffer.split('\n', 1)
                    line = line.strip()
                    if line:
                        try:
                            msg = json.loads(line)
                            if msg.get("type") == "yellow_spots":
                                with self._lock:
                                    self.latest_spots = msg.get("spots", [])
                                # print(f"[SpotTrackerSocket] Received {len(self.latest_spots)} spots")
                        except json.JSONDecodeError:
                            pass

            except socket.timeout:
                continue
            except ConnectionResetError:
                print("[SpotTrackerSocket] Connection reset by SpotTracker")
                with self._lock:
                    self.client_socket = None
                break
            except Exception as e:
                print(f"[SpotTrackerSocket] Receive error: {e}")
                break

    def send_telemetry(self, telemetry: dict):
        """
        Send telemetry data to connected SpotTracker.

        Args:
            telemetry: Dictionary containing lat, lon, alt, yaw
        """
        with self._lock:
            sock = self.client_socket

        if not sock:
            return

        try:
            msg = {
                "type": "telemetry",
                "telemetry": {
                    "lat": telemetry.get("lat", 0.0),
                    "lon": telemetry.get("lon", 0.0),
                    "alt": telemetry.get("alt", 0.0),
                    "yaw": telemetry.get("yaw", 0.0)
                }
            }
            data = json.dumps(msg).encode('utf-8') + b'\n'
            sock.sendall(data)
        except Exception as e:
            print(f"[SpotTrackerSocket] Send telemetry failed: {e}")
            with self._lock:
                self.client_socket = None

    def get_latest_spots(self) -> list:
        """
        Get the latest detected spots from SpotTracker.

        Returns:
            List of spot dictionaries with id, lat, lon, cx, cy, area, rank
        """
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
                self.client_socket = None

        if self.server_socket:
            try:
                self.server_socket.close()
            except:
                pass

        print("[SpotTrackerSocket] Server stopped")


class DroneManager:
    def __init__(self):
        self.radio = RadioComm(port="/dev/ttyAMA0")
        self.radio.start()

        self.Controller = Controller()

        # Socket server for SpotTracker communication
        self.spot_socket = SpotTrackerSocketServer(host="127.0.0.1", port=5005)
        self.spot_socket.start()

        self.status = "Idle"
        self.battery = -1
        self.log = ""

        # Example telemetry state
        self.telemetry = {
            "lat": 0.0,
            "lon": 0.0,
            "alt": 0.0,
            "vx": 0.0,
            "vy": 0.0,
            "vz": 0.0,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
            "xacc": 0.0,
            "yacc": 0.0,
        }

        # Store detected spots from SpotTracker
        self.detected_spots = []

        self._last_tx = time.time()

    # -------------------------------------------------
    # COMMAND HANDLING
    # -------------------------------------------------
    def handle_command(self, packet):
        cmd = packet.get("command")
        params = packet.get("params", {})

        log_message("RPi", f"Received command: {cmd}\n")

        if cmd == "START":
            self.status = "Flying"
            self.log = log_message("RPi", "Starting Drone\n")
            self.Controller.start_drone()

        elif cmd == "LAND":
            self.status = "Landing"
            self.log = log_message("RPi", "Landing Drone\n")
            self.Controller.land_drone()

        elif cmd == "RTL":
            self.status = "Returning"
            self.log = log_message("RPi", "Activating RTL\n")
            self.Controller.return_to_launch()

        elif cmd == "SET_MODE":
            self.log = log_message(
                "RPi", f"Setting Mode to {params.get('mode')}\n")
            self.Controller.set_mode(params.get('mode'))

        elif cmd == "MOVE":
            self.log = log_message("RPi", f"Setting Waypoint to {params}\n")
            self.Controller.send_coords(params)

        else:
            self.log = log_message("RPi", f"Unknown Command: {cmd}\n")
    # -------------------------------------------------
    # TELEMETRY TX
    # -------------------------------------------------

    def send_telemetry(self):
        state = self.Controller.get_drone_state()
        # self.log = self.log + state.get("log", "")
        self.status = state.get("status", self.status)
        self.battery = state.get("battery", self.battery)
        self.telemetry.update(state.get("telemetry", {}))

        # Send telemetry to GCS via radio
        packet = {
            "name": DRONE_NAME,
            # "password": PASSWORD,
            "status": self.status,
            # "battery": self.battery,
            "telemetry": self.telemetry,
            # "log": self.log
        }
        self.radio.send_packet(packet)

        # Send telemetry to SpotTracker via socket
        self.spot_socket.send_telemetry(self.telemetry)

    # -------------------------------------------------
    # SPOT TRACKING
    # -------------------------------------------------
    def update_spots(self):
        """Update detected spots from SpotTracker."""
        self.detected_spots = self.spot_socket.get_latest_spots()
        return self.detected_spots

    def get_detected_spots(self):
        """Get the latest detected spots."""
        return self.detected_spots

    # -------------------------------------------------
    # MAIN LOOP
    # -------------------------------------------------
    def run(self):
        try:
            while True:
                cmd = self.radio.get_latest_command()
                if cmd:
                    self.handle_command(cmd)

                if time.time() - self._last_tx > 0.5:  # 2 Hz
                    self.send_telemetry()
                    self.update_spots()  # Update spots from SpotTracker
                    self._last_tx = time.time()
                time.sleep(0.01)
        except KeyboardInterrupt:
            print("\nDroneManager stopped by user")
        finally:
            self.stop()

    def stop(self):
        """Clean shutdown of all components."""
        self.spot_socket.stop()
        print("DroneManager shutdown complete")


if __name__ == "__main__":
    drone = DroneManager()
    drone.run()
