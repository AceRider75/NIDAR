import time
from radio_comm import RadioComm
from controller import Controller
from utils import log_message

DRONE_NAME = "Sprayer"
PASSWORD = "vihang@2025"

class DroneManager:
    def __init__(self):
        self.radio = RadioComm(port="/dev/ttyAMA0")
        self.radio.start()

        self.Controller = Controller()

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

        self._last_tx = time.time()

    # -------------------------------------------------
    # COMMAND HANDLING
    # -------------------------------------------------
    def handle_command(self, packet):
        cmd = packet.get("command")
        params = packet.get("params", {})

        if cmd == "START":
            self.status = "Flying"
            self.log = log_message("RPi","Starting Drone\n")
            self.Controller.start_drone()

        elif cmd == "LAND":
            self.status = "Landing"
            self.log = log_message("RPi","Landing Drone\n")
            self.Controller.land_drone()

        elif cmd == "RTL":
            self.status = "Returning"
            self.log = log_message("RPi","Activating RTL\n")
            self.Controller.return_to_launch()

        elif cmd == "SET_MODE":
            self.log = log_message("RPi",f"Setting Mode to {params.get('mode')}\n")
            self.Controller.set_mode(params.get('mode'))

        elif cmd == "MOVE":
            self.log = log_message("RPi",f"Setting Waypoint to {params}\n")
            self.Controller.send_coords(params)

        else:
            self.log = log_message("RPi",f"Unknown Command: {cmd}\n")
    # -------------------------------------------------
    # TELEMETRY TX
    # -------------------------------------------------
    def send_telemetry(self):
        state = self.Controller.get_drone_state()
        self.log = self.log + state.get("log", "")
        self.status = state.get("status", self.status)
        self.battery = state.get("battery", self.battery)
        self.telemetry.update(state.get("telemetry", {}))
        packet = {
            "name": DRONE_NAME,
            "password": PASSWORD,
            "status": self.status,
            "battery": self.battery,
            "telemetry": self.telemetry,
            "log": self.log
        }
        self.radio.send_packet(packet)

    # -------------------------------------------------
    # MAIN LOOP
    # -------------------------------------------------
    def run(self):
        while True:
            cmd = self.radio.get_latest_command()
            if cmd:
                self.handle_command(cmd)

            if time.time() - self._last_tx > 0.5:  # 2 Hz
                self.send_telemetry()
                self._last_tx = time.time()
            time.sleep(0.01)

if __name__ == "__main__":
    drone = DroneManager()
    drone.run()
