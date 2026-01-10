import time
import threading
from typing import List, Dict, Any

from core.drone_state import DroneState, DroneName
from core.radio_comm import RadioComm
from core.telemetry_parser import parse_telemetry
from utils.logger import log_message


class GCSController:

    def __init__(self):
        self.drone_states: List[DroneState] = [None] * len(DroneName)

        self.sprayer_state = DroneState(
            name="Sprayer",
            password="vihang@2025",
            radio=RadioComm(port="/dev/ttyUSB0")
        )

        self.scanner_state = DroneState(
            name="Scanner",
            password="vihang@2025",
            radio=RadioComm(port="/dev/ttyUSB0")
        )

        self.lock = threading.Lock()
        self._running = True

        self._init_drone_states()

        self.listener_thread = threading.Thread(
            target=self._update_loop,
            daemon=True
        )
        self.listener_thread.start()

    # ---------------------------------------------------------
    # INIT
    # ---------------------------------------------------------
    def _init_drone_states(self) -> None:
        self.drone_states[DroneName.Scanner.value] = self.scanner_state
        self.drone_states[DroneName.Sprayer.value] = self.sprayer_state

        self.scanner_state.radio.start()
        self.sprayer_state.radio.start()

    # ---------------------------------------------------------
    # LISTENING AND UPDATING
    # ---------------------------------------------------------
# ---------------------------------------------------------
# LISTENING AND UPDATING
    # ---------------------------------------------------------
    def _update_loop(self) -> None:
        while self._running:
            # --- Sprayer ---
            pkt = self.sprayer_state.radio.get_latest_packet()
            if pkt:
                print("GCS GOT SPRAYER PACKET", pkt)
                try:
                    self._process_packet(DroneName.Sprayer, pkt)
                except Exception as e:
                    log_message("GCSController", f"Error processing sprayer packet: {e}")

            # --- Scanner ---
            pkt = self.scanner_state.radio.get_latest_packet()
            if pkt:
                print("GCS GOT SCANNER PACKET", pkt)
                try:
                    self._process_packet(DroneName.Scanner, pkt)
                except Exception as e:
                    log_message("GCSController", f"Error processing scanner packet: {e}")

            time.sleep(0.01)


    # ---------------------------------------------------------
    # PACKET PROCESSING
    # ---------------------------------------------------------
    def _process_packet(self, drone: DroneName, packet: Dict[str, Any]) -> None:
        with self.lock:
            state = self.drone_states[drone.value]

            # --- AUTH ---
            # Accept packet if name matches and either no password is provided or the password matches
            if packet.get("name") != state.name:
                return
            pw = packet.get("password")
            if pw is not None and pw != state.password:
                log_message("GCSController", f"Rejected packet for {packet.get('name')} - bad password\n")
                return

            # --- BASIC STATE ---
            # Accept either 'status' or 'state' from incoming packets
            state.status = packet.get("status", packet.get("state", state.status))

            if "battery" in packet:
                state.battery = packet["battery"]

            if packet.get("log") is not None:
                state.log = packet["log"]

            # --- TELEMETRY ---
            incoming_telemetry = packet.get("telemetry", packet)

            if isinstance(incoming_telemetry, dict):
                for key, value in incoming_telemetry.items():
                    # Only update keys that exist in telemetry dict
                    if key in state.telemetry:
                        try:
                            state.telemetry[key] = float(value)
                        except (ValueError, TypeError):
                            state.telemetry[key] = value
            else:
                log_message("GCSController", f"Unexpected telemetry format: {incoming_telemetry}")

    # PUBLIC STATE ACCESS
    # ---------------------------------------------------------
    def get_drone_state(self, drone: DroneName) -> Dict[str, Any]:
        with self.lock:
            telemetry = self.drone_states[drone.value].telemetry.copy()
            status = self.drone_states[drone.value].status or "Idle"
            battery = self.drone_states[drone.value].battery if self.drone_states[drone.value].battery is not None else -1
            log = self.drone_states[drone.value].log

        ui_telemetry = parse_telemetry(telemetry)

        return {
            "status": status,
            "battery": battery,
            "log": log,
            "telemetry": telemetry,
            "ui_telemetry": ui_telemetry,
        }

    # ---------------------------------------------------------
    # COMMANDS (UI → DRONE)
    # ---------------------------------------------------------
    def land(self, drone: DroneName) -> None:
        self.drone_states[drone.value].radio.send_command("LAND")

    def start(self, drone: DroneName) -> None:
        self.drone_states[drone.value].radio.send_command("START")
        with self.lock:
            state = self.drone_states[drone.value]
            state.lat0 = state.telemetry.get("lat")
            state.lon0 = state.telemetry.get("lon")
            state.started = True

    def rtl(self, drone: DroneName) -> None:
        self.drone_states[drone.value].radio.send_command("RTL")

    def set_mode(self, drone: DroneName, mode_name: str) -> None:
        self.drone_states[drone.value].radio.send_command(
            "SET_MODE",
            {"mode": mode_name}
        )

    def send_coords(self, drone: DroneName, x: float, y: float, z: float) -> None:
        self.drone_states[drone.value].radio.send_command(
            "MOVE",
            {"x": x, "y": y, "z": z}
        )

    # ---------------------------------------------------------
    # STOP
    # ---------------------------------------------------------
    def stop(self) -> None:
        self._running = False
        self.listener_thread.join()
        print("[GCSController] Stopped")
