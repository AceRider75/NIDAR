import time
import threading
from typing import List, Dict, Any
from core.drone_state import DroneState, DroneName
from core.radio_comm import RadioComm
from core.telemetry_parser import parse_telemetry

class GCSController:

    def __init__(self):

        self.drone_states: List[DroneState] = [None] * len(DroneName)   
        self.sprayer_state = DroneState(name = "Sprayer",
                                        password = "vihang@2025",
                                        radio = RadioComm(port="/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0"))
        self.scanner_state = DroneState(name = "Scanner", 
                                        password = "vihang@2025",
                                        radio = RadioComm(port = ""))


        self.lock = threading.Lock()

        self._running = True
        self.listener_thread = threading.Thread(
            target=self._update_loop,
            daemon=True
        )
        self.listener_thread.start()

    def _init_drone_states(self) -> None:
        self.drone_states[DroneName.Scanner.value] = self.scanner_state
        self.drone_states[DroneName.Sprayer.value] = self.sprayer_state


    # ---------------------------------------------------------
    # LISTENING AND UPDATING COMMANDS   
    # ---------------------------------------------------------
    def _update_loop(self) -> None:
        while self._running:

            sprayer_packet = self.sprayer_state.radio.get_latest_packet() 
            if sprayer_packet:
                self._process_packet(DroneName.Sprayer, sprayer_packet)

            scanner_packet = self.scanner_state.radio.get_latest_packet() 
            if scanner_packet:
                self._process_packet(DroneName.Scanner, scanner_packet)

            time.sleep(0.01)

    def _process_packet(self, drone: DroneName, packet: Dict[str, Any]) -> None:

        with self.lock:
            name = packet.get("name")
            password = packet.get("password")
            if(self.drone_states[drone.value].name == name and 
               self.drone_states[drone.value].password == password):
                
                self.drone_states[drone.value].status = packet.get("status", self.drone_states[drone.value].status)
                self.drone_states[drone.value].battery = packet.get("battery", self.drone_states[drone.value].battery)
                log = packet.get("log")
                self.drone_states[drone.value].log = log if log is not None else ""

                incoming_telemetry = packet.get("telemetry", {})

                for key in self.drone_states[drone.value].telemetry:
                    if key in incoming_telemetry:
                        self.drone_states[drone.value].telemetry[key] = incoming_telemetry[key]

                    else:
                        print("[GCSController] Unknown packet:", packet)


    def get_drone_state(self, drone: DroneName) -> Dict[str, Any]:

        with self.lock:
            t = self.drone_states[drone.value].telemetry.copy()
            ui_telemetry = parse_telemetry(t)

            state = {
                "status": self.drone_states[drone.value].status or "Idle",
                "battery": self.drone_states[drone.value].battery if self.drone_states[drone.value].battery is not None else -1,
                "log": self.drone_states[drone.value].log,
                "telemetry": t,
                "ui_telemetry": ui_telemetry,
            }

            return state

    # ---------------------------------------------------------
    # PUBLIC COMMAND FUNCTIONS (UI → Drone)
    # ---------------------------------------------------------
    def land(self, drone: DroneName) -> None:
        self.drone_states[drone.value].radio.send_command("LAND")

    def start(self, drone: DroneName) -> None:
        self.drone_states[drone.value].radio.send_command("START")
        with self.lock:
            self.drone_states[drone.value].lat0 = self.drone_states[drone.value].telemetry["lat"]
            self.drone_states[drone.value].lon0 = self.drone_states[drone.value].telemetry["lon"]
            self.drone_states[drone.value].started = True

    def rtl(self, drone: DroneName) -> None:
        self.drone_states[drone.value].radio.send_command("RTL")

    def set_mode(self, drone: DroneName, mode_name: str) -> None:
        self.drone_states[drone.value].radio.send_command("SET_MODE", {"mode": mode_name})

    def send_coords(self, drone: DroneName, x: float, y: float, z: float) -> None:
        self.drone_states[drone.value].radio.send_command("MOVE", {"x": x, "y": y, "z": z})

    # ---------------------------------------------------------
    # STOP
    # ---------------------------------------------------------
    def stop(self) -> None:
        self._running = False
        self.listener_thread.join()
        print("[GCSController] Stopped")