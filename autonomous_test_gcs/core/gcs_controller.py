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
        self.sprayer_state = DroneState(name = "Sprayer",
                                        password = "vihang@2025",
                                        radio = RadioComm(port=r"/dev/ttyUSB0"))    #Windows style COM port (For Linux/Mac, use "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0")
        self.scanner_state = DroneState(name = "Scanner", 
                                        password = "vihang@2025",
                                        radio = RadioComm(port = ""))    #Windows style COM port (For Linux/Mac, use "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0002-if00-port0")


        self.lock = threading.Lock()

        self._running = True
        self.listener_thread = threading.Thread(
            target=self._update_loop,
            daemon=True
        )
        self._init_drone_states()
        self.listener_thread.start()

    def _init_drone_states(self) -> None:               #Initialize drone states and start their radios
        self.drone_states[DroneName.Scanner.value] = self.scanner_state
        self.drone_states[DroneName.Sprayer.value] = self.sprayer_state
        self.drone_states[DroneName.Scanner.value].radio.start()
        self.drone_states[DroneName.Sprayer.value].radio.start()    


    # ---------------------------------------------------------
    # LISTENING AND UPDATING COMMANDS   
    # ---------------------------------------------------------
    def _update_loop(self) -> None:             #Listen for incoming packets and update drone states
        while self._running:

            sprayer_packet = self.sprayer_state.radio.get_latest_packet() 
            if sprayer_packet:
                self._process_packet(DroneName.Sprayer, sprayer_packet)

            scanner_packet = self.scanner_state.radio.get_latest_packet() 
            if scanner_packet:
                self._process_packet(DroneName.Scanner, scanner_packet)

            time.sleep(0.01)

    def _process_packet(self, drone: DroneName, packet: Dict[str, Any]) -> None:    #Process incoming packet and update drone state

        with self.lock:
            name = packet.get("name")
            password = packet.get("password")
            if(self.drone_states[drone.value].name == name and 
               self.drone_states[drone.value].password == password):
                #print("Here")
                self.drone_states[drone.value].status = packet.get("status", self.drone_states[drone.value].status)
                self.drone_states[drone.value].battery = packet.get("battery", self.drone_states[drone.value].battery)
                log = packet.get("log")
                self.drone_states[drone.value].log = log if log is not None else ""

                incoming_telemetry = packet.get("telemetry", {})

                for key in self.drone_states[drone.value].telemetry:
                    if key in incoming_telemetry:
                        self.drone_states[drone.value].telemetry[key] = incoming_telemetry[key]

                    else:
                        log_message("GCSController",f"Unknown packet: {packet}\n")

    def get_drone_state(self, drone: DroneName) -> Dict[str, Any]:          #Get current state of the specified drone

        with self.lock:
            t = self.drone_states[drone.value].telemetry.copy()
            status = self.drone_states[drone.value].status or "Idle"
            battery = self.drone_states[drone.value].battery if self.drone_states[drone.value].battery is not None else -1
            log = self.drone_states[drone.value].log
           
        ui_telemetry = parse_telemetry(t)
        state = {
            "status": status,
            "battery": battery,
            "log": log,
            "telemetry": t,
            "ui_telemetry": ui_telemetry,
        }
        return state

    # ---------------------------------------------------------
    # PUBLIC COMMAND FUNCTIONS (UI → Drone)
    # ---------------------------------------------------------
    def land(self, drone: DroneName) -> None:                       #Send land command to specified drone
        self.drone_states[drone.value].radio.send_command("LAND")

    def start(self, drone: DroneName) -> None:                      #Send start command to specified drone
        self.drone_states[drone.value].radio.send_command("START")
        with self.lock:
            self.drone_states[drone.value].lat0 = self.drone_states[drone.value].telemetry["lat"]
            self.drone_states[drone.value].lon0 = self.drone_states[drone.value].telemetry["lon"]
            self.drone_states[drone.value].started = True

    def rtl(self, drone: DroneName) -> None:                    #Send return-to-launch command to specified drone   
        self.drone_states[drone.value].radio.send_command("RTL")

    def set_mode(self, drone: DroneName, mode_name: str) -> None:           #Send set mode command to specified drone
        self.drone_states[drone.value].radio.send_command("SET_MODE", {"mode": mode_name})

    def send_coords(self, drone: DroneName, x: float, y: float, z: float) -> None:      #Send move command with coordinates to specified drone
        self.drone_states[drone.value].radio.send_command("MOVE", {"x": x, "y": y, "z": z})

    # ---------------------------------------------------------
    # STOP
    # ---------------------------------------------------------
    def stop(self) -> None:                 #Stop the GCS controller and its threads
        self._running = False
        self.listener_thread.join()
        print("[GCSController] Stopped")
