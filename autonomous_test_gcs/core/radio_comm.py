import threading
import time
import serial
from core.message_parser import json_to_dict, dict_to_json

class RadioComm:
    def __init__(self, 
                 port="/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0", 
                 baud=57600):
        self.port = port
        self.baud = baud
        self.serial = None

        self._running = False
        self.listener_thread = None
        self._latest_packet = None
        self._packet_lock = threading.Lock()

        self._connect()


    def _connect(self) -> None:
        try:
            self.serial = serial.Serial(
                self.port,
                self.baud,
                timeout=0.05,
            )
            print("[RadioComm] Connected to radio")
        except Exception as e:
            print(f"[RadioComm] ERROR: {e}")
            self.serial = None


    def start(self) -> None:
        if not self.serial:
            print("[RadioComm] Cannot start — no serial connection")
            return
        self._running = True
        self.listener_thread = threading.Thread(
            target=self._listen_loop,
            daemon=True
        )
        self.listener_thread.start()
        print("[RadioComm] Listener thread started")


    def stop(self) -> None:
        self._running = False
        if self.listener_thread:
            self.listener_thread.join()
        if self.serial and self.serial.is_open:
            self.serial.close()
        print("[RadioComm] Stopped")


    def _send_json(self, data: dict) -> None:
        if not self.serial:
            return
        json_msg = dict_to_json(data, indent=None)
        if json_msg is None:
            print("[RadioComm] JSON generation failed")
            return
        try:
            self.serial.write((json_msg + "\n").encode("utf-8"))
        except Exception as e:
            print(f"[RadioComm] Send Error: {e}")


    def send_command(self, command: str, params=None) -> None:
        packet = {
            "type": "command",
            "command": command,
            "params": params or {},
            "timestamp": time.time()
        }
        self._send_json(packet)


    def _listen_loop(self) -> None:
        buffer = ""

        while self._running:
            try:
                data = self.serial.read().decode("utf-8", errors="ignore")
                if not data:
                    continue

                buffer += data
                if "\n" in buffer:
                    line, buffer = buffer.split("\n", 1)
                    packet = json_to_dict(line)
                    if packet:
                        self._update_state(packet)

            except Exception as e:
                print(f"[RadioComm] Listen Error: {e}")
                time.sleep(0.5)

    def _update_state(self, packet: dict) -> None:
        with self._packet_lock:
            self._latest_packet = packet


    def get_latest_packet(self) -> dict:
        with self._packet_lock:
            pkt = self._latest_packet
            self._latest_packet = None   
            return pkt