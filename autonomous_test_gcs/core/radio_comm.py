import threading
import time
import serial
from core.message_parser import json_to_dict, dict_to_json
from utils.logger import log_message

class RadioComm:                #Handles low-level radio communication with the drone
    def __init__(self, 
                 port=r"\\.\COM5",    #Windows style COM port (For Linux/Mac, use "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0")
                 baud=57600):
        self.port = port
        self.baud = baud
        self.serial = None

        self._running = False
        self.listener_thread = None
        self._latest_packet = None          #Holds the latest received data packet
        self._packet_lock = threading.Lock()

        self._connect()


    def _connect(self) -> None:         #Establish serial connection to the radio
        try:
            self.serial = serial.Serial(
                self.port,
                self.baud,
                timeout=0.05,
            )
            print("[RadioComm] Connected to radio")
            log_message("GCS","Radio Connected\n")
        except Exception as e:
            print(f"[RadioComm] ERROR: {e}")
            self.serial = None


    def start(self) -> None:        #Start the listener thread to receive incoming packets
        if not self.serial:
            print("[RadioComm] Cannot start — no serial connection")
            log_message("GCS","Radio start failed — no serial connection\n")
            return
        self._running = True
        self.listener_thread = threading.Thread(
            target=self._listen_loop,
            daemon=True
        )
        self.listener_thread.start()        
        print("[RadioComm] Listener thread started")
        log_message("GCS","Radio listener started\n")


    def stop(self) -> None:         #Stop the listener thread and close the serial connection
        self._running = False
        if self.listener_thread:
            self.listener_thread.join()
        if self.serial and self.serial.is_open:
            self.serial.close()
        print("[RadioComm] Stopped")
        log_message("GCS","Radio stopped\n")


    def _send_json(self, data: dict) -> None:       #Send a JSON-formatted packet over the radio
        if not self.serial:
            return
        json_msg = dict_to_json(data, indent=None)
        if json_msg is None:
            print("[RadioComm] JSON generation failed")
            log_message("GCS","Radio TX failed — JSON generation failed\n")
            return
        try:
            self.serial.write((json_msg + "\n").encode("utf-8"))       #Send JSON string with newline delimiter 
        except Exception as e:
            print(f"[RadioComm] Send Error: {e}")
            log_message("GCS",f"Radio TX error: {e}\n")

    def send_command(self, command: str, params=None) -> None:      #Send a command packet to the drone
        packet = {
            "type": "command",
            "command": command,
            "params": params or {},
            "timestamp": time.time()
        }
        self._send_json(packet)


    def _listen_loop(self) -> None:         #Continuously listen for incoming packets from the drone
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

    def _update_state(self, packet: dict) -> None:          #Update the latest received packet in a thread-safe manner
        with self._packet_lock:
            self._latest_packet = packet


    def get_latest_packet(self) -> dict:        #Retrieve and clear the latest received packet in a thread-safe manner
        with self._packet_lock:
            pkt = self._latest_packet
            self._latest_packet = None   
            return pkt
