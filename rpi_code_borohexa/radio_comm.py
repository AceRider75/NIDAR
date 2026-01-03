import threading
import time
import serial
from message_parser import json_to_dict, dict_to_json
from utils import log_message

class RadioComm:
    def __init__(self, port="/dev/ttyAMA0", baud=57600):
        self.port = port
        self.baud = baud
        self.serial = None

        self._running = False
        self._rx_thread = None
        self._latest_command = None
        self._lock = threading.Lock()

        self._connect()

    def _connect(self):
        try:
            self.serial = serial.Serial(
                self.port,
                self.baud,
                timeout=0.2
            )
            print(f"[DEBUG] Radio Connected on {self.port} at {self.baud} baud")
            log_message("RPi","Radio Connected\n")
        except Exception as e:
            print(f"[DEBUG] Radio Connection Failed: {e}")
            log_message("RPi",f"Radio Connection Failed: {e}\n")
            self.serial = None

    def start(self):
        if not self.serial:
            print(f"[DEBUG] Cannot start radio - serial is None")
            return
        print(f"[DEBUG] Starting radio listen loop")
        self._running = True
        self._rx_thread = threading.Thread(
            target=self._listen_loop,
            daemon=True
        )
        self._rx_thread.start()

    def stop(self):
        self._running = False
        if self._rx_thread:
            self._rx_thread.join()
        if self.serial and self.serial.is_open:
            self.serial.close()

    # -------------------------------------------------
    # RX
    # -------------------------------------------------
    def _listen_loop(self):
        while self._running:
            try:
                # Check if serial port is still open and valid
                if not self.serial or not self.serial.is_open:
                    log_message("RPi","Radio disconnected, attempting reconnect\n")
                    time.sleep(1)
                    self._connect()
                    if not self.serial:
                        time.sleep(5)  # Wait longer before retry
                        continue
                    continue

                # Use readline to get complete lines
                line = self.serial.readline().decode("utf-8", errors="ignore").strip()
                
                if not line:
                    continue
                    
                print(f"[DEBUG] Raw line received: {repr(line)}")

                packet = json_to_dict(line)
                if not packet:
                    print(f"[DEBUG] Failed to parse JSON from line: {repr(line)}")
                    continue

                print(f"[DEBUG] Received packet: {packet}")
                if packet.get("type") == "command":
                    print(f"[DEBUG] COMMAND RECEIVED: {packet}")
                    with self._lock:
                        self._latest_command = packet

            except serial.SerialException as e:
                log_message("RPi",f"Radio SerialException: {e}\n")
                # Close the bad connection
                if self.serial:
                    try:
                        self.serial.close()
                    except:
                        pass
                    self.serial = None
                time.sleep(1)
                
            except OSError as e:
                log_message("RPi",f"Radio OSError: {e}\n")
                # Close the bad connection
                if self.serial:
                    try:
                        self.serial.close()
                    except:
                        pass
                    self.serial = None
                time.sleep(1)
                
            except Exception as e:
                log_message("RPi",f"Radio RX error: {e}\n")
                time.sleep(0.1)

    def get_latest_command(self):
        with self._lock:
            cmd = self._latest_command
            self._latest_command = None
            return cmd

    # -------------------------------------------------
    # TX
    # -------------------------------------------------
    def send_packet(self, packet: dict):
        if not self.serial or not self.serial.is_open:
            return

        json_msg = dict_to_json(packet, indent=None)
        if json_msg is None:
            log_message("RPi","Radio JSON encoding failed\n")
            return

        try:
            self.serial.write((json_msg + "\n").encode("utf-8"))
        except serial.SerialException as e:
            log_message("RPi",f"Radio TX SerialException: {e}\n")
            # Mark connection as bad
            if self.serial:
                try:
                    self.serial.close()
                except:
                    pass
                self.serial = None
        except Exception as e:
            log_message("RPi",f"Radio TX error: {e}\n")
