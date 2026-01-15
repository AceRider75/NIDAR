import json
import serial
import threading
import time
from utils import json_to_dict, dict_to_json
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
        self._cmd_queue = []
        self._cmd_lock = threading.Lock()
        self._rx_buffer = ""  # Buffer for partial line reception

        self._connect()

    def _connect(self):
        try:
            self.serial = serial.Serial(
                self.port,
                self.baud,
                timeout=0.5  # Increased timeout for more reliable line reception
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

                # Read available data and add to buffer
                chunk = self.serial.readline().decode("utf-8", errors="ignore")
                if not chunk:
                    continue
                
                self._rx_buffer += chunk

                # Process complete lines from buffer
                while "\n" in self._rx_buffer:
                    line, self._rx_buffer = self._rx_buffer.split("\n", 1)
                    line = line.strip()
                    if not line:
                        continue
                    
                    print(f"[DEBUG] Raw line received: {repr(line)}")

                    # Try to extract valid JSON objects from potentially corrupted line
                    packets = self._extract_json_objects(line)
                    if packets:
                        for packet in packets:
                            print(f"[DEBUG] Received packet: {packet}")
                            if packet.get("type") == "command":
                                print(f"[DEBUG] COMMAND RECEIVED: {packet}")
                                with self._lock:
                                    self._latest_command = packet
                    else:
                        print(f"[DEBUG] Failed to parse JSON from line: {repr(line)}")

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

    def _extract_json_objects(self, line: str):
        """
        Extract valid JSON objects from a potentially corrupted line.
        Uses brace depth scanning to find complete JSON objects.
        """
        results = []
        depth = 0
        start_idx = None
        for i, ch in enumerate(line):
            if ch == '{':
                if depth == 0:
                    start_idx = i
                depth += 1
            elif ch == '}':
                if depth > 0:
                    depth -= 1
                    if depth == 0 and start_idx is not None:
                        candidate = line[start_idx:i+1]
                        parsed = json_to_dict(candidate)
                        if parsed is not None:
                            results.append(parsed)
                        start_idx = None
        return results

    def get_latest_command(self):
        with self._lock:
            cmd = self._latest_command
            self._latest_command = None
            return cmd

    # -------------------------------------------------
    # TX
    # -------------------------------------------------
    def send_packet(self, packet: dict):
        if self.serial is None:
            return
        try:
            packet["timestamp"] = time.time()
            data = json.dumps(packet) + "\n"  # newline frame
            self.serial.write(data.encode('utf-8'))
            self.serial.flush()
        except Exception as e:
            print(f"[DEBUG] Error sending packet: {e}")
