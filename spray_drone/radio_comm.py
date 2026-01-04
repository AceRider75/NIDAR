import threading
import time
import serial
from utils import json_to_dict, dict_to_json, log_message
import hashlib

class RadioComm:
    """
    Radio communication with reliable command handling
    Features:
    - Robust JSON parsing with frame delimiters
    - Command deduplication (executes each command_id only once)
    - Automatic reconnection on serial errors
    - Acknowledgment system
    """
    
    def __init__(self, port="/dev/ttyAMA0", baud=57600):
        self.port = port
        self.baud = baud
        self.serial = None

        self._running = False
        self._rx_thread = None
        self._latest_command = None
        self._lock = threading.Lock()
        
        # Command deduplication - tracks executed command IDs
        self._executed_commands = set()
        self._executed_lock = threading.Lock()
        self._max_command_history = 100  # Keep last 100 command IDs
        
        # Message framing
        self._rx_buffer = ""
        self.START_MARKER = "<START>"
        self.END_MARKER = "<END>"
        
        self._connect()

    def _connect(self):
        """Establish serial connection"""
        try:
            self.serial = serial.Serial(
                self.port,
                self.baud,
                timeout=0.2
            )
            print(f"[RadioComm] Connected on {self.port} at {self.baud} baud")
            log_message("RPi", "Radio Connected")
        except Exception as e:
            print(f"[RadioComm] Connection Failed: {e}")
            log_message("RPi", f"Radio Connection Failed: {e}")
            self.serial = None

    def start(self):
        """Start radio communication thread"""
        if not self.serial:
            print(f"[RadioComm] Cannot start - serial is None")
            return
        
        print(f"[RadioComm] Starting radio listen loop")
        self._running = True
        self._rx_thread = threading.Thread(
            target=self._listen_loop,
            daemon=True
        )
        self._rx_thread.start()

    def stop(self):
        """Stop radio communication"""
        self._running = False
        if self._rx_thread:
            self._rx_thread.join()
        if self.serial and self.serial.is_open:
            self.serial.close()

    # ==========================================================================
    # RECEIVE (RX)
    # ==========================================================================
    
    def _listen_loop(self):
        """Main receive loop with robust frame parsing"""
        while self._running:
            try:
                # Check serial connection health
                if not self.serial or not self.serial.is_open:
                    log_message("RPi", "Radio disconnected, reconnecting...")
                    time.sleep(1)
                    self._connect()
                    if not self.serial:
                        time.sleep(5)
                        continue
                    continue

                # Read available data
                if self.serial.in_waiting > 0:
                    chunk = self.serial.read(self.serial.in_waiting).decode("utf-8", errors="ignore")
                    self._rx_buffer += chunk
                    
                    # Process complete frames
                    self._process_buffer()
                
                time.sleep(0.01)  # Small delay to prevent CPU spinning

            except serial.SerialException as e:
                log_message("RPi", f"Radio SerialException: {e}")
                self._close_serial()
                time.sleep(1)
                
            except OSError as e:
                log_message("RPi", f"Radio OSError: {e}")
                self._close_serial()
                time.sleep(1)
                
            except Exception as e:
                log_message("RPi", f"Radio RX error: {e}")
                time.sleep(0.1)

    def _process_buffer(self):
        """Extract and process complete frames from buffer"""
        while self.START_MARKER in self._rx_buffer and self.END_MARKER in self._rx_buffer:
            try:
                # Find frame boundaries
                start_idx = self._rx_buffer.find(self.START_MARKER)
                end_idx = self._rx_buffer.find(self.END_MARKER, start_idx)
                
                if start_idx == -1 or end_idx == -1:
                    break
                
                # Extract frame
                frame_start = start_idx + len(self.START_MARKER)
                frame_data = self._rx_buffer[frame_start:end_idx]
                
                # Remove processed frame from buffer
                self._rx_buffer = self._rx_buffer[end_idx + len(self.END_MARKER):]
                
                # Parse JSON
                packet = json_to_dict(frame_data)
                
                if packet:
                    self._handle_packet(packet)
                else:
                    print(f"[RadioComm] Failed to parse JSON: {repr(frame_data[:100])}")
                    log_message("RPi", f"JSON parse failed")
                
            except Exception as e:
                print(f"[RadioComm] Buffer processing error: {e}")
                # Clear buffer on error
                self._rx_buffer = ""
                break
        
        # Prevent buffer overflow - keep only reasonable amount
        if len(self._rx_buffer) > 10000:
            print(f"[RadioComm] Buffer overflow, clearing")
            self._rx_buffer = ""

    def _handle_packet(self, packet: dict):
        """Handle received packet with deduplication"""
        packet_type = packet.get("type")
        
        if packet_type == "command":
            command_id = packet.get("command_id")
            
            if not command_id:
                print(f"[RadioComm] Command without command_id, ignoring")
                return
            
            # Check if already executed
            with self._executed_lock:
                if command_id in self._executed_commands:
                    print(f"[RadioComm] Duplicate command {command_id}, ignoring")
                    # Send ACK anyway (in case previous ACK was lost)
                    self._send_ack(command_id, "duplicate")
                    return
                
                # Mark as executed
                self._executed_commands.add(command_id)
                
                # Limit history size
                if len(self._executed_commands) > self._max_command_history:
                    # Remove oldest (convert to list, remove first, convert back)
                    cmd_list = list(self._executed_commands)
                    self._executed_commands = set(cmd_list[-self._max_command_history:])
            
            # Store for processing
            print(f"[RadioComm] New command {command_id}: {packet.get('command')}")
            with self._lock:
                self._latest_command = packet
            
            # Send acknowledgment
            self._send_ack(command_id, "received")
        
        else:
            # Handle other packet types (telemetry responses, etc.)
            print(f"[RadioComm] Received packet type: {packet_type}")

    def _send_ack(self, command_id: str, status: str):
        """Send acknowledgment for received command"""
        ack_packet = {
            "type": "ack",
            "command_id": command_id,
            "status": status,
            "timestamp": time.time()
        }
        self.send_packet(ack_packet)

    def _close_serial(self):
        """Safely close serial connection"""
        if self.serial:
            try:
                self.serial.close()
            except:
                pass
            self.serial = None

    def get_latest_command(self):
        """Get latest command (thread-safe)"""
        with self._lock:
            cmd = self._latest_command
            self._latest_command = None
            return cmd

    # ==========================================================================
    # TRANSMIT (TX)
    # ==========================================================================
    
    def send_packet(self, packet: dict):
        """Send packet with frame markers for reliable transmission"""
        if not self.serial or not self.serial.is_open:
            return

        json_msg = dict_to_json(packet, indent=None)
        if json_msg is None:
            log_message("RPi", "JSON encoding failed")
            return

        try:
            # Add frame markers
            frame = f"{self.START_MARKER}{json_msg}{self.END_MARKER}\n"
            self.serial.write(frame.encode("utf-8"))
            
        except serial.SerialException as e:
            log_message("RPi", f"Radio TX SerialException: {e}")
            self._close_serial()
            
        except Exception as e:
            log_message("RPi", f"Radio TX error: {e}")

