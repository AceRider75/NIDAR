import serial
import threading
import time
import sys

# PORT = "COM5"          # Windows → COMx
PORT = "/dev/ttyUSB0"  # Linux / macOS
BAUD = 57600
TIMEOUT = 1


class BaseStation:
    def __init__(self, port, baud):
        self.port = port
        self.baud = baud
        self.running = True
        self.ser = None
        self.connect()

    def connect(self):
        while True:
            try:
                self.ser = serial.Serial(
                    self.port,
                    self.baud,
                    timeout=TIMEOUT
                )
                print(f"[INFO] Connected to {self.port}")
                break
            except Exception as e:
                print(f"[ERROR] Waiting for radio... ({e})")
                time.sleep(2)

    def receiver(self):
        while self.running:
            try:
                if self.ser.in_waiting:
                    data = self.ser.readline().decode(errors="ignore").strip()
                    if data:
                        print(f"[RX] {data}")
            except Exception as e:
                print(f"[RX ERROR] {e}")
                self.reconnect()

    def transmitter(self):
        print("Type command and press ENTER:")
        while self.running:
            try:
                cmd = sys.stdin.readline().strip()
                if cmd:
                    self.ser.write((cmd + "\n").encode())
                    print(f"[TX] {cmd}")
            except Exception as e:
                print(f"[TX ERROR] {e}")
                self.reconnect()

    def reconnect(self):
        try:
            self.ser.close()
        except:
            pass
        print("[INFO] Reconnecting...")
        self.connect()

    def start(self):
        rx = threading.Thread(target=self.receiver, daemon=True)
        tx = threading.Thread(target=self.transmitter, daemon=True)
        rx.start()
        tx.start()

        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("\n[INFO] Shutting down")
            self.running = False
            self.ser.close()


if __name__ == "__main__":
    base = BaseStation(PORT, BAUD)
    base.start()
