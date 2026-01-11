import serial

PORT = "/dev/ttyUSB0"   # change if needed (COMx on Windows)
BAUD = 57600
TIMEOUT = 1

ser = serial.Serial(
    port=PORT,
    baudrate=BAUD,
    timeout=TIMEOUT
)

print("[INFO] SiK receiver started")

try:
    while True:
        data = ser.readline()
        if data:
            print("[RX]", data.decode(errors="ignore").strip())
except KeyboardInterrupt:
    print("\n[INFO] Stopping receiver")
    ser.close()
