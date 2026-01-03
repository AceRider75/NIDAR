import os
import csv


from datetime import datetime



BASE_DIR = os.path.dirname(os.path.abspath(__file__))

LOG_DIR = os.path.join(BASE_DIR, "data", "logs")

LOG_FILE = os.path.join(
    LOG_DIR,
    f"logs_{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.csv"
)



# Ensure log directory exists before creating file
os.makedirs(LOG_DIR, exist_ok=True)

with open(LOG_FILE, "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["time", "device", "message"])

TELEMETRY_DIR = os.path.join(BASE_DIR, "data", "telemetry")

TELEMETRY_FILE = os.path.join(
    TELEMETRY_DIR,
    f"telemetry_{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.csv"
)
# Ensure telemetry directory exists before creating file
os.makedirs(TELEMETRY_DIR, exist_ok=True)

with open(TELEMETRY_FILE, "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow([
        "time",
        "status",
        "battery",
        "lat",
        "lon",
        "alt",
        "vx",
        "vy",
        "vz",
        "roll",
        "pitch",
        "yaw",

        "xacc",
        "yacc"
    ])
    


def main():
    # Import here to avoid circular imports at module import time
    from drone_manager import DroneManager

    drone = DroneManager()
    try:
        drone.run()
    except KeyboardInterrupt:
        print("Shutting down drone manager")
        try:
            if hasattr(drone, 'Controller') and drone.Controller:
                drone.Controller.stop()
        except Exception:
            pass
        try:
            if hasattr(drone, 'radio') and drone.radio:
                drone.radio.stop()
        except Exception:
            pass

if __name__ == "__main__":
    main()

