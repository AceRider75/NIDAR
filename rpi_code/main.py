import os
import csv


from datetime import datetime




from paths import LOG_FILE, TELEMETRY_FILE, LOG_DIR, TELEMETRY_DIR, BASE_DIR


os.makedirs(LOG_DIR, exist_ok=True)
os.makedirs(TELEMETRY_DIR, exist_ok=True)
tele_exists = os.path.exists(TELEMETRY_FILE)
log_exists = os.path.exists(LOG_FILE)
# Ensure log directory exists before creating file


with open(LOG_FILE, "a", newline="") as f:
    writer = csv.writer(f)
    if not log_exists:
    
        writer.writerow(["time", "device", "message"])

# Ensure telemetry directory exists before creating file


with open(TELEMETRY_FILE, "a", newline="") as f:
    writer = csv.writer(f)
    if not tele_exists:
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
    import signal
    import sys

    drone = DroneManager()
    
    def signal_handler(sig, frame):
        print("\nShutting down drone manager...")
        cleanup(drone)
        sys.exit(0)
    
    # Register signal handlers for graceful shutdown
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    try:
        drone.run()
    except KeyboardInterrupt:
        print("Shutting down drone manager")
        cleanup(drone)
    except Exception as e:
        print(f"Error in drone manager: {e}")
        cleanup(drone)
        raise

def cleanup(drone):
    """Ensure proper cleanup of resources"""
    try:
        if hasattr(drone, 'Controller') and drone.Controller:
            drone.Controller.stop()
    except Exception as e:
        print(f"Error stopping Controller: {e}")
    try:
        if hasattr(drone, 'radio') and drone.radio:
            drone.radio.stop()
    except Exception as e:
        print(f"Error stopping radio: {e}")

if __name__ == "__main__":
    main()