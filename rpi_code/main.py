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
            "time", "status", "battery", "lat", "lon", "alt",
            "vx", "vy", "vz", "roll", "pitch", "yaw", "xacc", "yacc"
        ])


def main():
    # Import here to avoid circular imports at module import time
    from drone_manager import DroneManager
    import signal
    import sys
    import time

    drone = DroneManager()
    
    def signal_handler(sig, frame):
        print("\nShutting down drone manager...")
        cleanup(drone)
        sys.exit(0)
    
    # Register signal handlers for graceful shutdown
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    try:
        # Auto-start mission when run directly
        print("=" * 60)
        print("AUTONOMOUS MISSION MODE")
        print("=" * 60)
        print("Waiting 3 seconds before auto-starting mission...")
        print("Press Ctrl+C to cancel...")
        time.sleep(3)
        
        # Send START command to begin mission automatically
        start_packet = {
            "command": "START",
            "params": {}
        }
        print("\n>>> Sending AUTO-START command...")
        drone.handle_command(start_packet)
        print(">>> Mission started automatically!\n")
        
        # Run the main drone manager loop
        drone.run()
        
    except KeyboardInterrupt:
        print("\nShutting down drone manager")
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
    try:
        if hasattr(drone, 'spot_socket') and drone.spot_socket:
            drone.spot_socket.stop()
    except Exception as e:
        print(f"Error stopping spot_socket: {e}")


if __name__ == "__main__":
    main()