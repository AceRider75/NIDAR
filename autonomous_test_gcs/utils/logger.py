import csv
import os
from datetime import datetime

# Compute project base (autonomous_test_gcs) without importing app to avoid circular imports
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

LOG_DIR = os.path.join(BASE_DIR, "data", "logs")
TELEMETRY_DIR = os.path.join(BASE_DIR, "data", "telemetry")

os.makedirs(LOG_DIR, exist_ok=True)
os.makedirs(TELEMETRY_DIR, exist_ok=True)

LOG_FILE = os.path.join(LOG_DIR, f"logs_{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.csv")
TELEMETRY_FILE = os.path.join(TELEMETRY_DIR, f"telemetry_{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.csv")

# Create files with headers if they don't exist
with open(LOG_FILE, "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["time", "device", "message"])

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


def log_message(device: str, message: str) -> str:
    current_time = datetime.now().strftime("%H:%M:%S")

    with open(LOG_FILE, "a", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([current_time, device, message])

    logged_message = f"{current_time} | [{device}]{message}"
    print(logged_message)

    return logged_message


def log_telemetry(data: dict) -> None:
    current_time = datetime.now().strftime("%H:%M:%S")
    telemetry = data.get("telemetry", {})

    row = [
        current_time,
        data.get("status"),
        data.get("battery"),
        telemetry.get("lat"),
        telemetry.get("lon"),
        telemetry.get("alt"),
        telemetry.get("vx"),
        telemetry.get("vy"),
        telemetry.get("vz"),
        telemetry.get("roll"),
        telemetry.get("pitch"),
        telemetry.get("yaw"),
        telemetry.get("xacc"),
        telemetry.get("yacc"),
    ]

    with open(TELEMETRY_FILE, "a", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(row)
