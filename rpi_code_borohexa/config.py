# config.py
import os

BASE_DIR = os.path.dirname(os.path.abspath(__file__))

LOG_FILE = os.path.join(BASE_DIR, "logs", "events.log")
TELEMETRY_FILE = os.path.join(BASE_DIR, "logs", "telemetry.log")

# Create log directory if missing
os.makedirs(os.path.dirname(LOG_FILE), exist_ok=True)
