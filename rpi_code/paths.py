import os

BASE_DIR = os.path.dirname(os.path.abspath(__file__))

LOG_DIR = os.path.join(BASE_DIR, "data", "logs")
TELEMETRY_DIR = os.path.join(BASE_DIR, "data", "telemetry")


LOG_FILE = os.path.join(LOG_DIR, "logs_live.csv")
TELEMETRY_FILE = os.path.join(TELEMETRY_DIR, "telemetry_live.csv")