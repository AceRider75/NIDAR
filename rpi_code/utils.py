import math
import csv
import time
from datetime import datetime
from paths import LOG_FILE, TELEMETRY_FILE




def haversine_dist(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    #returns distance in meters
    R = 6371000.0
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlambda/2)**2
    c = 2*math.atan2(math.sqrt(a), math.sqrt(1-a))
    return R * c                

def meters_to_latlon(lat0: float, lon0: float, dx: float, dy: float) -> float:
    R = 6371000.0  # earth radius in meters
    lat0_rad = math.radians(lat0)
    dlat = (dy / R) * (180.0 / math.pi)
    dlon = (dx / (R * math.cos(lat0_rad))) * (180.0 / math.pi)

    return lat0 + dlat, lon0 + dlon

def log_message(device: str, message: str) -> str:

    

    with open(LOG_FILE, "a", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([time.time(), device, message])
    
    logged_message = f"{time.time()} | [{device}]{message}"
    print(logged_message)

    return logged_message

def log_telemetry(data: dict) -> None:
    
    
    telemetry = data.get("telemetry", {})

    row = [
        time.time(),
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