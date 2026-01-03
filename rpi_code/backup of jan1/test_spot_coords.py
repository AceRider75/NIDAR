import unittest
import math
import os
from datetime import datetime
import sys

# Add spot_tracker path
sys.path.append('/home/vihang/python_scripts/auto_test_with_rpi/rpi_code/src/image_processing')

# Mock cv2, picamera2, sklearn if not available
try:
    import cv2
except ImportError:
    from unittest.mock import MagicMock
    sys.modules['cv2'] = MagicMock()

try:
    import picamera2
except ImportError:
    from unittest.mock import MagicMock
    sys.modules['picamera2'] = MagicMock()

try:
    import sklearn
    from sklearn.cluster import DBSCAN
except ImportError:
    from unittest.mock import MagicMock
    sys.modules['sklearn'] = MagicMock()
    sys.modules['sklearn.cluster'] = MagicMock()

# Import functions to test
from spot_tracker import calculate_real_coords, get_latest_telemetry

# Set up logs directory
LOG_DIR = "/home/vihang/python_scripts/auto_test_with_rpi/rpi_code/logs"
os.makedirs(LOG_DIR, exist_ok=True)

def log_spot_data_logfile(spot_id: int, drone_coords: tuple, spot_coords: tuple, log_filename: str = "spot_data.log"):
    """
    Log spot data continuously into a log file in LOG_DIR.
    """
    log_path = os.path.join(LOG_DIR, log_filename)
    timestamp = datetime.utcnow().strftime("%Y-%m-%d %H:%M:%S")
    
    log_entry = (
        f"{timestamp} | spot_id={spot_id} | "
        f"drone_lat={drone_coords[0]:.7f}, drone_lon={drone_coords[1]:.7f}, drone_alt={drone_coords[2]:.2f} | "
        f"spot_lat={spot_coords[0]:.7f}, spot_lon={spot_coords[1]:.7f}\n"
    )
    
    with open(log_path, "a") as f:
        f.write(log_entry)


class TestSpotTracker(unittest.TestCase):

    def test_calculate_real_coords_center(self):
        drone_lat = 10.0
        drone_lon = 20.0
        drone_alt = 100.0
        drone_yaw = 0.0
        spot_center = (320, 240)
        image_size = (640, 480)

        lat, lon = calculate_real_coords(drone_lat, drone_lon, drone_alt, drone_yaw, spot_center, image_size)
        self.assertAlmostEqual(lat, drone_lat)
        self.assertAlmostEqual(lon, drone_lon)

    def test_calculate_real_coords_offset_north(self):
        drone_lat = 0.0
        drone_lon = 0.0
        drone_alt = 100.0
        drone_yaw = 0.0
        image_size = (640, 480)
        spot_center = (320, 120)  # 120 px up

        lat, lon = calculate_real_coords(drone_lat, drone_lon, drone_alt, drone_yaw, spot_center, image_size)

        expected_dlat = (20.65 / 6371000.0) * (180 / math.pi)
        self.assertAlmostEqual(lat, drone_lat + expected_dlat, places=5)
        self.assertAlmostEqual(lon, drone_lon, places=5)

    def test_calculate_real_coords_offset_east(self):
        drone_lat = 0.0
        drone_lon = 0.0
        drone_alt = 100.0
        drone_yaw = 0.0
        image_size = (640, 480)
        spot_center = (480, 240)  # 160 px right

        lat, lon = calculate_real_coords(drone_lat, drone_lon, drone_alt, drone_yaw, spot_center, image_size)
        expected_dlon = (27.8 / 6371000.0) * (180 / math.pi)
        self.assertAlmostEqual(lat, drone_lat, places=5)
        self.assertAlmostEqual(lon, drone_lon + expected_dlon, places=5)

    def test_calculate_real_coords_yaw_90(self):
        drone_lat = 0.0
        drone_lon = 0.0
        drone_alt = 100.0
        drone_yaw = 90.0
        image_size = (640, 480)
        spot_center = (320, 120)

        lat, lon = calculate_real_coords(drone_lat, drone_lon, drone_alt, drone_yaw, spot_center, image_size)
        expected_dlon = (20.65 / 6371000.0) * (180 / math.pi)
        self.assertAlmostEqual(lat, drone_lat, places=5)
        self.assertAlmostEqual(lon, drone_lon + expected_dlon, places=4)

    def test_telemetry_reading(self):
        filename = "test_telemetry.log"
        with open(filename, "w") as f:
            f.write("header\n")
            f.write("12:00:00,Active,99,10.5,20.5,50.0,0,0,0,0,0,180.0,0,0\n")

        telem = get_latest_telemetry(filename)
        self.assertIsNotNone(telem)
        self.assertEqual(telem["lat"], 10.5)
        self.assertEqual(telem["lon"], 20.5)
        self.assertEqual(telem["alt"], 50.0)
        self.assertEqual(telem["yaw"], 180.0)
        
        os.remove(filename)

    def test_log_spot_data_logfile(self):
        log_filename = "test_spot_data.log"
        log_path = os.path.join(LOG_DIR, log_filename)
        if os.path.exists(log_path):
            os.remove(log_path)

        log_spot_data_logfile(1, (10.0, 20.0, 50.0), (10.0001, 20.0001), log_filename)

        self.assertTrue(os.path.exists(log_path))
        with open(log_path, "r") as f:
            lines = f.readlines()
            self.assertEqual(len(lines), 1)
            self.assertIn("spot_id=1", lines[0])
            self.assertIn("10.0001000", lines[0])

        # Append another entry to test continuous logging
        log_spot_data_logfile(2, (10.1, 20.1, 55.0), (10.1001, 20.1001), log_filename)
        with open(log_path, "r") as f:
            lines = f.readlines()
            self.assertEqual(len(lines), 2)
            self.assertIn("spot_id=2", lines[1])

        os.remove(log_path)


if __name__ == '__main__':
    unittest.main()
