import cv2
import numpy as np
from typing import List, Tuple, Dict, Optional
import logging
import os
import math
import csv
import socket
import json


try:
    # Prefer shared constants so paths stay consistent across processes
    from paths import TELEMETRY_FILE as DEFAULT_TELEMETRY_FILE
except Exception:
    DEFAULT_TELEMETRY_FILE = None

from dataclasses import dataclass, field
from collections import defaultdict
from datetime import datetime
from sklearn.cluster import DBSCAN
import cv2
import numpy as np
from typing import List, Tuple, Dict, Optional
import logging
import os
import math
from dataclasses import dataclass, field
from collections import defaultdict
from datetime import datetime
from sklearn.cluster import DBSCAN

# ========= SPOT LOGGING CONFIGURATION =========
# DO NOT CHANGE THIS PATH (your requested directory)
LOG_DIR = "/home/vihang/python_scripts/auto_test_with_rpi/rpi_code/logs"
os.makedirs(LOG_DIR, exist_ok=True)

LOG_FILE = os.path.join(LOG_DIR, "spot_data.log")

logging.basicConfig(
    level=logging.INFO,
    format="%(message)s",
    handlers=[
        logging.FileHandler(LOG_FILE, mode="a"),  # writes to file
        logging.StreamHandler()                    # prints to terminal
    ]
)
logger = logging.getLogger("spot_tracker")


def log_spot_data_logfile(spot_id: int, drone_coords: tuple, spot_coords: tuple):
    """
    Log spot data continuously into the log file in LOG_DIR in this exact format:
    2026-01-01 12:44:21 | spot_id=3 | drone_lat=..., drone_lon=..., drone_alt=... | spot_lat=..., spot_lon=...
    """
    timestamp = datetime.utcnow().strftime("%Y-%m-%d %H:%M:%S")

    entry = (
        f"{timestamp} | spot_id={spot_id} | "
        f"drone_lat={drone_coords[0]:.7f}, drone_lon={drone_coords[1]:.7f}, drone_alt={drone_coords[2]:.2f} | "
        f"spot_lat={spot_coords[0]:.7f}, spot_lon={spot_coords[1]:.7f}"
    )

    logger.info(entry)  # logged + printed


# Global queue to maintain last 30 detected yellow spots
_spot_queue: List[Dict] = []


def write_telemetry(spot_list: List, telemetry_file: str = DEFAULT_TELEMETRY_FILE,
                    drone_coords: Optional[Tuple[float, float, float]] = None,
                    max_queue_size: int = 30):
    """
    Write detected yellow spots to telemetry file, maintaining a rolling queue of last 30 spots.

    Args:
        spot_list: List of TrackedSpot objects detected in current frame
        telemetry_file: Path to telemetry CSV file to write to
        drone_coords: Optional (lat, lon, alt) tuple for drone position
        max_queue_size: Maximum number of spots to keep in queue (default 30)

    Returns:
        None

    Notes:
        - Maintains a global queue of the last 30 spots detected
        - Each new spot is appended to the queue; oldest spots are removed when queue exceeds max_queue_size
        - Writes the entire queue to the telemetry file (or separate spots file) in CSV format
        - Safe for concurrent writes using append mode
    """
    global _spot_queue

    if not telemetry_file:
        return

    # Add new spots to the queue
    for spot in spot_list:
        spot_data = {
            "timestamp": datetime.now().isoformat(),
            "spot_id": spot.id,
            "center_x": spot.center[0],
            "center_y": spot.center[1],
            "area": spot.area,
            "area_rank": spot.area_rank,
            "track_length": len(spot.track_history)
        }

        # Add drone coordinates if available
        if drone_coords:
            spot_data.update({
                "drone_lat": drone_coords[0],
                "drone_lon": drone_coords[1],
                "drone_alt": drone_coords[2]
            })

        _spot_queue.append(spot_data)

    # Maintain queue size limit (FIFO - remove oldest)
    if len(_spot_queue) > max_queue_size:
        _spot_queue = _spot_queue[-max_queue_size:]

    # Write queue to file
    try:
        # Create directory if it doesn't exist
        telemetry_dir = os.path.dirname(telemetry_file)
        if telemetry_dir and not os.path.exists(telemetry_dir):
            os.makedirs(telemetry_dir, exist_ok=True)

        # Determine if file exists to decide whether to write header
        file_exists = os.path.exists(telemetry_file)

        with open(telemetry_file, 'a', newline='') as f:
            if _spot_queue:
                # Define CSV columns based on available data
                if drone_coords:
                    fieldnames = ["timestamp", "spot_id", "center_x", "center_y", "area",
                                  "area_rank", "track_length", "drone_lat", "drone_lon", "drone_alt"]
                else:
                    fieldnames = ["timestamp", "spot_id", "center_x", "center_y", "area",
                                  "area_rank", "track_length"]

                writer = csv.DictWriter(f, fieldnames=fieldnames)

                # Write header only if file is new
                if not file_exists:
                    writer.writeheader()

                # Write all spots in the queue
                for spot_data in _spot_queue:
                    # Filter to only include relevant fields
                    filtered_data = {k: v for k,
                                     v in spot_data.items() if k in fieldnames}
                    writer.writerow(filtered_data)

    except Exception as e:
        logger.error(f"Failed to write telemetry for spots: {e}")


# Handle imports for both package and direct execution
if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG)

    def get_logger():
        return logging.getLogger(__name__)
else:
    try:
        from ..utils.logger import get_logger
    except ImportError:
        logging.basicConfig(level=logging.DEBUG)

        def get_logger():
            return logging.getLogger(__name__)


@dataclass
class TrackedSpot:
    """Represents a tracked yellow spot."""
    id: int
    center: Tuple[int, int]
    area: float
    contour: np.ndarray
    last_seen_frame: int
    first_seen_frame: int
    track_history: List[Tuple[int, int]] = field(default_factory=list)
    area_rank: int = 0  # Rank based on area (1 = largest)

    def update(self, center: Tuple[int, int], area: float, contour: np.ndarray, frame_num: int):
        """Update spot information."""
        self.center = center
        self.area = area
        self.contour = contour
        self.last_seen_frame = frame_num
        self.track_history.append(center)

        # Keep history limited to last 30 positions
        if len(self.track_history) > 30:
            self.track_history.pop(0)


class SpotTracker:
    """Tracks yellow spots across frames."""

    def __init__(self, max_distance: float = 50.0,
                 area_tolerance: float = 0.3,
                 max_frames_missing: int = 10,
                 dilation_kernel_size: int = 0,
                 clustering_distance: float = 50.0,
                 min_cluster_samples: int = 1):
        """
        Initialize spot tracker.

        Args:
            max_distance: Maximum distance for matching spots between frames
            area_tolerance: Maximum relative area change (0.3 = 30%)
            max_frames_missing: Max frames a spot can be missing before removed
            dilation_kernel_size: Kernel size for dilation to merge nearby contours (0 = no dilation)
            clustering_distance: Maximum distance between spots to be considered in same cluster
            min_cluster_samples: Minimum samples for DBSCAN clustering
        """
        self.max_distance = max_distance
        self.area_tolerance = area_tolerance
        self.max_frames_missing = max_frames_missing
        self.dilation_kernel_size = dilation_kernel_size
        self.clustering_distance = clustering_distance
        self.min_cluster_samples = min_cluster_samples

        self.tracked_spots: Dict[int, TrackedSpot] = {}
        self.next_id = 1
        self.current_frame = 0
        self.logger = get_logger()

    def _cluster_and_merge_spots(self, centers: List[Tuple[int, int]],
                                 contours: List[np.ndarray],
                                 mask: np.ndarray) -> Tuple[List[Tuple[int, int]], List[np.ndarray], List[float]]:
        """
        Cluster nearby spots and merge them using dilation.

        Args:
            centers: List of spot centers
            contours: List of spot contours
            mask: Binary mask of detected spots

        Returns:
            Tuple of (merged_centers, merged_contours, merged_areas)
        """
        if len(centers) == 0:
            return [], [], []

        # Convert centers to numpy array for clustering
        centers_array = np.array(centers)

        # Perform DBSCAN clustering
        clustering = DBSCAN(eps=self.clustering_distance,
                            min_samples=self.min_cluster_samples).fit(centers_array)
        labels = clustering.labels_

        # Group contours by cluster
        clusters = defaultdict(list)
        noise_points = []

        for idx, label in enumerate(labels):
            if label == -1:  # Noise point (outlier)
                noise_points.append(idx)
            else:
                clusters[label].append(idx)

        # Process each cluster
        merged_centers = []
        merged_contours = []
        merged_areas = []

        # Create a blank mask for merged clusters
        height, width = mask.shape

        for cluster_id, indices in clusters.items():
            # Create a mask for this cluster
            cluster_mask = np.zeros_like(mask)

            # Draw all contours in this cluster
            for idx in indices:
                cv2.drawContours(cluster_mask, [contours[idx]], -1, 255, -1)

            # Apply dilation to merge nearby spots in the cluster
            if self.dilation_kernel_size > 0:
                kernel = np.ones((self.dilation_kernel_size,
                                 self.dilation_kernel_size), np.uint8)
                cluster_mask = cv2.dilate(cluster_mask, kernel, iterations=1)

            # Find merged contours in the cluster
            cluster_contours, _ = cv2.findContours(cluster_mask, cv2.RETR_EXTERNAL,
                                                   cv2.CHAIN_APPROX_SIMPLE)

            # Process each merged contour
            for contour in cluster_contours:
                area = cv2.contourArea(contour)
                if area > 100:  # Filter small noise
                    M = cv2.moments(contour)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        merged_centers.append((cx, cy))
                        merged_contours.append(contour)
                        merged_areas.append(area)

        # Add noise points (outliers) as individual spots
        for idx in noise_points:
            # Apply dilation to individual noise points too
            if self.dilation_kernel_size > 0:
                noise_mask = np.zeros_like(mask)
                cv2.drawContours(noise_mask, [contours[idx]], -1, 255, -1)
                kernel = np.ones((self.dilation_kernel_size,
                                 self.dilation_kernel_size), np.uint8)
                noise_mask = cv2.dilate(noise_mask, kernel, iterations=1)

                # Re-extract contour
                noise_contours, _ = cv2.findContours(noise_mask, cv2.RETR_EXTERNAL,
                                                     cv2.CHAIN_APPROX_SIMPLE)
                if noise_contours:
                    contour = noise_contours[0]
                    area = cv2.contourArea(contour)
                    M = cv2.moments(contour)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        merged_centers.append((cx, cy))
                        merged_contours.append(contour)
                        merged_areas.append(area)
            else:
                merged_centers.append(centers[idx])
                merged_contours.append(contours[idx])
                merged_areas.append(cv2.contourArea(contours[idx]))

        self.logger.debug(f"Clustering: {len(centers)} spots -> {len(clusters)} clusters + "
                          f"{len(noise_points)} outliers = {len(merged_centers)} merged spots")

        return merged_centers, merged_contours, merged_areas

    def update(self, centers: List[Tuple[int, int]],
               contours: List[np.ndarray],
               areas: Optional[List[float]] = None,
               mask: Optional[np.ndarray] = None) -> Dict[int, TrackedSpot]:
        """
        Update tracking with new detections.

        Args:
            centers: List of spot centers
            contours: List of spot contours
            areas: Optional list of spot areas (calculated if not provided)
            mask: Optional mask for clustering and dilation processing

        Returns:
            Dictionary of currently tracked spots
        """
        self.current_frame += 1

        # Apply clustering followed by dilation if enabled
        if mask is not None and len(centers) > 0:
            centers, contours, areas = self._cluster_and_merge_spots(
                centers, contours, mask)
        elif areas is None:
            # Calculate areas if not provided and no clustering was applied
            areas = [cv2.contourArea(contour) for contour in contours]

        # Match new detections to existing tracks
        matched_tracks, unmatched_detections, unmatched_tracks = self._match_spots(
            centers, areas, contours
        )

        # Update matched tracks
        for track_id, detection_idx in matched_tracks.items():
            self.tracked_spots[track_id].update(
                centers[detection_idx],
                areas[detection_idx],
                contours[detection_idx],
                self.current_frame
            )

        # Create new tracks for unmatched detections
        for detection_idx in unmatched_detections:
            new_spot = TrackedSpot(
                id=self.next_id,
                center=centers[detection_idx],
                area=areas[detection_idx],
                contour=contours[detection_idx],
                last_seen_frame=self.current_frame,
                first_seen_frame=self.current_frame
            )
            new_spot.track_history.append(centers[detection_idx])
            self.tracked_spots[self.next_id] = new_spot
            self.next_id += 1
            timestamp = datetime.utcnow().strftime("%Y-%m-%d %H:%M:%S")
            self.logger.info(
                f"{timestamp} | new_spot_detected | spot_id={new_spot.id}")

        # Remove tracks that haven't been seen for too long
        tracks_to_remove = []
        for track_id in unmatched_tracks:
            frames_missing = self.current_frame - \
                self.tracked_spots[track_id].last_seen_frame
            if frames_missing > self.max_frames_missing:
                tracks_to_remove.append(track_id)
                timestamp = datetime.utcnow().strftime("%Y-%m-%d %H:%M:%S")
                self.logger.info(
                    f"{timestamp} | removed_lost_track | spot_id={track_id}")

        for track_id in tracks_to_remove:
            del self.tracked_spots[track_id]

        # Rank spots by area
        self._rank_spots_by_area()

        return self.tracked_spots

    def _match_spots(self, centers: List[Tuple[int, int]],
                     areas: List[float],
                     contours: List[np.ndarray]) -> Tuple[Dict[int, int], List[int], List[int]]:
        """
        Match new detections to existing tracks.

        Returns:
            Tuple of (matched_tracks, unmatched_detections, unmatched_tracks)
        """
        if not self.tracked_spots:
            return {}, list(range(len(centers))), []

        # Build cost matrix
        track_ids = list(self.tracked_spots.keys())
        cost_matrix = np.zeros((len(track_ids), len(centers)))

        for i, track_id in enumerate(track_ids):
            track = self.tracked_spots[track_id]
            for j, (center, area) in enumerate(zip(centers, areas)):
                # Calculate distance cost
                distance = np.sqrt((center[0] - track.center[0])**2 +
                                   (center[1] - track.center[1])**2)

                # Calculate area similarity cost
                area_diff = abs(area - track.area) / max(track.area, 1)

                # Combined cost
                if distance < self.max_distance and area_diff < self.area_tolerance:
                    cost_matrix[i, j] = distance + (area_diff * 100)
                else:
                    # Very high cost for invalid matches
                    cost_matrix[i, j] = 1e6

        # Greedy matching (simple approach)
        matched_tracks = {}
        matched_detections = set()

        for _ in range(min(len(track_ids), len(centers))):
            if cost_matrix.min() >= 1e6:
                break

            min_idx = np.unravel_index(cost_matrix.argmin(), cost_matrix.shape)
            track_idx, detection_idx = min_idx

            matched_tracks[track_ids[track_idx]] = detection_idx
            matched_detections.add(detection_idx)

            # Invalidate this row and column
            cost_matrix[track_idx, :] = 1e6
            cost_matrix[:, detection_idx] = 1e6

        # Find unmatched
        unmatched_detections = [i for i in range(
            len(centers)) if i not in matched_detections]
        unmatched_tracks = [
            track_id for track_id in track_ids if track_id not in matched_tracks]

        return matched_tracks, unmatched_detections, unmatched_tracks

    def _rank_spots_by_area(self):
        """Rank spots by area (largest = rank 1)."""
        sorted_spots = sorted(self.tracked_spots.values(),
                              key=lambda s: s.area, reverse=True)
        for rank, spot in enumerate(sorted_spots, start=1):
            spot.area_rank = rank

    def get_spot_by_rank(self, rank: int) -> Optional[TrackedSpot]:
        """Get spot by area rank."""
        for spot in self.tracked_spots.values():
            if spot.area_rank == rank:
                return spot
        return None

    def get_largest_spot(self) -> Optional[TrackedSpot]:
        """Get the largest spot."""
        return self.get_spot_by_rank(1)

    def visualize_tracks(self, image: np.ndarray,
                         show_history: bool = True,
                         show_rank: bool = True) -> np.ndarray:
        """
        Visualize tracked spots on image.


    def visualize_tracks(self, image: np.ndarray, 
                        show_history: bool = True,
                        show_r

    def visualize_tracks(self, image: np.ndarray, 
                        show_history: bool = True,
                        show_r
    def visualize_tracks(self, image: np.ndarray, 
                        show_history: bool = True,
                        show_r

        Args:
            image: Input image
            show_history: Whether to show track history
            show_rank: Whether to show area rank

        Returns:
            Image with visualizations
        """
        overlay = image.copy()

        for spot in self.tracked_spots.values():
            # Draw track history
            if show_history and len(spot.track_history) > 1:
                for i in range(len(spot.track_history) - 1):
                    cv2.line(overlay, spot.track_history[i], spot.track_history[i + 1],
                             (255, 0, 255), 2)

            # Draw contour
            cv2.drawContours(overlay, [spot.contour], -1, (0, 255, 0), 2)

            # Draw centroid
            cv2.circle(overlay, spot.center, 8, (0, 0, 255), -1)
            cv2.circle(overlay, spot.center, 12, (255, 255, 255), 2)

            # Draw crosshair
            cv2.line(overlay, (spot.center[0] - 15, spot.center[1]),
                     (spot.center[0] + 15, spot.center[1]), (255, 255, 255), 2)
            cv2.line(overlay, (spot.center[0], spot.center[1] - 15),
                     (spot.center[0], spot.center[1] + 15), (255, 255, 255), 2)

            # Prepare label
            if show_rank:
                label = f"#{spot.area_rank} (ID:{spot.id})"
            else:
                label = f"ID:{spot.id}"

            area_label = f"Area: {spot.area:.0f}px"
            centroid_label = f"({spot.center[0]}, {spot.center[1]})"

            # Position labels
            text_x = spot.center[0] + 20
            text_y = spot.center[1] - 30

            # Draw labels with outline
            cv2.putText(overlay, label, (text_x, text_y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3)
            cv2.putText(overlay, label, (text_x, text_y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

            cv2.putText(overlay, centroid_label, (text_x, text_y + 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3)
            cv2.putText(overlay, centroid_label, (text_x, text_y + 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

            cv2.putText(overlay, area_label, (text_x, text_y + 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3)
            cv2.putText(overlay, area_label, (text_x, text_y + 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Add summary
        summary = f"Tracked Spots: {len(self.tracked_spots)}"
        cv2.putText(overlay, summary, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 3)
        cv2.putText(overlay, summary, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

        return overlay

    def reset(self):
        """Reset tracker state."""
        self.tracked_spots.clear()
        self.next_id = 1
        self.current_frame = 0


# ========= OLD SEPARATE SOCKET CLIENT (COMMENTED OUT) =========
# class SpotSocketClient:
#     def __init__(self, host="127.0.0.1", port=5005):
#         self.host = host
#         self.port = port
#         self.sock = None
#         self.connected = False
#         self._connect()
#
#     def _connect(self):
#         try:
#             self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#             self.sock.connect((self.host, self.port))
#             self.connected = True
#             print("[SpotSocket] Connected to drone controller")
#         except Exception as e:
#             print(f"[SpotSocket] Connection failed: {e}")
#             self.connected = False
#
#     def send_spots(self, payload: dict):
#         if not self.connected:
#             self._connect()
#             return
#
#         try:
#             msg = json.dumps(payload).encode()
#             self.sock.sendall(msg + b"\n")  # newline = message delimiter
#         except Exception as e:
#             print(f"[SpotSocket] Send failed: {e}")
#             self.connected = False
# ========= END OLD SEPARATE SOCKET CLIENT =========


class DroneManagerSocket:
    """
    Unified socket client for communicating with drone_manager.
    Handles both:
    - Receiving telemetry data (lat, lon, alt, yaw)
    - Sending detected spot data
    """

    def __init__(self, host: str = "127.0.0.1", port: int = 5005):
        """
        Initialize the unified drone manager socket.

        Args:
            host: Host address of drone_manager socket server
            port: Port number of drone_manager socket server
        """
        self.host = host
        self.port = port
        self.sock = None
        self.connected = False
        self.latest_telemetry: Optional[Dict[str, float]] = None
        self.buffer = ""
        self._connect()

    def _connect(self):
        """Establish socket connection to drone_manager."""
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            # Short timeout for non-blocking behavior
            self.sock.settimeout(0.05)
            self.sock.connect((self.host, self.port))
            self.connected = True
            print(
                f"[DroneManagerSocket] Connected to drone_manager at {self.host}:{self.port}")
        except Exception as e:
            print(f"[DroneManagerSocket] Connection failed: {e}")
            self.connected = False

    def _reconnect(self):
        """Attempt to reconnect if connection was lost."""
        if self.sock:
            try:
                self.sock.close()
            except:
                pass
        self._connect()

    def _receive_telemetry(self):
        """
        Try to receive telemetry data from drone_manager.
        Non-blocking: updates internal cache if new data available.
        """
        if not self.connected:
            return

        try:
            data = self.sock.recv(1024).decode('utf-8')
            if data:
                self.buffer += data

                # Process complete messages (newline delimited JSON)
                while '\n' in self.buffer:
                    line, self.buffer = self.buffer.split('\n', 1)
                    line = line.strip()
                    if line:
                        try:
                            msg = json.loads(line)
                            # Check if it's a telemetry message
                            if msg.get("type") == "telemetry" or "telemetry" in msg:
                                telem = msg.get("telemetry", msg)
                                if all(key in telem for key in ['lat', 'lon', 'alt', 'yaw']):
                                    self.latest_telemetry = {
                                        "lat": float(telem['lat']),
                                        "lon": float(telem['lon']),
                                        "alt": float(telem['alt']),
                                        "yaw": float(telem['yaw'])
                                    }
                        except json.JSONDecodeError:
                            pass
                        except (KeyError, ValueError):
                            pass
        except socket.timeout:
            pass  # No data available
        except ConnectionResetError:
            print("[DroneManagerSocket] Connection reset, will reconnect...")
            self.connected = False
        except Exception as e:
            print(f"[DroneManagerSocket] Receive error: {e}")
            self.connected = False

    def get_latest_telemetry(self) -> Optional[Dict[str, float]]:
        """
        Get the latest telemetry data from drone_manager.

        Returns:
            Dictionary with lat, lon, alt, yaw or None if not available
        """
        if not self.connected:
            self._reconnect()

        self._receive_telemetry()
        return self.latest_telemetry

    def send_spots(self, payload: dict):
        """
        Send detected spot data to drone_manager.

        Args:
            payload: Dictionary containing spot detection data
        """
        if not self.connected:
            self._reconnect()
            if not self.connected:
                return

        try:
            msg = json.dumps(payload).encode('utf-8')
            self.sock.sendall(msg + b"\n")
        except Exception as e:
            print(f"[DroneManagerSocket] Send failed: {e}")
            self.connected = False

    def close(self):
        """Close the socket connection."""
        if self.sock:
            try:
                self.sock.close()
            except:
                pass
            self.connected = False
            print("[DroneManagerSocket] Connection closed")


# ========= OLD FILE-BASED TELEMETRY READER (COMMENTED OUT) =========
# def get_latest_telemetry(telemetry_file: str) -> Optional[Dict[str, float]]:
#     """
#     Read the latest telemetry data from the log file.
#
#     Args:
#         telemetry_file: Path to the telemetry log file
#
#     Returns:
#         Dictionary with telemetry data or None if read fails
#     """
#     try:
#         if not os.path.exists(telemetry_file):
#             return None
#
#         # Read the last few lines efficiently
#         with open(telemetry_file, 'rb') as f:
#             try:
#                 # Go to the end of the file
#                 f.seek(0, os.SEEK_END)
#                 file_size = f.tell()
#
#                 # Read last 1KB (should cover at least one line)
#                 seek_offset = min(file_size, 1024)
#                 f.seek(-seek_offset, os.SEEK_END)
#
#                 lines = f.readlines()
#
#                 if not lines:
#                     return None
#
#                 # Get the last non-empty line
#                 last_line = lines[-1].decode().strip()
#                 if not last_line and len(lines) > 1:
#                     last_line = lines[-2].decode().strip()
#
#             except OSError:
#                 return None
#
#         if not last_line:
#             return None
#
#         # Parse CSV line
#         # Format: time, status, battery, lat, lon, alt, vx, vy, vz, roll, pitch, yaw, xacc, yacc
#         parts = last_line.split(',')
#         if len(parts) < 12:
#             return None
#
#         return {
#             "lat": float(parts[3]),
#             "lon": float(parts[4]),
#             "alt": float(parts[5]),
#             "yaw": float(parts[11])
#         }
#     except Exception as e:
#         # logging.error(f"Error reading telemetry: {e}")
#         return None
# ========= END OLD FILE-BASED TELEMETRY READER =========

def calculate_real_coords(drone_lat: float, drone_lon: float, drone_alt: float, drone_yaw: float,
                          spot_center: Tuple[int, int], image_size: Tuple[int, int],
                          fov_h: float = 62.2) -> Tuple[float, float]:
    """
    Calculate real-world coordinates of a spot.

    Args:
        drone_lat: Drone latitude
        drone_lon: Drone longitude
        drone_alt: Drone altitude in meters
        drone_yaw: Drone yaw in degrees
        spot_center: Spot center (x, y) in pixels
        image_size: Image size (width, height) in pixels
        fov_h: Horizontal Field of View in degrees

    Returns:
        Tuple of (spot_lat, spot_lon)
    """
    img_w, img_h = image_size
    cx, cy = spot_center

    # Calculate offsets from center in pixels
    dx_px = cx - (img_w / 2)
    dy_px = (img_h / 2) - cy  # Y is inverted in image coordinates (top is 0)

    # Calculate angular offsets
    # Assume linear projection for simplicity (valid for small angles)
    # FOV_V = FOV_H * (H / W)
    fov_v = fov_h * (img_h / img_w)

    angle_x = (dx_px / img_w) * fov_h
    angle_y = (dy_px / img_h) * fov_v

    # Calculate ground distances relative to drone
    # x is right, y is forward (relative to camera frame)
    # We assume camera is looking straight down (nadir)

    dist_x = drone_alt * math.tan(math.radians(angle_x))
    dist_y = drone_alt * math.tan(math.radians(angle_y))

    # Rotate distances by drone yaw to get North/East offsets
    # Yaw is usually 0 = North, 90 = East
    # Camera x (right) corresponds to East (at 0 yaw)
    # Camera y (forward) corresponds to North (at 0 yaw)

    yaw_rad = math.radians(drone_yaw)

    # NED frame offsets
    # x_b = forward (camera y), y_b = right (camera x)
    x_b = dist_y
    y_b = dist_x

    north_offset = x_b * math.cos(yaw_rad) - y_b * math.sin(yaw_rad)
    east_offset = x_b * math.sin(yaw_rad) + y_b * math.cos(yaw_rad)

    # Convert offsets to lat/lon
    R = 6371000.0  # Earth radius
    d_lat = (north_offset / R) * (180 / math.pi)
    d_lon = (east_offset / (R * math.cos(math.radians(drone_lat)))) * \
        (180 / math.pi)

    return drone_lat + d_lat, drone_lon + d_lon


def log_spot_data(spot_id: int, drone_coords: Tuple[float, float, float],
                  spot_coords: Tuple[float, float], filename: str = "spot_data.csv"):
    """
    Log spot data to CSV.

    Args:
        spot_id: ID of the spot
        drone_coords: (lat, lon, alt)
        spot_coords: (lat, lon)
        filename: Output CSV file
    """
    # Use append mode which is generally safe for concurrent writes on Linux (atomic for small writes)
    # We open and close the file each time to minimize lock holding time if any
    try:
        file_exists = os.path.exists(filename)

        with open(filename, 'a', newline='') as f:
            writer = csv.writer(f)
            if not file_exists:
                writer.writerow(["time", "spot_id", "drone_lat",
                                "drone_lon", "drone_alt", "spot_lat", "spot_lon"])

            current_time = datetime.now().strftime("%H:%M:%S")
            writer.writerow([
                current_time,
                spot_id,
                f"{drone_coords[0]:.7f}",
                f"{drone_coords[1]:.7f}",
                f"{drone_coords[2]:.2f}",
                f"{spot_coords[0]:.7f}",
                f"{spot_coords[1]:.7f}"
            ])
    except Exception as e:
        print(f"Failed to log spot data: {e}")


def main():
    """Run SpotTracker service safely with proper logging and video recording."""
    from color_detector import ColorDetector
    from picamera2 import Picamera2
    import sys
    import time

    # Setup directories
    current_dir = os.path.dirname(os.path.abspath(__file__))
    root_dir = os.path.dirname(os.path.dirname(current_dir))
    LOG_DIR = os.path.join(root_dir, "logs")
    os.makedirs(LOG_DIR, exist_ok=True)

    # Initialize camera
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(
        main={"size": (640, 480), "format": "RGB888"})
    picam2.configure(config)
    picam2.start()
    time.sleep(1)  # Give camera time to start

    # Prepare video output
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    raw_path = os.path.join(LOG_DIR, f"feed_raw_{timestamp}.mp4")
    annotated_path = os.path.join(LOG_DIR, f"feed_annotated_{timestamp}.mp4")
    raw_writer = None
    annotated_writer = None
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")

    # Initialize detector and tracker
    detector = ColorDetector()
    tracker = SpotTracker(
        max_distance=80,
        max_frames_missing=5,
        dilation_kernel_size=15,
        clustering_distance=60.0,
        min_cluster_samples=1
    )

    # ========= OLD SEPARATE SOCKET SETUP (COMMENTED OUT) =========
    # socket_client = SpotSocketClient(
    #     host="127.0.0.1",  # same machine
    #     port=5005
    # )
    # # Telemetry and spot log files (use shared constant when available)
    # telemetry_file = DEFAULT_TELEMETRY_FILE or os.path.join(root_dir, "data", "telemetry", "telemetry_live.csv")
    # if not os.path.exists(telemetry_file):
    #     print(f"Warning: telemetry file missing at {telemetry_file}; spot logging will have no GPS until controller writes it")
    # print(f"Telemetry file: {telemetry_file}")
    # ========= END OLD SEPARATE SOCKET SETUP =========

    # Unified socket connection to drone_manager for both telemetry and spot data
    drone_socket = DroneManagerSocket(
        host="127.0.0.1",
        port=5005  # Single port for bidirectional communication with drone_manager
    )

    spot_log_file = os.path.join(LOG_DIR, "spot_data.log")
    print(f"Spot tracking service started. Logs: {LOG_DIR}")
    print(f"Drone manager socket: 127.0.0.1:5005")
    print(f"Spot log file: {spot_log_file}")

    try:
        while True:
            frame = picam2.capture_array()
            if frame is None:
                time.sleep(0.01)
                continue

            height, width = frame.shape[:2]

            # Initialize video writers only once
            if raw_writer is None:
                raw_writer = cv2.VideoWriter(
                    raw_path, fourcc, 30.0, (width, height))
                annotated_writer = cv2.VideoWriter(
                    annotated_path, fourcc, 30.0, (width, height))
                if not raw_writer.isOpened() or not annotated_writer.isOpened():
                    print("Failed to open video writers. Exiting.")
                    break

            # Detect yellow spots
            centers, contours, mask = detector.detect_yellow_spots(frame)

            # Update tracker with mask for dilation
            tracked_spots = tracker.update(centers, contours, mask=mask)

            # Read latest telemetry from drone_manager via socket
            telemetry = drone_socket.get_latest_telemetry()

            # ========= OLD FILE-BASED TELEMETRY READ (COMMENTED OUT) =========
            # telemetry = get_latest_telemetry(telemetry_file)
            # ========= END OLD FILE-BASED TELEMETRY READ =========

            if telemetry is None:
                # Surface a gentle warning once per run to highlight telemetry pipeline issues
                if not hasattr(main, "warned_no_telemetry"):
                    print(
                        "Warning: No telemetry data available yet; check that drone_manager is running and sending telemetry via socket")
                    main.warned_no_telemetry = True

            # Log spot data continuously
            if telemetry and tracked_spots:
                payload = {
                    "type": "yellow_spots",
                    "timestamp": time.time(),
                    "image_size": [width, height],
                    "spots": []
                }

                for spot in tracked_spots.values():
                    spot_lat, spot_lon = calculate_real_coords(
                        telemetry["lat"], telemetry["lon"], telemetry["alt"], telemetry["yaw"],
                        spot.center, (width, height)
                    )
                    log_spot_data_logfile(
                        spot.id,
                        (telemetry["lat"], telemetry["lon"], telemetry["alt"]),
                        (spot_lat, spot_lon)
                    )

                    payload["spots"].append({
                        "id": spot.id,
                        "lat": f"{spot_lat:.7f}",
                        "lon": f"{spot_lon:.7f}",
                        "cx": spot.center[0],
                        "cy": spot.center[1],
                        "area": spot.area,
                        "rank": spot.area_rank
                    })

                # Send spots via unified drone_manager socket
                drone_socket.send_spots(payload)

            # Visualize overlay
            overlay = tracker.visualize_tracks(
                frame, show_history=True, show_rank=True)
            if telemetry:
                telem_str = f"Alt: {telemetry['alt']:.1f}m Yaw: {telemetry['yaw']:.1f}"
                cv2.putText(overlay, telem_str, (10, height - 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

            # Write frames to video
            raw_writer.write(frame)
            annotated_writer.write(overlay)

            # Optional: print largest spot info
            largest = tracker.get_largest_spot()
            if largest:
                print(f"Largest spot: ID={largest.id}, Rank={largest.area_rank}, "
                      f"Center={largest.center}, Area={largest.area:.2f}px²")

            # Sleep briefly to avoid CPU overload (adjust as needed)
            time.sleep(0.01)

    except KeyboardInterrupt:
        print("Spot tracker service stopped by user.")

    finally:
        # Release resources safely
        if raw_writer is not None:
            raw_writer.release()
        if annotated_writer is not None:
            annotated_writer.release()
        picam2.stop()
        cv2.destroyAllWindows()

        # Close socket connection
        drone_socket.close()

        print(f"Videos saved: {raw_path}, {annotated_path}")
        print(f"Spot log saved: {spot_log_file}")


if __name__ == "__main__":
    main()
