import time
import threading
from typing import List, Dict, Any, Tuple

from core.drone_state import DroneState, DroneName
from core.radio_comm import RadioComm
from core.telemetry_parser import parse_telemetry
from utils.logger import log_message


class GCSController:

    def __init__(self):
        self.drone_states: List[DroneState] = [None] * len(DroneName)

        self.sprayer_state = DroneState(
            name="Sprayer",
            password="vihang@2025",
            radio=RadioComm(port="/dev/ttyUSB1")
            # radio=RadioComm(port=r"")
        )

        self.scanner_state = DroneState(
            name="Scanner",
            password="vihang@2025",
            radio=RadioComm(port=r"/dev/ttyUSB0")
        )

        self.lock = threading.Lock()
        self._running = True

        self._init_drone_states()

        self.listener_thread = threading.Thread(
            target=self._update_loop,
            daemon=True
        )
        self.listener_thread.start()

    # ---------------------------------------------------------
    # INIT
    # ---------------------------------------------------------
    def _init_drone_states(self) -> None:
        self.drone_states[DroneName.Scanner.value] = self.scanner_state
        self.drone_states[DroneName.Sprayer.value] = self.sprayer_state

        self.scanner_state.radio.start()
        self.sprayer_state.radio.start()

    # ---------------------------------------------------------
    # LISTENING AND UPDATING
    # ---------------------------------------------------------
# ---------------------------------------------------------
# LISTENING AND UPDATING
    # ---------------------------------------------------------
    def _update_loop(self) -> None:
        while self._running:
            # --- Sprayer ---
            pkt = self.sprayer_state.radio.get_latest_packet()
            if pkt:
                # print("GCS GOT SPRAYER PACKET", pkt)
                try:
                    self._process_packet(DroneName.Sprayer, pkt)
                except Exception as e:
                    log_message("GCSController", f"Error processing sprayer packet: {e}")

            # --- Scanner ---
            pkt = self.scanner_state.radio.get_latest_packet()
            if pkt:
                # print("GCS GOT SCANNER PACKET", pkt)
                try:
                    self._process_packet(DroneName.Scanner, pkt)
                except Exception as e:
                    log_message("GCSController", f"Error processing scanner packet: {e}")

            time.sleep(0.01)


    # ---------------------------------------------------------
    # PACKET PROCESSING
    # ---------------------------------------------------------
    def _sanitize_value(self, value, default=0.0):
        """Convert value to float, return default if None or invalid."""
        if value is None:
            return default
        try:
            return float(value)
        except (ValueError, TypeError):
            return default

    def _process_packet(self, drone: DroneName, packet: Dict[str, Any]) -> None:
        with self.lock:
            state = self.drone_states[drone.value]

            # --- AUTH ---
            if packet.get("name") != state.name:
                return
            pw = packet.get("password")
            if pw is not None and pw != state.password:
                log_message("GCSController", f"Rejected packet for {packet.get('name')} - bad password\n")
                return

            # --- BASIC STATE ---
            state.status = packet.get("status", packet.get("state", state.status))

            if "battery" in packet:
                state.battery = self._sanitize_value(packet["battery"], -1)

            if packet.get("log") is not None:
                state.log = packet["log"]

            # --- TELEMETRY ---
            incoming_telemetry = packet.get("telemetry", packet)

            if isinstance(incoming_telemetry, dict):
                for key, value in incoming_telemetry.items():
                    if key in state.telemetry:
                        state.telemetry[key] = self._sanitize_value(value, 0.0)
            else:
                log_message("GCSController", f"Unexpected telemetry format: {incoming_telemetry}")

            # --- YELLOW SPOTS ---
            if "yellow_spots" in packet:
                self._process_yellow_spots(state, packet["yellow_spots"])

    # def _process_yellow_spots(self, state: DroneState, spots: List[Dict[str, Any]]) -> None:
    #     """Process and store yellow spot detections with full details."""
    #     with state.yellow_spots_lock:
    #         for spot in spots:
    #             spot_id = str(spot.get("id"))
    #             if not spot_id:
    #                 continue
                
    #             try:
    #                 coord_data = {
    #                     "lat": float(spot.get("lat", 0)),
    #                     "lon": float(spot.get("lon", 0)),
    #                     "cx": int(spot.get("cx", 0)),
    #                     "cy": int(spot.get("cy", 0)),
    #                     "area": float(spot.get("area", 0)),
    #                     "rank": int(spot.get("rank", 0)),
    #                     "timestamp": time.time()
    #                 }
                    
    #                 if spot_id not in state.yellow_spots:
    #                     state.yellow_spots[spot_id] = []
    #                     log_message("GCSController", f"New spot detected: ID={spot_id}")
                    
    #                 state.yellow_spots[spot_id].append(coord_data)
                    
    #                 log_message("GCSController", 
    #                     f"Spot {spot_id}: ({coord_data['lat']:.7f}, {coord_data['lon']:.7f}) "
    #                     f"Rank={coord_data['rank']}, Area={coord_data['area']:.0f}px - "
    #                     f"Total coords: {len(state.yellow_spots[spot_id])}")
                        
    #             except (ValueError, TypeError) as e:
    #                 log_message("GCSController", f"Error processing spot {spot_id}: {e}")

    def _process_yellow_spots(self, state: DroneState, spots: List[Dict[str, Any]]) -> None:
        """Process and store yellow spot detections with full details."""
        with state.yellow_spots_lock:
            current_time = time.time()
            
            for spot in spots:
                spot_id = str(spot.get("id"))
                if not spot_id:
                    continue
                
                try:
                    coord_data = {
                        "lat": float(spot.get("lat", 0)),
                        "lon": float(spot.get("lon", 0)),
                        "cx": int(spot.get("cx", 0)),
                        "cy": int(spot.get("cy", 0)),
                        "area": float(spot.get("area", 0)),
                        "rank": int(spot.get("rank", 0)),
                        "timestamp": current_time
                    }
                    
                    if spot_id not in state.yellow_spots:
                        state.yellow_spots[spot_id] = []
                        log_message("GCSController", f"New spot detected: ID={spot_id}")
                        state.yellow_spots[spot_id].append(coord_data)
                    else:
                        # Only add if this is a NEW detection (not from the same packet burst)
                        # Check if last detection was more than 0.5 seconds ago
                        last_detection = state.yellow_spots[spot_id][-1]
                        time_since_last = current_time - last_detection["timestamp"]
                        
                        # Only add if enough time has passed (new unique detection)
                        if time_since_last > 0.5:  # 500ms threshold
                            state.yellow_spots[spot_id].append(coord_data)
                            log_message("GCSController", 
                                f"Spot {spot_id}: ({coord_data['lat']:.7f}, {coord_data['lon']:.7f}) "
                                f"Rank={coord_data['rank']}, Area={coord_data['area']:.0f}px - "
                                f"Total coords: {len(state.yellow_spots[spot_id])}")
                    
                except (ValueError, TypeError) as e:
                    log_message("GCSController", f"Error processing spot {spot_id}: {e}")

    # PUBLIC STATE ACCESS
    # ---------------------------------------------------------
    def get_drone_state(self, drone: DroneName) -> Dict[str, Any]:
        with self.lock:
            telemetry = self.drone_states[drone.value].telemetry.copy()
            status = self.drone_states[drone.value].status or "Idle"
            battery = self.drone_states[drone.value].battery if self.drone_states[drone.value].battery is not None else -1
            log = self.drone_states[drone.value].log
            
            state = self.drone_states[drone.value]
            with state.yellow_spots_lock:
                yellow_spots = {spot_id: coords.copy() for spot_id, coords in state.yellow_spots.items()}
                spot_count = len(yellow_spots)
                coord_count = sum(len(coords) for coords in yellow_spots.values())

        ui_telemetry = parse_telemetry(telemetry)

        return {
            "status": status,
            "battery": battery,
            "log": log,
            "telemetry": telemetry,
            "ui_telemetry": ui_telemetry,
            "yellow_spots": yellow_spots,
            "yellow_spots_count": spot_count,
            "yellow_spots_coords_count": coord_count
        }

    def get_yellow_spots(self, drone: DroneName) -> Dict[str, List[Dict]]:
        """Get all yellow spots detected by the drone."""
        with self.lock:
            state = self.drone_states[drone.value]
            with state.yellow_spots_lock:
                return {spot_id: coords.copy() for spot_id, coords in state.yellow_spots.items()}

    def get_yellow_spots_summary(self, drone: DroneName) -> Dict[str, Any]:
        """Get summary of yellow spots with statistics."""
        with self.lock:
            state = self.drone_states[drone.value]
            with state.yellow_spots_lock:
                summary = {
                    "total_spots": len(state.yellow_spots),
                    "total_coordinates": sum(len(coords) for coords in state.yellow_spots.values()),
                    "spots": {}
                }
                
                for spot_id, coords in state.yellow_spots.items():
                    if coords:
                        summary["spots"][spot_id] = {
                            "detection_count": len(coords),
                            "first_seen": coords[0]["timestamp"],
                            "last_seen": coords[-1]["timestamp"],
                            "latest_rank": coords[-1]["rank"],
                            "avg_area": sum(c["area"] for c in coords) / len(coords),
                            "coordinates": [
                                {
                                    "lat": c["lat"],
                                    "lon": c["lon"],
                                    "timestamp": c["timestamp"]
                                }
                                for c in coords
                            ]
                        }
                
                return summary

    def get_spot_centers(self, drone: DroneName, min_detections: int = 2) -> List[Tuple[float, float]]:
        """
        Get averaged center coordinates for each spot that has been detected at least min_detections times.
        Returns list of (lat, lon) tuples representing the average position of each valid spot.
        
        Args:
            drone: Which drone to get spots from
            min_detections: Minimum number of detections required for a spot to be considered valid (default: 2)
        """
        with self.lock:
            state = self.drone_states[drone.value]
            with state.yellow_spots_lock:
                centers = []
                for spot_id, coords in state.yellow_spots.items():
                    # Only include spots that have been detected min_detections or more times
                    if coords and len(coords) >= min_detections:
                        avg_lat = sum(c["lat"] for c in coords) / len(coords)
                        avg_lon = sum(c["lon"] for c in coords) / len(coords)
                        centers.append((avg_lat, avg_lon))
                return centers

    def get_valid_yellow_spots(self, drone: DroneName, min_detections: int = 2) -> Dict[str, Dict]:
        """
        Get yellow spots that have been detected at least min_detections times.
        Returns averaged coordinates for each valid spot.
        
        Args:
            drone: Which drone to get spots from
            min_detections: Minimum number of detections required for a spot to be considered valid (default: 2)
            
        Returns:
            Dictionary with spot_id as key and dict with avg lat, lon, area, detection_count as value
        """
        with self.lock:
            state = self.drone_states[drone.value]
            with state.yellow_spots_lock:
                valid_spots = {}
                for spot_id, coords in state.yellow_spots.items():
                    if coords and len(coords) >= min_detections:
                        avg_lat = sum(c["lat"] for c in coords) / len(coords)
                        avg_lon = sum(c["lon"] for c in coords) / len(coords)
                        avg_area = sum(c["area"] for c in coords) / len(coords)
                        valid_spots[spot_id] = {
                            "lat": avg_lat,
                            "lon": avg_lon,
                            "area": avg_area,
                            "detection_count": len(coords),
                            "rank": coords[-1].get("rank", 0),
                            "first_seen": coords[0]["timestamp"],
                            "last_seen": coords[-1]["timestamp"]
                        }
                return valid_spots

    def clear_yellow_spots(self, drone: DroneName) -> None:
        """Clear all stored yellow spots for a drone."""
        with self.lock:
            state = self.drone_states[drone.value]
            with state.yellow_spots_lock:
                spot_count = len(state.yellow_spots)
                coord_count = sum(len(coords) for coords in state.yellow_spots.values())
                state.yellow_spots.clear()
                log_message("GCSController", 
                    f"Cleared {spot_count} spots ({coord_count} coordinates) for {state.name}")

    def export_yellow_spots_csv(self, drone: DroneName, filepath: str) -> bool:
        """
        Export yellow spots data to CSV file.
        
        Args:
            drone: Which drone's spots to export
            filepath: Path to save CSV file
            
        Returns:
            True if export successful, False otherwise
        """
        import csv
        
        try:
            with self.lock:
                state = self.drone_states[drone.value]
                with state.yellow_spots_lock:
                    spots_data = {spot_id: coords.copy() for spot_id, coords in state.yellow_spots.items()}
            
            with open(filepath, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(["spot_id", "detection_num", "timestamp", "lat", "lon", 
                                "cx", "cy", "area", "rank"])
                
                for spot_id, coords in spots_data.items():
                    for i, coord in enumerate(coords, 1):
                        writer.writerow([
                            spot_id,
                            i,
                            coord["timestamp"],
                            f"{coord['lat']:.7f}",
                            f"{coord['lon']:.7f}",
                            coord["cx"],
                            coord["cy"],
                            f"{coord['area']:.2f}",
                            coord["rank"]
                        ])
            
            log_message("GCSController", f"Exported spots to {filepath}")
            return True
            
        except Exception as e:
            log_message("GCSController", f"Failed to export spots: {e}")
            return False

    # ---------------------------------------------------------
    # COMMANDS (UI → DRONE)
    # ---------------------------------------------------------
    def land(self, drone: DroneName) -> None:
        self.drone_states[drone.value].radio.send_command("LAND")

    def start(self, drone: DroneName) -> None:
        self.drone_states[drone.value].radio.send_command("START")
        with self.lock:
            state = self.drone_states[drone.value]
            state.lat0 = state.telemetry.get("lat")
            state.lon0 = state.telemetry.get("lon")
            state.started = True

    def rtl(self, drone: DroneName) -> None:
        self.drone_states[drone.value].radio.send_command("RTL")

    def set_mode(self, drone: DroneName, mode_name: str) -> None:
        self.drone_states[drone.value].radio.send_command(
            "SET_MODE",
            {"mode": mode_name}
        )

    def send_coords(self, drone: DroneName, x: float, y: float, z: float) -> None:
        self.drone_states[drone.value].radio.send_command(
            "MOVE",
            {"x": x, "y": y, "z": z}
        )

    def transfer_waypoints_to_sprayer(self, scanner_drone: DroneName = DroneName.Scanner, min_detections: int = 2) -> bool:
        """
        Transfer detected yellow spot centers from scanner to sprayer drone incrementally.
        Only transfers spots that have been detected at least min_detections times.
        Also checks for imported waypoints if no yellow spots are available.
        Sends START_WAYPOINTS, then ADD_WAYPOINT for each, and END_WAYPOINTS.
        
        Args:
            scanner_drone: Which drone to get spots from
            min_detections: Minimum number of detections required for a spot to be considered valid (default: 2)
        """
        try:
            # Check if sprayer radio is connected
            sprayer_radio = self.drone_states[DroneName.Sprayer.value].radio
            if not sprayer_radio.serial or not sprayer_radio.serial.is_open:
                log_message("GCSController", "Sprayer radio not connected - cannot transfer waypoints")
                return False

            # Get total spots vs valid spots for logging
            all_spots = self.get_yellow_spots(scanner_drone)
            total_spots = len(all_spots)
            
            # Get only valid yellow spots from scanner (filtered by min_detections)
            centers = self.get_spot_centers(scanner_drone, min_detections=min_detections)
            
            if centers:
                log_message("GCSController", 
                    f"Valid spots: {len(centers)}/{total_spots} (min {min_detections} detections)")
            
            # If no valid yellow spots, check for imported/set waypoints
            if not centers:
                if hasattr(self, 'sprayer_waypoints') and self.sprayer_waypoints:
                    # Filter imported waypoints by detection_count as well
                    valid_imported = [wp for wp in self.sprayer_waypoints 
                                     if wp.get('detection_count', min_detections) >= min_detections]
                    if valid_imported:
                        log_message("GCSController", f"No valid yellow spots, using {len(valid_imported)} imported waypoints")
                        centers = [(wp.get('lat', wp.get('latitude')), wp.get('lon', wp.get('longitude'))) 
                                   for wp in valid_imported 
                                   if wp.get('lat', wp.get('latitude')) and wp.get('lon', wp.get('longitude'))]
                
            if not centers:
                log_message("GCSController", f"No valid spots (≥{min_detections} detections) or imported waypoints to transfer")
                return False

            default_alt = 4.0
            total = len(centers)
            log_message("GCSController", f"Transferring {total} waypoints to Sprayer...")

            # Start batch - send multiple times for reliability
            for _ in range(2):  # Send START twice for reliability
                self.drone_states[DroneName.Sprayer.value].radio.send_command(
                    "START_WAYPOINTS",
                    {"expected_count": total, "altitude": default_alt}
                )
                time.sleep(0.3)  # 300ms delay

            # Send one at a time with sufficient delay for radio transmission
            for idx, (lat, lon) in enumerate(centers, start=1):
                self.drone_states[DroneName.Sprayer.value].radio.send_command(
                    "ADD_WAYPOINT",
                    {"index": idx, "lat": lat, "lon": lon, "alt": default_alt}
                )
                time.sleep(0.3)  # 300ms delay per packet to prevent radio buffer overflow

            # End batch - send multiple times for reliability
            for _ in range(2):  # Send END twice for reliability
                self.drone_states[DroneName.Sprayer.value].radio.send_command(
                    "END_WAYPOINTS",
                    {"expected_count": total}
                )
                time.sleep(0.3)  # 300ms delay

            log_message("GCSController",
                        f"Transferred {total} waypoints to Sprayer drone (incremental)")
            return True

        except Exception as e:
            log_message("GCSController", f"Failed to transfer waypoints: {e}")
            return False

    def transfer_waypoints_detailed(self, scanner_drone: DroneName = DroneName.Scanner, 
                                   altitude: float = 4.0, 
                                   use_weighted_centers: bool = True,
                                   min_detections: int = 2) -> bool:
        """
        Incremental transfer with detailed options.
        Only transfers spots that have been detected at least min_detections times.
        
        Args:
            scanner_drone: Which drone to get spots from
            altitude: Altitude for waypoints
            use_weighted_centers: Whether to weight by area
            min_detections: Minimum number of detections required for a spot to be considered valid (default: 2)
        """
        try:
            with self.lock:
                state = self.drone_states[scanner_drone.value]
                with state.yellow_spots_lock:
                    if not state.yellow_spots:
                        log_message("GCSController", "No spots to transfer")
                        return False

                    # Build list first to know expected_count
                    # Only include spots with min_detections or more coordinates
                    total_spots = len(state.yellow_spots)
                    points = []
                    for spot_id, coords in state.yellow_spots.items():
                        if not coords or len(coords) < min_detections:
                            continue
                        if use_weighted_centers:
                            total_weight = sum(c.get("area", 1.0) for c in coords)
                            if total_weight > 0:
                                weighted_lat = sum(c["lat"] * c.get("area", 1.0) for c in coords) / total_weight
                                weighted_lon = sum(c["lon"] * c.get("area", 1.0) for c in coords) / total_weight
                                points.append((weighted_lat, weighted_lon))
                        else:
                            avg_lat = sum(c["lat"] for c in coords) / len(coords)
                            avg_lon = sum(c["lon"] for c in coords) / len(coords)
                            points.append((avg_lat, avg_lon))

            if not points:
                log_message("GCSController", f"No valid waypoints (≥{min_detections} detections) to transfer")
                return False

            total = len(points)
            log_message("GCSController", 
                f"Valid spots: {total}/{total_spots} (min {min_detections} detections)")
            
            # Start batch - send multiple times for reliability
            for _ in range(2):
                self.drone_states[DroneName.Sprayer.value].radio.send_command(
                    "START_WAYPOINTS",
                    {"expected_count": total, "altitude": altitude}
                )
                time.sleep(0.3)

            for idx, (lat, lon) in enumerate(points, start=1):
                self.drone_states[DroneName.Sprayer.value].radio.send_command(
                    "ADD_WAYPOINT",
                    {"index": idx, "lat": lat, "lon": lon, "alt": altitude}
                )
                time.sleep(0.3)  # 300ms delay per packet

            # End batch - send multiple times for reliability
            for _ in range(2):
                self.drone_states[DroneName.Sprayer.value].radio.send_command(
                    "END_WAYPOINTS",
                    {"expected_count": total}
                )
                time.sleep(0.3)

            log_message("GCSController",
                        f"Transferred {total} waypoints (alt={altitude}m) to Sprayer (incremental)")
            return True

        except Exception as e:
            log_message("GCSController", f"Failed to transfer waypoints: {e}")
            return False

    def get_waypoints(self) -> list:
        """Get current waypoints for sprayer drone."""
        if hasattr(self, 'sprayer_waypoints'):
            return self.sprayer_waypoints
        if hasattr(self, 'waypoints'):
            return self.waypoints
        return []

    def set_waypoints(self, waypoints: list) -> bool:
        """Set waypoints for the sprayer drone."""
        try:
            self.sprayer_waypoints = waypoints
            print(f"[GCS] Set {len(waypoints)} waypoints")
            return True
        except Exception as e:
            print(f"[GCS] Error setting waypoints: {e}")
            return False

    def clear_waypoints(self) -> bool:
        """Clear all waypoints."""
        try:
            self.sprayer_waypoints = []
            if hasattr(self, 'waypoints'):
                self.waypoints = []
            print("[GCS] Waypoints cleared")
            return True
        except Exception as e:
            print(f"[GCS] Error clearing waypoints: {e}")
            return False

    def export_waypoints_to_csv(self, filepath: str) -> bool:
        """Export waypoints to CSV file."""
        import csv
        try:
            waypoints = self.get_waypoints()
            if not waypoints:
                return False
            
            with open(filepath, 'w', newline='') as csvfile:
                fieldnames = ['index', 'latitude', 'longitude', 'altitude', 'type']
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                writer.writeheader()
                
                for i, wp in enumerate(waypoints):
                    writer.writerow({
                        'index': i + 1,
                        'latitude': wp.get('lat', 0),
                        'longitude': wp.get('lon', 0),
                        'altitude': wp.get('alt', 3.0),
                        'type': wp.get('type', 'waypoint')
                    })
            return True
        except Exception as e:
            print(f"[GCS] Export error: {e}")
            return False

    def import_waypoints_from_csv(self, filepath: str) -> bool:
        """Import waypoints from CSV file."""
        import csv
        try:
            waypoints = []
            with open(filepath, 'r', newline='') as csvfile:
                reader = csv.DictReader(csvfile)
                for row in reader:
                    waypoint = {
                        'lat': float(row.get('latitude', row.get('lat', 0))),
                        'lon': float(row.get('longitude', row.get('lon', 0))),
                        'alt': float(row.get('altitude', row.get('alt', 3.0))),
                        'type': row.get('type', 'waypoint')
                    }
                    waypoints.append(waypoint)
        
            self.sprayer_waypoints = waypoints
            return True
        except Exception as e:
            print(f"[GCS] Import error: {e}")
            return False

    # ---------------------------------------------------------
    # STOP
    # ---------------------------------------------------------
    def stop(self) -> None:
        self._running = False
        self.listener_thread.join()
        print("[GCSController] Stopped")
