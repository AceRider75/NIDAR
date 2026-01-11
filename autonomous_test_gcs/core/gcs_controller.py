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
                print("GCS GOT SPRAYER PACKET", pkt)
                try:
                    self._process_packet(DroneName.Sprayer, pkt)
                except Exception as e:
                    log_message("GCSController", f"Error processing sprayer packet: {e}")

            # --- Scanner ---
            pkt = self.scanner_state.radio.get_latest_packet()
            if pkt:
                print("GCS GOT SCANNER PACKET", pkt)
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

    def _process_yellow_spots(self, state: DroneState, spots: List[Dict[str, Any]]) -> None:
        """Process and store yellow spot detections with full details."""
        with state.yellow_spots_lock:
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
                        "timestamp": time.time()
                    }
                    
                    if spot_id not in state.yellow_spots:
                        state.yellow_spots[spot_id] = []
                        log_message("GCSController", f"New spot detected: ID={spot_id}")
                    
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

    def get_spot_centers(self, drone: DroneName) -> List[Tuple[float, float]]:
        """
        Get averaged center coordinates for each spot.
        Returns list of (lat, lon) tuples representing the average position of each spot.
        """
        with self.lock:
            state = self.drone_states[drone.value]
            with state.yellow_spots_lock:
                centers = []
                for spot_id, coords in state.yellow_spots.items():
                    if coords:
                        avg_lat = sum(c["lat"] for c in coords) / len(coords)
                        avg_lon = sum(c["lon"] for c in coords) / len(coords)
                        centers.append((avg_lat, avg_lon))
                return centers

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

    def transfer_waypoints_to_sprayer(self, scanner_drone: DroneName = DroneName.Scanner) -> bool:
        """
        Transfer detected yellow spot centers from scanner to sprayer drone incrementally.
        Sends START_WAYPOINTS, then ADD_WAYPOINT for each, and END_WAYPOINTS.
        """
        try:
            centers = self.get_spot_centers(scanner_drone)
            if not centers:
                log_message("GCSController", "No yellow spots detected to transfer")
                return False

            default_alt = 3.0
            total = len(centers)

            # Start batch
            self.drone_states[DroneName.Sprayer.value].radio.send_command(
                "START_WAYPOINTS",
                {"expected_count": total, "altitude": default_alt}
            )

            # Send one at a time
            for idx, (lat, lon) in enumerate(centers, start=1):
                self.drone_states[DroneName.Sprayer.value].radio.send_command(
                    "ADD_WAYPOINT",
                    {"index": idx, "lat": lat, "lon": lon, "alt": default_alt}
                )

            # End batch
            self.drone_states[DroneName.Sprayer.value].radio.send_command(
                "END_WAYPOINTS",
                {"expected_count": total}
            )

            log_message("GCSController",
                        f"Transferred {total} waypoints to Sprayer drone (incremental)")
            return True

        except Exception as e:
            log_message("GCSController", f"Failed to transfer waypoints: {e}")
            return False

    def transfer_waypoints_detailed(self, scanner_drone: DroneName = DroneName.Scanner, 
                                   altitude: float = 3.0, 
                                   use_weighted_centers: bool = True) -> bool:
        """
        Incremental transfer with detailed options.
        """
        try:
            with self.lock:
                state = self.drone_states[scanner_drone.value]
                with state.yellow_spots_lock:
                    if not state.yellow_spots:
                        log_message("GCSController", "No spots to transfer")
                        return False

                    # Build list first to know expected_count
                    points = []
                    for spot_id, coords in state.yellow_spots.items():
                        if not coords:
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
                log_message("GCSController", "No valid waypoints to transfer")
                return False

            total = len(points)
            self.drone_states[DroneName.Sprayer.value].radio.send_command(
                "START_WAYPOINTS",
                {"expected_count": total, "altitude": altitude}
            )

            for idx, (lat, lon) in enumerate(points, start=1):
                self.drone_states[DroneName.Sprayer.value].radio.send_command(
                    "ADD_WAYPOINT",
                    {"index": idx, "lat": lat, "lon": lon, "alt": altitude}
                )

            self.drone_states[DroneName.Sprayer.value].radio.send_command(
                "END_WAYPOINTS",
                {"expected_count": total}
            )

            log_message("GCSController",
                        f"Transferred {total} waypoints (alt={altitude}m) to Sprayer (incremental)")
            return True

        except Exception as e:
            log_message("GCSController", f"Failed to transfer waypoints: {e}")
            return False

    # ---------------------------------------------------------
    # STOP
    # ---------------------------------------------------------
    def stop(self) -> None:
        self._running = False
        self.listener_thread.join()
        print("[GCSController] Stopped")
