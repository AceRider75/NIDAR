from dataclasses import dataclass, field
from utils import haversine_dist
from typing import Tuple, List, Optional
from mission_plan import MissionPlanner
from lxml import etree

@dataclass
class GeoFence:
    """Geofencing configuration with dual mode support"""
    mode: str = "polygon"  # "polygon" or "radius"
    
    # Radius mode parameters
    home_lat: float = 0.0
    home_lon: float = 0.0
    max_radius: float = 500.0  # meters
    
    # Polygon mode parameters
    polygon_points: List[Tuple[float, float]] = field(default_factory=list)
    mission_planner: Optional[MissionPlanner] = None
    
    # Common parameters
    min_altitude: float = 0.5
    max_altitude: float = 50.0
    enabled: bool = True
    
    def is_within_bounds(self, lat: float, lon: float, alt: float) -> Tuple[bool, str]:
        """Check if position is within geofence"""
        if not self.enabled:
            return True, "Geofence disabled"
        
        # Check altitude first (common to both modes)
        if alt < self.min_altitude:
            return False, f"Altitude too low: {alt:.2f}m < {self.min_altitude}m"
        if alt > self.max_altitude:
            return False, f"Altitude too high: {alt:.2f}m > {self.max_altitude}m"
        
        # Mode-specific horizontal checks
        if self.mode == "polygon":
            return self._check_polygon_bounds(lat, lon)
        elif self.mode == "radius":
            return self._check_radius_bounds(lat, lon)
        else:
            return False, f"Unknown geofence mode: {self.mode}"
    
    def _check_polygon_bounds(self, lat: float, lon: float) -> Tuple[bool, str]:
        """Check if point is within polygon boundary"""
        if self.mission_planner is None:
            return False, "Polygon geofence not initialized"
        
        if not self.mission_planner.polygon:
            return False, "No polygon loaded in mission planner"
        
        # Use mission planner's point-in-polygon check
        if self.mission_planner.is_point_inside(lat, lon):
            return True, "Within polygon bounds"
        else:
            return False, f"Outside polygon boundary: ({lat:.6f}, {lon:.6f})"
    
    def _check_radius_bounds(self, lat: float, lon: float) -> Tuple[bool, str]:
        """Check if point is within radius from home"""
        dist = haversine_dist(self.home_lat, self.home_lon, lat, lon)
        if dist > self.max_radius:
            return False, f"Outside geofence: {dist:.2f}m > {self.max_radius}m"
        return True, f"Within radius: {dist:.2f}m"


    def get_polygon_corners(kml_path, polygon_name="Field"):        #Extract Polygon Corners from KML File
        tree = etree.parse(kml_path)

        ns = {
            "kml": "http://www.opengis.net/kml/2.2"
        }

        placemarks = tree.xpath(
            f"//kml:Placemark[kml:name='{polygon_name}']",
            namespaces=ns
        )

        if not placemarks:
            # Fallback: try to find ANY placemark
            all_placemarks = tree.xpath("//kml:Placemark", namespaces=ns)
            if len(all_placemarks) == 1:
                print(f"Warning: Placemark '{polygon_name}' not found. Using the only available placemark.")
                placemarks = all_placemarks
            elif len(all_placemarks) > 1:
                names = [p.find("kml:name", namespaces=ns).text for p in all_placemarks]
                raise ValueError(f"Placemark '{polygon_name}' not found. Available placemarks: {names}")
            else:
                raise ValueError(f"Placemark '{polygon_name}' not found and no other placemarks in file.")

        coords_text = placemarks[0].xpath(
            ".//kml:Polygon//kml:coordinates/text()",
            namespaces=ns
        )[0]

        corners = []
        for c in coords_text.strip().split():
            lon, lat, *_ = map(float, c.split(","))
            corners.append((lat, lon))   # (lat, lon)

        return corners
