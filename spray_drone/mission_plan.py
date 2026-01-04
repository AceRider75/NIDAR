# mission_plan.py

from geofence import get_polygon_corners
from typing import List, Tuple, Optional
import math
from utils import haversine_dist as _haversine_dist


class MissionPlanner:
    def __init__(self, kml: str = "/Users/trishit_debsharma/Documents/Code/NIDAR/boro_hexa/rpi_code_boro_hexa/data/JUs.kml", polygon_name: str = "Field"):
        self.kml = kml
        self.polygon_name = polygon_name
        self.polygon = []
        self.center_lat = 0.0
        self.center_lon = 0.0

        if kml:
            # [(lat, lon), ...] last point may be duplicate
            self.polygon = get_polygon_corners(kml, polygon_name)
            if len(self.polygon) > 0 and self.polygon[0] == self.polygon[-1]:
                self.polygon = self.polygon[:-1]

            # Calculate centroid
            self.center_lat, self.center_lon = self._centroid(self.polygon)
            print(f"Polygon centroid: ({self.center_lat:.6f}, {self.center_lon:.6f})")
            print(f"Polygon has {len(self.polygon)} corners")

    def _centroid(self, vertices: List[Tuple[float, float]]) -> Tuple[float, float]:
        
        if not vertices:
            return 0.0, 0.0
        lat = sum(p[0] for p in vertices) / len(vertices)
        lon = sum(p[1] for p in vertices) / len(vertices)
        return lat, lon

    def is_point_inside(self, lat: float, lon: float) -> bool:
        """Ray-casting point-in-polygon test. Assumes `self.polygon` is list of (lat,lon).

        Uses the standard even-odd rule on projected lat/lon coordinates. Good
        for reasonably small polygons where lat/lon distortion is negligible.
        """
        if not self.polygon:
            return False

        inside = False
        n = len(self.polygon)
        j = n - 1
        for i in range(n):
            yi, xi = self.polygon[i]   # lat, lon
            yj, xj = self.polygon[j]
            intersect = ((xi > lon) != (xj > lon)) and (
                lat < (yj - yi) * (lon - xi) / (xj - xi + 1e-16) + yi
            )
            if intersect:
                inside = not inside
            j = i

        return inside

    # def _haversine_dist(self, lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    #     """Calculate distance between two GPS coordinates in meters."""
    #     R = 6371000.0  # Earth radius in meters
    #     phi1 = math.radians(lat1)
    #     phi2 = math.radians(lat2)
    #     dphi = math.radians(lat2 - lat1)
    #     dlambda = math.radians(lon2 - lon1)
    #     a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlambda/2)**2
    #     c = 2*math.atan2(math.sqrt(a), math.sqrt(1-a))
    #     return R * c

    def _optimize_waypoint_order(
        self, 
        waypoints: List[Tuple[float, float]], 
        start_lat: float, 
        start_lon: float
    ) -> List[Tuple[float, float]]:
        """Reorder waypoints using greedy nearest-neighbor algorithm to minimize flight distance.
        
        Args:
            waypoints: List of (lat, lon) tuples to reorder
            start_lat: Starting latitude (current drone position)
            start_lon: Starting longitude (current drone position)
            
        Returns:
            Reordered list of waypoints starting with the nearest point
        """
        if not waypoints:
            return []
        
        if len(waypoints) == 1:
            return waypoints
        
        # Greedy nearest-neighbor: always visit the nearest unvisited point
        unvisited = list(waypoints)
        ordered = []
        current_lat, current_lon = start_lat, start_lon
        
        while unvisited:
            # Find nearest waypoint to current position
            nearest_idx = 0
            min_dist = float('inf')
            
            for i, (lat, lon) in enumerate(unvisited):
                dist = _haversine_dist(current_lat, current_lon, lat, lon)
                if dist < min_dist:
                    min_dist = dist
                    nearest_idx = i
            
            # Move nearest waypoint to ordered list
            nearest = unvisited.pop(nearest_idx)
            ordered.append(nearest)
            
            # Update current position
            current_lat, current_lon = nearest[0], nearest[1]
        
        total_distance = 0.0
        prev_lat, prev_lon = start_lat, start_lon
        for lat, lon in ordered:
            total_distance += _haversine_dist(prev_lat, prev_lon, lat, lon)
            prev_lat, prev_lon = lat, lon
        
        print(f"Optimized waypoint order - Total flight distance: {total_distance:.1f}m")
        
        return ordered

    def generate_mission_from_points(
        self, 
        points: List[Tuple[float, float]], 
        start_lat: Optional[float] = None, 
        start_lon: Optional[float] = None
    ) -> List[Tuple[float, float]]:
        """Given an array of (lat, lon) points, keep only those that lie
        inside the planner's KML polygon and return them as an optimized mission list.

        The waypoints are reordered using nearest-neighbor algorithm to minimize
        flight distance and time. If start_lat/start_lon are provided, optimization
        begins from that position; otherwise uses polygon centroid.

        Args:
            points: List of (lat, lon) tuples representing candidate waypoints
            start_lat: Optional starting latitude for route optimization
            start_lon: Optional starting longitude for route optimization

        Returns:
            Optimized list of validated waypoints, or empty list if no polygon loaded
        """
        if not self.polygon:
            print("No polygon loaded; cannot validate points")
            return []

        # Step 1: Filter points inside polygon
        valid = []
        for lat, lon in points:
            if self.is_point_inside(lat, lon):
                valid.append((lat, lon))
            else:
                print(f"Point outside polygon, skipping: ({lat:.6f}, {lon:.6f})")

        print(f"Validated {len(valid)} / {len(points)} input points inside polygon")
        
        if not valid:
            return []
        
        # Step 2: Optimize waypoint order to minimize distance
        if start_lat is None or start_lon is None:
            # Use polygon centroid as starting point if not provided
            start_lat = self.center_lat
            start_lon = self.center_lon
            print(f"Using polygon centroid as start: ({start_lat:.6f}, {start_lon:.6f})")
        else:
            print(f"Optimizing route from start position: ({start_lat:.6f}, {start_lon:.6f})")
        
        optimized = self._optimize_waypoint_order(valid, start_lat, start_lon)
        
        return optimized
    


