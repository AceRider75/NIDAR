# mission_plan.py

from typing import List, Tuple, Optional
import math
from lxml import etree
from utils import haversine_dist 


class MissionPlanner:
    def __init__(self, kml: str = None, polygon_name: str = "Field"):
        self.kml = kml
        self.polygon_name = polygon_name
        self.polygon = []
        self.center_lat = 0.0
        self.center_lon = 0.0

        if kml:
            # [(lat, lon), ...] last point may be duplicate
            self.polygon = self.get_polygon_corners(kml, polygon_name)
            if len(self.polygon) > 0 and self.polygon[0] == self.polygon[-1]:
                self.polygon = self.polygon[:-1]

            # Calculate centroid
            self.center_lat, self.center_lon = self._centroid(self.polygon)
            print(f"Polygon centroid: ({self.center_lat:.6f}, {self.center_lon:.6f})")
            print(f"Polygon has {len(self.polygon)} corners")
    
    def get_polygon_corners(self, kml_path, polygon_name="Field"):
        """Extract Polygon Corners from KML File"""
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

    def generate_mission_from_points(
        self,
        spacing_meters: float = 5.0,
        corner_points: int = 0,
        min_loop_size: float = 3.0,
        start_lat: Optional[float] = None,
        start_lon: Optional[float] = None,
    ) -> List[Tuple[float, float]]:
        
        if not self.kml or not self.polygon:
            return []

        # Reorder polygon to start from corner closest to drone's starting position
        working_polygon = self._reorder_polygon_from_start(
            self.polygon, start_lat, start_lon
        )

        waypoints = []
        current_polygon = list(working_polygon)  # Start with outer boundary
        loop_count = 0
        
        while True:
            # Check if polygon is still valid (not collapsed to center)
            # Check if polygon is still valid (not collapsed to center)
            # Use max_dist to ensure we cover the outer edges of complex polygons
            max_dist = max(
                haversine_dist(lat, lon, self.center_lat, self.center_lon)
                for lat, lon in current_polygon
            )
            
            if max_dist < min_loop_size:
                print(f"Stopping at loop {loop_count}: polygon fully collapsed (max dist: {max_dist:.2f}m)")
                break
            
            # Generate smooth path for this loop
            loop_waypoints = self._generate_smooth_loop(current_polygon, corner_points)
            waypoints.extend(loop_waypoints)
            
            loop_count += 1
            
            # Shrink polygon for next loop
            current_polygon = self._shrink_polygon(current_polygon, spacing_meters)
            
            # Safety limit
            if loop_count > 100:
                print("Safety limit reached (100 loops)")
                break
        
        # Add final center point
        waypoints.append((self.center_lat, self.center_lon))
        
        print(f"Generated {len(waypoints)} waypoints across {loop_count} loops")
        print(f"Spacing: {spacing_meters}m, Corner smoothing: {corner_points} points per corner")
        
        return waypoints

    def _reorder_polygon_from_start(
        self,
        polygon: List[Tuple[float, float]],
        start_lat: Optional[float],
        start_lon: Optional[float],
    ) -> List[Tuple[float, float]]:
        
        if not polygon or start_lat is None or start_lon is None:
            return polygon
        
        # Find the corner closest to the start location
        min_dist = float('inf')
        closest_idx = 0
        
        for i, (lat, lon) in enumerate(polygon):
            dist = haversine_dist(start_lat, start_lon, lat, lon)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
        
        # Reorder polygon to start from closest corner
        reordered = polygon[closest_idx:] + polygon[:closest_idx]
        
        print(f"Reordered polygon to start from corner {closest_idx} (closest to takeoff, {min_dist:.1f}m away)")
        
        return reordered

    def _generate_smooth_loop(
        self, polygon: List[Tuple[float, float]], corner_points: int
    ) -> List[Tuple[float, float]]:
        
        if len(polygon) < 3:
            return list(polygon)
        
        waypoints = []
        n = len(polygon)
        
        for i in range(n):
            # Get previous, current, and next corners
            prev_corner = polygon[(i - 1) % n]
            curr_corner = polygon[i]
            next_corner = polygon[(i + 1) % n]
            
            # Calculate distances to determine corner radius
            dist_to_prev = haversine_dist(
                curr_corner[0], curr_corner[1], prev_corner[0], prev_corner[1]
            )
            dist_to_next = haversine_dist(
                curr_corner[0], curr_corner[1], next_corner[0], next_corner[1]
            )
            
            # Corner radius is fraction of shortest adjacent edge
            # This ensures the arc doesn't extend past the midpoint of edges
            corner_radius_ratio = 0.25 # Use 25% of shortest edge
            effective_radius = min(dist_to_prev, dist_to_next) * corner_radius_ratio
            
            # Calculate control points for Bezier curve
            # Point where arc starts (on edge from prev to curr)
            t_start = 1.0 - (effective_radius / dist_to_prev) if dist_to_prev > 0 else 1.0
            t_start = max(0.5, min(1.0, t_start))  # Clamp to [0.5, 1.0]
            
            arc_start = (
                prev_corner[0] + t_start * (curr_corner[0] - prev_corner[0]),
                prev_corner[1] + t_start * (curr_corner[1] - prev_corner[1])
            )
            
            # Point where arc ends (on edge from curr to next)
            t_end = effective_radius / dist_to_next if dist_to_next > 0 else 0.0
            t_end = min(0.5, max(0.0, t_end))  # Clamp to [0.0, 0.5]
            
            arc_end = (
                curr_corner[0] + t_end * (next_corner[0] - curr_corner[0]),
                curr_corner[1] + t_end * (next_corner[1] - curr_corner[1])
            )
            
            # Add the straight segment point (arc start)
            waypoints.append(arc_start)
            
            # Adaptive Smoothing: Calculate number of points based on arc length
            # Estimate arc length using chord length (distance between start and end)
            chord_dist = haversine_dist(
                arc_start[0], arc_start[1], arc_end[0], arc_end[1]
            )
            
            # Minimum spacing between points in the turn (meters)
            # 2.0m is safe for 8m/s flight (0.25s between points)
            min_turn_spacing = 2.0
            
            # Calculate number of intermediate points
            # We want at least 1 point if possible to make it a curve, but not if it's tiny
            if chord_dist < min_turn_spacing:
                num_points = 0
            else:
                num_points = int(chord_dist / min_turn_spacing)
            
            # Generate Bezier curve points
            # Using quadratic Bezier: B(t) = (1-t)²P0 + 2(1-t)tP1 + t²P2
            # P0 = arc_start, P1 = curr_corner (control point), P2 = arc_end
            
            # We need num_points intermediate points
            # Steps will be num_points + 1 segments
            steps = num_points + 1
            
            for j in range(1, steps):
                t = j / steps
                t_inv = 1.0 - t
                
                # Quadratic Bezier interpolation
                lat = (t_inv * t_inv * arc_start[0] + 
                       2 * t_inv * t * curr_corner[0] + 
                       t * t * arc_end[0])
                lon = (t_inv * t_inv * arc_start[1] + 
                       2 * t_inv * t * curr_corner[1] + 
                       t * t * arc_end[1])
                
                waypoints.append((lat, lon))
            
            # Add arc end point
            waypoints.append(arc_end)
        
        return waypoints

    def _shrink_polygon(
        self, polygon: List[Tuple[float, float]], step_meters: float
    ) -> List[Tuple[float, float]]:
        
        if not polygon:
            return []

        new_polygon = []
        
        for lat, lon in polygon:
            # Distance from vertex to center
            dist_to_center = haversine_dist(lat, lon, self.center_lat, self.center_lon)
            
            if dist_to_center <= step_meters:
                # Vertex would pass center, place at center
                new_polygon.append((self.center_lat, self.center_lon))
                continue
            
            # Vector from vertex toward center
            vec_lat = self.center_lat - lat
            vec_lon = self.center_lon - lon
            
            # Move ratio: step_meters / dist_to_center
            move_ratio = step_meters / dist_to_center
            
            new_lat = lat + (vec_lat * move_ratio)
            new_lon = lon + (vec_lon * move_ratio)
            
            new_polygon.append((new_lat, new_lon))
        
        return new_polygon

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

        if inside:
            return True

        # Check tolerance if outside
        tolerance = 10.0  # meters
        dist = self._get_distance_to_polygon(lat, lon)
        if dist <= tolerance:
            print(f"Point outside polygon but within tolerance: {dist:.2f}m <= {tolerance}m")
            return True

        return False

    def _get_distance_to_polygon(self, lat: float, lon: float) -> float:
        """Calculate minimum distance from point to polygon boundary in meters"""
        min_dist = float('inf')
        poly = self.polygon
        if not poly:
            return float('inf')

        # Local approximation constants
        meters_per_lat = 111320.0
        meters_per_lon = 40075000.0 * math.cos(math.radians(lat)) / 360.0

        for i in range(len(poly)):
            p1 = poly[i]
            p2 = poly[(i + 1) % len(poly)]

            # Convert to local meters relative to point (lat, lon)
            x1 = (p1[1] - lon) * meters_per_lon
            y1 = (p1[0] - lat) * meters_per_lat
            x2 = (p2[1] - lon) * meters_per_lon
            y2 = (p2[0] - lat) * meters_per_lat

            # Point is at (0,0)
            # Distance from (0,0) to segment (x1,y1)-(x2,y2)

            # Squared length of segment
            l2 = (x2 - x1)**2 + (y2 - y1)**2

            if l2 == 0:
                dist = math.sqrt(x1**2 + y1**2)
            else:
                # Project point onto line, clamped to segment
                # t = dot(p - p1, p2 - p1) / l2
                # p is (0,0), so p - p1 is (-x1, -y1)
                # p2 - p1 is (x2-x1, y2-y1)
                t = ((-x1) * (x2 - x1) + (-y1) * (y2 - y1)) / l2
                t = max(0, min(1, t))

                proj_x = x1 + t * (x2 - x1)
                proj_y = y1 + t * (y2 - y1)

                dist = math.sqrt(proj_x**2 + proj_y**2)

            if dist < min_dist:
                min_dist = dist

        return min_dist
    