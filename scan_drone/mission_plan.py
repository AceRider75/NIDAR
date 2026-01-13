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
        """
        Generate a scanline (lawnmower) pattern coverage path.
        Optimizes for longest straight lines and improved handling of arbitrary shapes.
        """
        if not self.kml or not self.polygon:
            return []

        # Convert polygon to local meters
        poly_meters = [self._latlon_to_meters(lat, lon) for lat, lon in self.polygon]
        
        # 1. Find optimal rotation angle (minimize height -> minimize number of turns)
        best_angle = 0
        min_height = float('inf')
        
        # Check angles from 0 to 180 degrees
        for angle_deg in range(0, 180, 5): 
            angle_rad = math.radians(angle_deg)
            # Rotate points to check height (projection on Y axis)
            # y' = x sin + y cos
            ys = []
            for x, y in poly_meters:
                y_rot = x * math.sin(angle_rad) + y * math.cos(angle_rad)
                ys.append(y_rot)
            
            height = max(ys) - min(ys)
            if height < min_height:
                min_height = height
                best_angle = angle_rad

        print(f"Optimal scan angle: {math.degrees(best_angle):.1f} degrees")
        
        # 2. Rotate polygon to this angle
        rot_poly = []
        cos_a = math.cos(best_angle)
        sin_a = math.sin(best_angle)
        
        for x, y in poly_meters:
            rx = x * cos_a - y * sin_a
            ry = x * sin_a + y * cos_a
            rot_poly.append((rx, ry))
            
        # 3. Generate scanlines
        if not rot_poly:
            return []

        min_y = min(p[1] for p in rot_poly)
        max_y = max(p[1] for p in rot_poly)
        
        # Center the grid
        height = max_y - min_y
        if spacing_meters <= 0:
            spacing_meters = 5.0
            
        num_lines = int(height / spacing_meters) + 1  # Ensure coverage
        y_start = min_y + (height - ((num_lines - 1) * spacing_meters)) / 2
        
        waypoints_local = []
        direction = 1 # 1 for Left->Right, -1 for Right->Left
        
        current_y = y_start
        # Iterate slightly past max_y to ensure boundary coverage if needed, 
        # but centered logic covers it.
        
        for i in range(num_lines):
            line_y = y_start + i * spacing_meters
            if line_y > max_y + 0.1: # float tolerance
                break
                
            # Find intersections
            intersections = []
            n_points = len(rot_poly)
            for j in range(n_points):
                p1 = rot_poly[j]
                p2 = rot_poly[(j + 1) % n_points]
                
                # Check for intersection with line y = line_y
                # Handle vertices exactly on line by using inequality > vs <=
                # Standard ray casting rule: include start, exclude end
                if (p1[1] <= line_y < p2[1]) or (p2[1] <= line_y < p1[1]):
                    # x = x1 + (y - y1) * (x2 - x1) / (y2 - y1)
                    if p2[1] != p1[1]:
                        x = p1[0] + (line_y - p1[1]) * (p2[0] - p1[0]) / (p2[1] - p1[1])
                        intersections.append(x)
            
            intersections.sort()
            
            # Form segments from pairs of intersections
            segments = []
            for k in range(0, len(intersections), 2):
                if k+1 < len(intersections):
                    segments.append((intersections[k], intersections[k+1]))
            
            # Add segments to path
            if direction == -1:
                segments.reverse() # Process right-most segment first
                
            for seg in segments:
                x_start, x_end = seg
                if direction == 1:
                    waypoints_local.append((x_start, line_y))
                    waypoints_local.append((x_end, line_y))
                else:
                    waypoints_local.append((x_end, line_y))
                    waypoints_local.append((x_start, line_y))
            
            direction *= -1
            
        # 4. Rotate waypoints back and convert to Lat/Lon
        final_waypoints = []
        for rx, ry in waypoints_local:
            # Inverse rotation
            x = rx * cos_a + ry * sin_a
            y = -rx * sin_a + ry * cos_a
            lat, lon = self._meters_to_latlon(x, y)
            final_waypoints.append((lat, lon))
            
        # 5. Optimize start point
        if start_lat is not None and start_lon is not None and final_waypoints:
             d_start = haversine_dist(start_lat, start_lon, final_waypoints[0][0], final_waypoints[0][1])
             d_end = haversine_dist(start_lat, start_lon, final_waypoints[-1][0], final_waypoints[-1][1])
             if d_end < d_start:
                 final_waypoints.reverse()
                 print("Reversed path to start closer to drone location")
        
        print(f"Generated {len(final_waypoints)} waypoints (Scanline algorithm)")
        return final_waypoints

    def _latlon_to_meters(self, lat: float, lon: float) -> Tuple[float, float]:
        """Convert lat/lon to local meters (x, y) relative to centroid"""
        # x = longitude diff (at center lat), y = latitude diff
        meters_per_lon = 40075000.0 * math.cos(math.radians(self.center_lat)) / 360.0
        meters_per_lat = 111320.0
        
        x = (lon - self.center_lon) * meters_per_lon
        y = (lat - self.center_lat) * meters_per_lat
        return x, y

    def _meters_to_latlon(self, x: float, y: float) -> Tuple[float, float]:
        """Convert local meters (x, y) back to lat/lon"""
        meters_per_lon = 40075000.0 * math.cos(math.radians(self.center_lat)) / 360.0
        meters_per_lat = 111320.0
        
        if meters_per_lon == 0: meters_per_lon = 1.0 # Safety
        
        lon = x / meters_per_lon + self.center_lon
        lat = y / meters_per_lat + self.center_lat
        return lat, lon

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
    