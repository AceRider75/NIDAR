# mission_plan.py

from geofence import get_polygon_corners
from typing import List, Tuple, Optional
import math
import utils


class MissionPlanner:
    def __init__(self, kml: str = None, polygon_name: str = "Field"):
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

    def set_targets(
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
                utils.haversine_dist(lat, lon, self.center_lat, self.center_lon)
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
            dist = utils.haversine_dist(start_lat, start_lon, lat, lon)
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
            dist_to_prev = utils.haversine_dist(
                curr_corner[0], curr_corner[1], prev_corner[0], prev_corner[1]
            )
            dist_to_next = utils.haversine_dist(
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
            chord_dist = utils.haversine_dist(
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
            dist_to_center = utils.haversine_dist(lat, lon, self.center_lat, self.center_lon)
            
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
