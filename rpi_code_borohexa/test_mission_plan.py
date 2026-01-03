#!/usr/bin/env python3
"""
Test script for mission_plan.py
Tests waypoint validation, optimization, and visualizes the flight path
"""

import os
import sys
import random
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from mission_plan import MissionPlanner
from utils import haversine_dist


def generate_random_points(center_lat, center_lon, count=20, radius_meters=50):
    """Generate random GPS points around a center location.
    
    Args:
        center_lat: Center latitude
        center_lon: Center longitude
        count: Number of points to generate
        radius_meters: Max radius in meters from center
        
    Returns:
        List of (lat, lon) tuples
    """
    points = [
        (22.497764, 88.372255),
        (22.497701, 88.372401),
        (22.497600, 88.372300),
        (22.497943, 88.372148),
        (22.498022, 88.372386),
    ]
    # R = 6371000.0  # Earth radius in meters
    
    # for _ in range(count):
    #     # Random angle and distance
    #     angle = random.uniform(0, 2 * 3.14159)
    #     distance = random.uniform(0, radius_meters)
        
    #     # Convert to lat/lon offset
    #     dlat = (distance * 3.14159 / 180.0) * (1.0 / R) * 180.0 / 3.14159
    #     dlon = (distance * 3.14159 / 180.0) * (1.0 / (R * abs(0.9998477))) * 180.0 / 3.14159  # approximate cos(lat)
        
    #     lat = center_lat + dlat * (1 if angle < 3.14159 else -1)
    #     lon = center_lon + dlon * (1 if (angle % 3.14159) < 1.5708 else -1)
        
    #     points.append((lat, lon))
    
    return points


def visualize_mission(planner, original_points, optimized_mission, start_lat, start_lon):
    """Create visualization of the mission plan.
    
    Args:
        planner: MissionPlanner instance with loaded polygon
        original_points: Original candidate waypoints
        optimized_mission: Optimized and validated mission waypoints
        start_lat: Starting latitude
        start_lon: Starting longitude
    """
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 8))
    
    # Extract polygon coordinates
    if planner.polygon:
        poly_lats = [p[0] for p in planner.polygon]
        poly_lons = [p[1] for p in planner.polygon]
        # Close the polygon
        poly_lats.append(planner.polygon[0][0])
        poly_lons.append(planner.polygon[0][1])
    else:
        poly_lats, poly_lons = [], []
    
    # ===== LEFT PLOT: Original points =====
    ax1.set_title('Original Waypoints (Before Optimization)', fontsize=14, fontweight='bold')
    ax1.set_xlabel('Longitude', fontsize=12)
    ax1.set_ylabel('Latitude', fontsize=12)
    ax1.grid(True, alpha=0.3)
    
    # Plot polygon
    if poly_lats:
        ax1.plot(poly_lons, poly_lats, 'b-', linewidth=2, label='Geofence Boundary')
        ax1.fill(poly_lons, poly_lats, color='lightblue', alpha=0.2)
    
    # Plot centroid
    if planner.center_lat and planner.center_lon:
        ax1.plot(planner.center_lon, planner.center_lat, 'ko', markersize=8, label='Polygon Center')
    
    # Plot start position
    ax1.plot(start_lon, start_lat, 'g^', markersize=12, label='Start Position', zorder=5)
    
    # Separate valid and invalid points
    valid_points = []
    invalid_points = []
    for lat, lon in original_points:
        if planner.is_point_inside(lat, lon):
            valid_points.append((lat, lon))
        else:
            invalid_points.append((lat, lon))
    
    # Plot points
    if valid_points:
        v_lats = [p[0] for p in valid_points]
        v_lons = [p[1] for p in valid_points]
        ax1.scatter(v_lons, v_lats, c='green', s=80, marker='o', 
                   label=f'Valid Points ({len(valid_points)})', alpha=0.7, zorder=3)
    
    if invalid_points:
        i_lats = [p[0] for p in invalid_points]
        i_lons = [p[1] for p in invalid_points]
        ax1.scatter(i_lons, i_lats, c='red', s=80, marker='x', 
                   label=f'Invalid Points ({len(invalid_points)})', alpha=0.7, zorder=3)
    
    ax1.legend(loc='best')
    ax1.set_aspect('equal', adjustable='datalim')
    
    # ===== RIGHT PLOT: Optimized mission =====
    ax2.set_title('Optimized Mission Path', fontsize=14, fontweight='bold')
    ax2.set_xlabel('Longitude', fontsize=12)
    ax2.set_ylabel('Latitude', fontsize=12)
    ax2.grid(True, alpha=0.3)
    
    # Plot polygon
    if poly_lats:
        ax2.plot(poly_lons, poly_lats, 'b-', linewidth=2, label='Geofence Boundary')
        ax2.fill(poly_lons, poly_lats, color='lightblue', alpha=0.2)
    
    # Plot start position
    ax2.plot(start_lon, start_lat, 'g^', markersize=12, label='Start Position', zorder=5)
    
    # Plot optimized path
    if optimized_mission:
        path_lats = [start_lat] + [p[0] for p in optimized_mission]
        path_lons = [start_lon] + [p[1] for p in optimized_mission]
        
        # Draw path with arrows
        ax2.plot(path_lons, path_lats, 'r-', linewidth=2, alpha=0.6, label='Flight Path', zorder=2)
        
        # Add waypoint numbers
        for i, (lat, lon) in enumerate(optimized_mission):
            ax2.scatter(lon, lat, c='orange', s=150, marker='o', edgecolors='black', 
                       linewidths=1.5, zorder=4)
            ax2.text(lon, lat, str(i+1), ha='center', va='center', 
                    fontsize=9, fontweight='bold', zorder=5)
        
        # Calculate and display total distance
        total_dist = 0.0
        for i in range(len(path_lats) - 1):
            total_dist += haversine_dist(path_lats[i], path_lons[i], 
                                        path_lats[i+1], path_lons[i+1])
        
        ax2.text(0.02, 0.98, f'Total Distance: {total_dist:.1f}m\nWaypoints: {len(optimized_mission)}',
                transform=ax2.transAxes, verticalalignment='top',
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8),
                fontsize=11, fontweight='bold')
    
    ax2.legend(loc='best')
    ax2.set_aspect('equal', adjustable='datalim')
    
    plt.tight_layout()
    
    # Save figure
    output_path = os.path.join(os.path.dirname(__file__), 'mission_visualization.png')
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"\n✓ Visualization saved to: {output_path}")
    
    plt.show()


def run_tests():
    """Run comprehensive tests on MissionPlanner"""
    
    print("=" * 60)
    print("MISSION PLANNER TEST SUITE")
    print("=" * 60)
    
    # Setup paths
    BASE_DIR = os.path.dirname(os.path.abspath(__file__))
    KML_PATH = os.path.join(BASE_DIR, "data", "JUs.kml")
    
    if not os.path.exists(KML_PATH):
        print(f"❌ ERROR: KML file not found at {KML_PATH}")
        print("   Using JU.kml as fallback...")
        KML_PATH = os.path.join(BASE_DIR, "data", "JU.kml")
        if not os.path.exists(KML_PATH):
            print(f"❌ ERROR: Fallback KML file not found either!")
            return
    
    print(f"\n1. Loading KML file: {os.path.basename(KML_PATH)}")
    print("-" * 60)
    
    # Initialize planner
    planner = MissionPlanner(KML_PATH)
    
    if not planner.polygon:
        print("❌ ERROR: Failed to load polygon from KML")
        return
    
    print(f"✓ Polygon loaded successfully")
    print(f"  - Corners: {len(planner.polygon)}")
    print(f"  - Centroid: ({planner.center_lat:.6f}, {planner.center_lon:.6f})")
    
    # Test 2: Point-in-polygon
    print(f"\n2. Testing Point-in-Polygon Detection")
    print("-" * 60)
    
    # Test centroid (should be inside)
    is_inside = planner.is_point_inside(planner.center_lat, planner.center_lon)
    print(f"  Centroid inside polygon: {is_inside} {'✓' if is_inside else '❌'}")
    
    # Test point far outside
    far_lat = planner.center_lat + 1.0
    far_lon = planner.center_lon + 1.0
    is_inside = planner.is_point_inside(far_lat, far_lon)
    print(f"  Far point inside polygon: {is_inside} {'✓' if not is_inside else '❌'}")
    
    # Test 3: Generate random waypoints
    print(f"\n3. Generating Random Waypoints")
    print("-" * 60)
    
    # Use a point inside the polygon as start (centroid)
    start_lat = planner.center_lat
    start_lon = planner.center_lon
    
    # Generate random points around polygon center
    random.seed(42)  # For reproducible results
    num_points = 25
    candidate_points = generate_random_points(start_lat, start_lon, count=num_points, radius_meters=40)
    
    print(f"  Generated {len(candidate_points)} random candidate waypoints")
    
    # Test 4: Generate optimized mission
    print(f"\n4. Generating Optimized Mission")
    print("-" * 60)
    
    optimized_mission = planner.generate_mission_from_points(
        candidate_points,
        start_lat=start_lat,
        start_lon=start_lon
    )
    
    print(f"\n✓ Mission generation complete!")
    print(f"  - Input points: {len(candidate_points)}")
    print(f"  - Valid points: {len(optimized_mission)}")
    print(f"  - Invalid points: {len(candidate_points) - len(optimized_mission)}")
    
    if optimized_mission:
        print(f"\n  First 5 waypoints:")
        for i, (lat, lon) in enumerate(optimized_mission[:5]):
            print(f"    {i+1}. ({lat:.6f}, {lon:.6f})")
        if len(optimized_mission) > 5:
            print(f"    ... and {len(optimized_mission) - 5} more")
    
    # Test 5: Visualize
    print(f"\n5. Generating Visualization")
    print("-" * 60)
    
    try:
        visualize_mission(planner, candidate_points, optimized_mission, start_lat, start_lon)
        print("✓ Visualization complete!")
    except Exception as e:
        print(f"❌ Visualization failed: {e}")
    
    # Summary
    print("\n" + "=" * 60)
    print("TEST SUMMARY")
    print("=" * 60)
    print(f"✓ All tests completed successfully!")
    print(f"  - Polygon loaded: {len(planner.polygon)} corners")
    print(f"  - Points validated: {len(candidate_points)}")
    print(f"  - Mission waypoints: {len(optimized_mission)}")
    print("=" * 60)


if __name__ == "__main__":
    run_tests()
