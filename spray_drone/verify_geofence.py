
import matplotlib.pyplot as plt
from lxml import etree
import os

def get_polygon_corners(kml_path, polygon_name="Field"):
    tree = etree.parse(kml_path)
    ns = {"kml": "http://www.opengis.net/kml/2.2"}
    placemarks = tree.xpath(f"//kml:Placemark[kml:name='{polygon_name}']", namespaces=ns)
    
    if not placemarks:
        all_placemarks = tree.xpath("//kml:Placemark", namespaces=ns)
        if len(all_placemarks) == 1:
            placemarks = all_placemarks
        else:
            raise ValueError(f"Placemark '{polygon_name}' not found.")

    coords_text = placemarks[0].xpath(".//kml:Polygon//kml:coordinates/text()", namespaces=ns)[0]
    corners = []
    for c in coords_text.strip().split():
        lon, lat, *_ = map(float, c.split(","))
        corners.append((lat, lon))
    return corners

def is_point_inside(polygon, lat, lon):
    inside = False
    n = len(polygon)
    j = n - 1
    for i in range(n):
        yi, xi = polygon[i]
        yj, xj = polygon[j]
        intersect = ((xi > lon) != (xj > lon)) and (
            lat < (yj - yi) * (lon - xi) / (xj - xi + 1e-16) + yi
        )
        if intersect:
            inside = not inside
        j = i
    return inside

# Configuration
kml_path = "data/JUs.kml"
point = (22.498041, 88.372439)

# Get polygon
try:
    polygon = get_polygon_corners(kml_path)
    print(f"Polygon corners: {len(polygon)}")
except Exception as e:
    print(f"Error reading KML: {e}")
    exit(1)

# Check point
is_inside = is_point_inside(polygon, point[0], point[1])
print(f"Point {point} is inside: {is_inside}")

# Plot
poly_lats = [p[0] for p in polygon]
poly_lons = [p[1] for p in polygon]

# Close the polygon for plotting if not already closed
if polygon[0] != polygon[-1]:
    poly_lats.append(polygon[0][0])
    poly_lons.append(polygon[0][1])

plt.figure(figsize=(10, 8))
plt.plot(poly_lons, poly_lats, 'b-', label='Geofence')
plt.plot(point[1], point[0], 'ro', label='Drone Position')

# Add some padding
margin = 0.0001
plt.xlim(min(poly_lons) - margin, max(poly_lons) + margin)
plt.ylim(min(poly_lats) - margin, max(poly_lats) + margin)

plt.title(f"Geofence Verification\nPoint Inside: {is_inside}")
plt.xlabel("Longitude")
plt.ylabel("Latitude")
plt.legend()
plt.grid(True)
plt.show()
