# geofence.py
from lxml import etree

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
