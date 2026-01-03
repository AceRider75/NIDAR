import math

def haversine_dist(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    #returns distance in meters
    R = 6371000.0
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlambda/2)**2
    c = 2*math.atan2(math.sqrt(a), math.sqrt(1-a))
    return R * c                

def meters_to_latlon(lat0: float, lon0: float, dx: float, dy: float) -> float:
    R = 6371000.0  # earth radius in meters

    lat0_rad = math.radians(lat0)
    dlat = (dy / R) * (180.0 / math.pi)
    dlon = (dx / (R * math.cos(lat0_rad))) * (180.0 / math.pi)

    return lat0 + dlat, lon0 + dlon

def nearest_neighbor_route(start, points):
    # start: (lat, lon)
    # points: list of {"lat":..., "lon":..., "alt":...}
    if not points:
        return []
    remaining = points.copy()
    route = []
    cur = {"lat": start[0], "lon": start[1]}
    while remaining:
        best_idx = None
        best_dist = float("inf")
        for i, p in enumerate(remaining):
            d = haversine_dist(cur["lat"], cur["lon"], p["lat"], p["lon"])
            if d < best_dist:
                best_dist = d
                best_idx = i
        route.append(remaining.pop(best_idx))
        cur = route[-1]
    return route
