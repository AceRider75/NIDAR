import customtkinter as ctk
import tkinter as tk
import math
import urllib.request
import os
import traceback
from concurrent.futures import ThreadPoolExecutor
from core.drone_state import DroneName

try:
    from PIL import Image, ImageTk
except ImportError:
    print("Pillow not installed. Map tiles won't load. pip install Pillow")
    Image = None
    ImageTk = None

# --- Web Mercator Projection Helpers ---
def lonlat_to_pixels(lon, lat, zoom):
    if Image is None: return 0, 0
    n = 2.0 ** zoom
    x = (lon + 180.0) / 360.0 * n * 256
    lat_rad = math.radians(lat)
    # protect against poles
    lat_rad = max(min(lat_rad, 1.48), -1.48) 
    y = (1.0 - math.asinh(math.tan(lat_rad)) / math.pi) / 2.0 * n * 256
    return x, y

def pixels_to_lonlat(px, py, zoom):
    if Image is None: return 0.0, 0.0
    n = 2.0 ** zoom
    lon_deg = (px / (256 * n)) * 360.0 - 180.0
    lat_rad = math.atan(math.sinh(math.pi * (1 - 2 * py / (256 * n))))
    lat_deg = math.degrees(lat_rad)
    return lon_deg, lat_deg

class MapView(ctk.CTkFrame):
    def __init__(self, master, controller=None, **kwargs):
        super().__init__(master, **kwargs)
        self.controller = controller
        
        # --- Map State ---
        self.zoom = 2
        self.center_lat = 0.0
        self.center_lon = 0.0
        self.drag_start = None
        self.active_drone = DroneName.Scanner
        self.follow_drone = True
        
        # Path history: {DroneName: [(lat, lon), ...]}
        self.paths = {DroneName.Scanner: [], DroneName.Sprayer: []}
        self.geofence_polygon = []
        self.waypoints = []
        
        # --- Tile System ---
        # Google Satellite
        self.tile_server_url = os.environ.get("NIDAR_TILE_SERVER", "https://mt0.google.com/vt/lyrs=s&hl=en&x={x}&y={y}&z={z}")
        self.tile_cache = {} # Memory cache (PhotoImages)
        self.tile_data = {}  # Raw bytes
        self.tile_executor = ThreadPoolExecutor(max_workers=4)
        
        # Disk cache setup
        self.cache_dir = os.path.join(os.getcwd(), "map_cache")
        os.makedirs(self.cache_dir, exist_ok=True)

        # --- UI Setup ---
        self.canvas = tk.Canvas(self, bg="#0f172a", highlightthickness=0)
        self.canvas.pack(fill="both", expand=True)
        
        # Bindings
        self.canvas.bind("<ButtonPress-1>", self.on_drag_start)
        self.canvas.bind("<B1-Motion>", self.on_drag_move)
        self.canvas.bind("<ButtonRelease-1>", self.on_click_release)
        self.canvas.bind("<MouseWheel>", self.on_zoom) # Windows
        self.canvas.bind("<Button-4>", self.on_zoom)   # Linux scroll up
        self.canvas.bind("<Button-5>", self.on_zoom)   # Linux scroll down
        
        # --- Overlay Controls ---
        self.btn_reload = ctk.CTkButton(self, text="⟳", width=30, height=30, command=self.reload_map)
        self.btn_reload.place(relx=0.96, rely=0.90, anchor="se")
        
        self.btn_follow = ctk.CTkButton(self, text="Follow: ON", width=80, height=30, command=self.toggle_follow,
                                        fg_color="#3b82f6")
        self.btn_follow.place(relx=0.04, rely=0.90, anchor="sw")

        # Start update loop
        self.after(100, self._update)

    def on_drag_start(self, event):
        self.drag_start = (event.x, event.y)

    def on_drag_move(self, event):
        # Disable follow mode if user manually drags
        if self.follow_drone:
            self.follow_drone = False
            self.btn_follow.configure(text="Follow: OFF", fg_color="#4b5563")
        if not self.drag_start: return
        dx = event.x - self.drag_start[0]
        dy = event.y - self.drag_start[1]
        self.drag_start = (event.x, event.y)
        
        # Shift center based on drag
        cx, cy = lonlat_to_pixels(self.center_lon, self.center_lat, self.zoom)
        cx -= dx
        cy -= dy
        self.center_lon, self.center_lat = pixels_to_lonlat(cx, cy, self.zoom)
        self._draw_map()

    def on_click_release(self, event):
        # If mouse didn't move much, treat as click to toggle drone
        if self.drag_start:
            dist = math.hypot(event.x - self.drag_start[0], event.y - self.drag_start[1])
            if dist < 5:
                self.toggle_active_drone()
        self.drag_start = None
        
    def toggle_active_drone(self):
        self.active_drone = DroneName.Sprayer if self.active_drone == DroneName.Scanner else DroneName.Scanner
        self._draw_map()
        
    def toggle_follow(self):
        self.follow_drone = not self.follow_drone
        text = "Follow: ON" if self.follow_drone else "Follow: OFF"
        color = "#3b82f6" if self.follow_drone else "#4b5563"
        self.btn_follow.configure(text=text, fg_color=color)
        self._draw_map()

    def change_zoom(self, delta):
        self.zoom = max(2, min(21, self.zoom + delta))
        self._draw_map()

    def reload_map(self):
        self.tile_data.clear()
        self.tile_cache.clear()
        self._draw_map()

    def set_geofence_polygon(self, coordinates: list):
        """Sets the geofence polygon to be drawn on the map."""
        self.geofence_polygon = coordinates
        self._draw_map()

    def display_waypoints(self, waypoints: list):
        """Sets the waypoints to be drawn on the map."""
        self.waypoints = waypoints
        self._draw_map()

    def clear_waypoints(self):
        """Clears all waypoints from the map."""
        self.waypoints = []
        self._draw_map()

    def on_zoom(self, event):
        if event.num == 4 or event.delta > 0:
            self.change_zoom(1)
        elif event.num == 5 or event.delta < 0:
            self.change_zoom(-1)

    def _update(self):
        if self.controller:
            # Update paths and center if tracking
            for d_name in [DroneName.Scanner, DroneName.Sprayer]:
                state = self.controller.get_drone_state(d_name)
                tele = state.get("telemetry", {})
                lat = tele.get("lat")
                lon = tele.get("lon")
                
                # Safe cast to float
                try:
                    lat = float(lat) if lat is not None else None
                    lon = float(lon) if lon is not None else None
                except (ValueError, TypeError):
                    continue

                # Filter invalid None (Allow 0,0)
                if lat is not None and lon is not None:
                    # Append to path if moved significantly
                    path = self.paths[d_name]
                    if not path or (abs(path[-1][0] - lat) > 1e-6 or abs(path[-1][1] - lon) > 1e-6):
                        path.append((lat, lon))
                        if len(path) > 2000: path.pop(0) # Limit path length
                    
                    # Auto-center on active drone
                    if d_name == self.active_drone and self.follow_drone:
                        self.center_lat = lat
                        self.center_lon = lon

        self._draw_map()
        self.after(500, self._update)

    def _background_fetch_tile(self, z, x, y):
        key = (z, x, y)
        filename = f"{z}_{x}_{y}.img"
        filepath = os.path.join(self.cache_dir, filename)
        
        try:
            # 1. Check disk cache
            if os.path.exists(filepath):
                with open(filepath, "rb") as f:
                    data = f.read()
                self.tile_data[key] = data
                self.after(0, self._draw_map)
                return

            # 2. Download
            url = self.tile_server_url.format(z=z, x=x, y=y)
            req = urllib.request.Request(url, headers={"User-Agent": "Mozilla/5.0 (X11; Linux x86_64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/122.0.0.0 Safari/537.36"})
            with urllib.request.urlopen(req, timeout=5) as resp:
                data = resp.read()
            
            # 3. Save to disk
            with open(filepath, "wb") as f:
                f.write(data)
                
            self.tile_data[key] = data
            self.after(0, self._draw_map)
        except Exception:
            # Remove from pending so it can be retried
            self.tile_data.pop(key, None)

    def _draw_map(self):
        self.canvas.delete("all")
        width = self.winfo_width()
        height = self.winfo_height()
        if width < 10 or height < 10: return

        # Center pixel
        center_px, center_py = lonlat_to_pixels(self.center_lon, self.center_lat, self.zoom)
        
        # Determine visible tiles (256x256)
        tl_px = center_px - width / 2
        tl_py = center_py - height / 2
        
        start_tx = int(tl_px / 256)
        start_ty = int(tl_py / 256)
        end_tx = int((tl_px + width) / 256) + 1
        end_ty = int((tl_py + height) / 256) + 1
        
        # Draw tiles
        for tx in range(start_tx, end_tx + 1):
            for ty in range(start_ty, end_ty + 1):
                tx_wrapped = tx % (2**self.zoom)
                key = (self.zoom, tx_wrapped, ty)
                
                tile_x = tx * 256 - tl_px
                tile_y = ty * 256 - tl_py
                
                if key in self.tile_data:
                    if key not in self.tile_cache:
                        try:
                            import io
                            img = Image.open(io.BytesIO(self.tile_data[key]))
                            self.tile_cache[key] = ImageTk.PhotoImage(img)
                        except: pass
                    
                    if key in self.tile_cache:
                        self.canvas.create_image(int(tile_x), int(tile_y), image=self.tile_cache[key], anchor="nw")
                else:
                    if key not in self.tile_data: # Fetch if not pending
                        self.tile_data[key] = None 
                        self.tile_executor.submit(self._background_fetch_tile, self.zoom, tx_wrapped, ty)
                    
        # Draw Paths & Markers
        for d_name in [DroneName.Scanner, DroneName.Sprayer]:
            path = self.paths.get(d_name, [])
            if not path: continue
            
            is_active = (d_name == self.active_drone)
            coords = []
            for lat, lon in path:
                px, py = lonlat_to_pixels(lon, lat, self.zoom)
                coords.extend([px - tl_px, py - tl_py])
            
            # Draw Line
            if len(coords) >= 4:
                color = "#3b82f6" if d_name == DroneName.Scanner else "#ef4444"
                if not is_active: color = "#1e3a8a" if d_name == DroneName.Scanner else "#7f1d1d"
                self.canvas.create_line(*coords, fill=color, width=3 if is_active else 2, smooth=True)
                
            # Draw Marker
            if coords:
                cx, cy = coords[-2], coords[-1]
                fill = "#60a5fa" if d_name == DroneName.Scanner else "#fca5a5"
                if not is_active: fill = "#1d4ed8" if d_name == DroneName.Scanner else "#991b1b"
                r = 6 if is_active else 4
                self.canvas.create_oval(cx-r, cy-r, cx+r, cy+r, fill=fill, outline="white" if is_active else "")

        # Draw Geofence
        if self.geofence_polygon:
            poly_coords = []
            for lat, lon in self.geofence_polygon:
                px, py = lonlat_to_pixels(lon, lat, self.zoom)
                poly_coords.extend([px - tl_px, py - tl_py])
            
            if len(poly_coords) >= 6:
                self.canvas.create_polygon(poly_coords,
                                           outline="#f87171",
                                           fill="#ef4444",
                                           stipple="gray50",
                                           width=2)

        # Draw Waypoints
        for wp in self.waypoints:
            lat, lon = wp.get("lat"), wp.get("lon")
            if lat is None or lon is None:
                continue

            px, py = lonlat_to_pixels(lon, lat, self.zoom)
            cx, cy = px - tl_px, py - tl_py
            
            # Draw circle for waypoint
            r = 5
            self.canvas.create_oval(cx - r, cy - r, cx + r, cy + r, 
                                    fill="#ffc107", outline="white")
            
            # Draw spot_id text
            spot_id = wp.get('spot_id', 'WP')
            self.canvas.create_text(cx + 8, cy, 
                                    text=spot_id, 
                                    anchor="w",
                                    fill="white",
                                    font=("Arial", 10, "bold"))

        # If active drone has no GPS history, show warning
        if not self.paths.get(self.active_drone):
            self.canvas.create_text(width/2, height/2, text="Waiting for GPS Fix...", fill="red", font=("Arial", 16, "bold"))
            self.canvas.create_text(width/2, height/2 + 25, text="(Drone at 0,0 or not connected)", fill="white", font=("Arial", 10))

        # Legend
        self.canvas.create_rectangle(10, 10, 160, 50, fill="#000000", outline="")
        self.canvas.create_text(20, 20, anchor="nw", text=f"Active: {self.active_drone.name}", fill="white", font=("Arial", 10, "bold"))
        self.canvas.create_text(20, 35, anchor="nw", text="Click map to toggle", fill="#ccc", font=("Arial", 9))

        # Top Right Info
        info_text = f"Lat: {self.center_lat:.6f}  Lon: {self.center_lon:.6f}  Zoom: {self.zoom}"
        self.canvas.create_text(width - 10, 10, anchor="ne", text=info_text, fill="#cbd5e1", font=("Arial", 10, "bold"))
