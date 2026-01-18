import customtkinter as ctk
import tkinter as tk
import math
import urllib.request
import os
import traceback
import time
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

def ease_out_cubic(t):
    """Easing function for smooth animations"""
    return 1 - (1 - t) ** 3

def ease_out_expo(t):
    """Exponential easing for smoother deceleration"""
    return 1 if t == 1 else 1 - pow(2, -10 * t)

class MapView(ctk.CTkFrame):
    def __init__(self, master, controller=None, **kwargs):
        super().__init__(master, **kwargs)
        self.controller = controller
        
        # --- Map State ---
        self.zoom = 2
        self.target_zoom = 2  # For smooth zoom animation
        self.center_lat = 0.0
        self.center_lon = 0.0
        self.target_lat = 0.0  # For smooth pan animation
        self.target_lon = 0.0
        self.drag_start = None
        self.active_drone = DroneName.Scanner
        self.follow_drone = True
        
        # --- Smooth Animation State ---
        self.animation_duration = 200  # Default ms for animations (faster = snappier)
        self.zoom_animation_duration = 200
        self.pan_animation_duration = 200
        self.zoom_animation_active = False
        self.pan_animation_active = False
        self.zoom_anim_start_time = 0
        self.pan_anim_start_time = 0
        self.zoom_start_value = 2
        self.pan_start_lat = 0.0
        self.pan_start_lon = 0.0
        
        # --- Inertial Scrolling State ---
        self.velocity_x = 0.0
        self.velocity_y = 0.0
        self.last_drag_time = 0
        self.last_drag_x = 0
        self.last_drag_y = 0
        self.inertia_active = False
        self.inertia_friction = 0.92  # Friction coefficient
        self.min_velocity = 0.5  # Stop inertia below this
        
        # --- Animation Throttling ---
        self.last_draw_time = 0
        self.min_draw_interval = 16  # ~60fps for smooth animations
        
        # Path history: {DroneName: [(lat, lon), ...]}
        self.paths = {DroneName.Scanner: [], DroneName.Sprayer: []}
        self.geofence_polygon = []
        self.waypoints = []
        
        # --- Tile System ---
        # Google Satellite
        self.tile_server_url = os.environ.get("NIDAR_TILE_SERVER", "https://mt0.google.com/vt/lyrs=s&hl=en&x={x}&y={y}&z={z}")
        self.tile_cache = {} # Memory cache (PhotoImages)
        self.tile_data = {}  # Raw bytes
        self.pending_tiles = set()  # Track tiles being fetched to avoid duplicates
        self.tile_executor = ThreadPoolExecutor(max_workers=8)  # More workers for faster loading
        
        # Disk cache setup
        self.cache_dir = os.path.join(os.getcwd(), "map_cache")
        os.makedirs(self.cache_dir, exist_ok=True)
        
        # Preload disk cache index for faster lookups
        self.disk_cache_set = set()
        self._load_disk_cache_index()

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
        # Double-click to auto-fit
        self.canvas.bind("<Double-Button-1>", self.on_double_click)
        
        # --- Overlay Controls ---
        self.btn_reload = ctk.CTkButton(self, text="⟳", width=30, height=30, command=self.reload_map)
        self.btn_reload.place(relx=0.96, rely=0.90, anchor="se")
        
        self.btn_follow = ctk.CTkButton(self, text="Follow: ON", width=80, height=30, command=self.toggle_follow,
                                        fg_color="#3b82f6")
        self.btn_follow.place(relx=0.04, rely=0.90, anchor="sw")
        
        # Auto-fit button
        self.btn_autofit = ctk.CTkButton(self, text="⛶", width=30, height=30, command=self.auto_fit_to_content,
                                         fg_color="#10b981")
        self.btn_autofit.place(relx=0.96, rely=0.83, anchor="se")
        
        # Zoom buttons for touch-friendly control
        self.btn_zoom_in = ctk.CTkButton(self, text="+", width=30, height=30, command=lambda: self.smooth_zoom(1))
        self.btn_zoom_in.place(relx=0.96, rely=0.05, anchor="ne")
        
        self.btn_zoom_out = ctk.CTkButton(self, text="-", width=30, height=30, command=lambda: self.smooth_zoom(-1))
        self.btn_zoom_out.place(relx=0.96, rely=0.12, anchor="ne")

        # Start update loop
        self.after(100, self._update)
        # Start animation loop
        self.after(16, self._animation_loop)  # ~60fps

    def on_drag_start(self, event):
        self.drag_start = (event.x, event.y)
        # Stop any inertia when user starts dragging
        self.inertia_active = False
        self.velocity_x = 0
        self.velocity_y = 0
        self.last_drag_time = time.time()
        self.last_drag_x = event.x
        self.last_drag_y = event.y

    def on_drag_move(self, event):
        # Disable follow mode if user manually drags
        if self.follow_drone:
            self.follow_drone = False
            self.btn_follow.configure(text="Follow: OFF", fg_color="#4b5563")
        if not self.drag_start: return
        
        dx = event.x - self.drag_start[0]
        dy = event.y - self.drag_start[1]
        self.drag_start = (event.x, event.y)
        
        # Track velocity for inertia
        current_time = time.time()
        dt = current_time - self.last_drag_time
        if dt > 0:
            # Smooth velocity with exponential moving average
            alpha = 0.3
            new_vx = (event.x - self.last_drag_x) / max(dt, 0.016)
            new_vy = (event.y - self.last_drag_y) / max(dt, 0.016)
            self.velocity_x = alpha * new_vx + (1 - alpha) * self.velocity_x
            self.velocity_y = alpha * new_vy + (1 - alpha) * self.velocity_y
        
        self.last_drag_time = current_time
        self.last_drag_x = event.x
        self.last_drag_y = event.y
        
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
            else:
                # Start inertial scrolling if there's sufficient velocity
                speed = math.hypot(self.velocity_x, self.velocity_y)
                if speed > 50:  # Minimum speed threshold
                    self.inertia_active = True
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
        self.pending_tiles.clear()
        self._load_disk_cache_index()
        self._draw_map()
    
    def _load_disk_cache_index(self):
        """Preload list of cached tiles for faster lookups"""
        try:
            self.disk_cache_set = set(os.listdir(self.cache_dir))
        except:
            self.disk_cache_set = set()

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
        """Center-based smooth zoom - keeps drone positions as focus"""
        # Determine zoom direction
        if event.num == 4 or event.delta > 0:
            delta = 1
        elif event.num == 5 or event.delta < 0:
            delta = -1
        else:
            return
        
        # Simply smooth zoom at current center (no mouse-centered adjustment)
        self.smooth_zoom(delta)
    
    def on_double_click(self, event):
        """Double-click to zoom in centered on click position"""
        # Get mouse position
        mouse_x = event.x
        mouse_y = event.y
        
        width = self.winfo_width()
        height = self.winfo_height()
        center_px, center_py = lonlat_to_pixels(self.center_lon, self.center_lat, self.zoom)
        
        # Mouse position in world pixels
        mouse_world_px = center_px - width/2 + mouse_x
        mouse_world_py = center_py - height/2 + mouse_y
        click_lon, click_lat = pixels_to_lonlat(mouse_world_px, mouse_world_py, self.zoom)
        
        # Zoom in 2 levels and center on click
        new_zoom = min(21, self.zoom + 2)
        self.smooth_pan_to(click_lat, click_lon)
        self.smooth_zoom(new_zoom - self.zoom)
    
    def smooth_zoom(self, delta, duration=None):
        """Start a smooth zoom animation"""
        target = max(2, min(21, self.target_zoom + delta))
        if target == self.target_zoom:
            return
        
        self.zoom_start_value = self.zoom
        self.target_zoom = target
        self.zoom_anim_start_time = time.time() * 1000
        self.zoom_animation_duration = duration if duration is not None else self.animation_duration
        self.zoom_animation_active = True
    
    def smooth_zoom_to(self, new_zoom, new_lat, new_lon, duration=None):
        """Smooth zoom with recentering"""
        self.zoom_start_value = self.zoom
        self.target_zoom = new_zoom
        self.zoom_anim_start_time = time.time() * 1000
        self.zoom_animation_duration = duration if duration is not None else self.animation_duration
        self.zoom_animation_active = True
        
        # Also animate pan
        self.pan_start_lat = self.center_lat
        self.pan_start_lon = self.center_lon
        self.target_lat = new_lat
        self.target_lon = new_lon
        self.pan_anim_start_time = time.time() * 1000
        self.pan_animation_duration = duration if duration is not None else self.animation_duration
        self.pan_animation_active = True
    
    def smooth_pan_to(self, lat, lon, duration=None):
        """Start a smooth pan animation to target coordinates"""
        self.pan_start_lat = self.center_lat
        self.pan_start_lon = self.center_lon
        self.target_lat = lat
        self.target_lon = lon
        self.pan_anim_start_time = time.time() * 1000
        self.pan_animation_duration = duration if duration is not None else self.animation_duration
        self.pan_animation_active = True
        # Disable follow when manually panning
        if self.follow_drone:
            self.follow_drone = False
            self.btn_follow.configure(text="Follow: OFF", fg_color="#4b5563")
    
    def _animation_loop(self):
        """Main animation loop running at ~60fps"""
        needs_redraw = False
        current_time = time.time() * 1000
        
        # Handle zoom animation
        if self.zoom_animation_active:
            elapsed = current_time - self.zoom_anim_start_time
            progress = min(1.0, elapsed / self.zoom_animation_duration)
            eased = ease_out_expo(progress)
            
            self.zoom = self.zoom_start_value + (self.target_zoom - self.zoom_start_value) * eased
            
            if progress >= 1.0:
                self.zoom = self.target_zoom
                self.zoom_animation_active = False
            needs_redraw = True
        
        # Handle pan animation
        if self.pan_animation_active:
            elapsed = current_time - self.pan_anim_start_time
            progress = min(1.0, elapsed / self.pan_animation_duration)
            eased = ease_out_cubic(progress)
            
            self.center_lat = self.pan_start_lat + (self.target_lat - self.pan_start_lat) * eased
            self.center_lon = self.pan_start_lon + (self.target_lon - self.pan_start_lon) * eased
            
            if progress >= 1.0:
                self.center_lat = self.target_lat
                self.center_lon = self.target_lon
                self.pan_animation_active = False
            needs_redraw = True
        
        # Handle inertial scrolling
        if self.inertia_active:
            # Apply friction
            self.velocity_x *= self.inertia_friction
            self.velocity_y *= self.inertia_friction
            
            speed = math.hypot(self.velocity_x, self.velocity_y)
            if speed < self.min_velocity:
                self.inertia_active = False
                self.velocity_x = 0
                self.velocity_y = 0
            else:
                # Move the map based on velocity (16ms frame time)
                dx = self.velocity_x * 0.016
                dy = self.velocity_y * 0.016
                
                cx, cy = lonlat_to_pixels(self.center_lon, self.center_lat, self.zoom)
                cx -= dx
                cy -= dy
                self.center_lon, self.center_lat = pixels_to_lonlat(cx, cy, self.zoom)
                needs_redraw = True
        
        if needs_redraw:
            # Throttle drawing for consistent frame rate
            time_since_last_draw = current_time - self.last_draw_time
            if time_since_last_draw >= self.min_draw_interval:
                self._draw_map()
                self.last_draw_time = current_time
        
        # Schedule next frame at 60fps for smooth animations
        self.after(16, self._animation_loop)
    
    def auto_fit_to_content(self):
        """Auto-fit the map to show all content (waypoints, geofence, drone paths)"""
        all_points = []
        
        # Collect waypoints
        for wp in self.waypoints:
            lat, lon = wp.get("lat"), wp.get("lon")
            if lat is not None and lon is not None:
                all_points.append((lat, lon))
        
        # Collect geofence points
        for lat, lon in self.geofence_polygon:
            all_points.append((lat, lon))
        
        # Collect drone path endpoints
        for d_name in [DroneName.Scanner, DroneName.Sprayer]:
            path = self.paths.get(d_name, [])
            if path:
                all_points.extend(path[-10:])  # Last 10 points from each path
        
        if not all_points:
            return
        
        # Find bounding box
        min_lat = min(p[0] for p in all_points)
        max_lat = max(p[0] for p in all_points)
        min_lon = min(p[1] for p in all_points)
        max_lon = max(p[1] for p in all_points)
        
        # Calculate center
        center_lat = (min_lat + max_lat) / 2
        center_lon = (min_lon + max_lon) / 2
        
        # Calculate zoom level to fit all points with padding
        width = max(10, self.winfo_width())
        height = max(10, self.winfo_height())
        padding = 0.2  # 20% padding
        
        # Find best zoom level
        best_zoom = 2
        for z in range(21, 1, -1):
            min_px, min_py = lonlat_to_pixels(min_lon, max_lat, z)
            max_px, max_py = lonlat_to_pixels(max_lon, min_lat, z)
            
            content_width = abs(max_px - min_px)
            content_height = abs(max_py - min_py)
            
            available_width = width * (1 - padding)
            available_height = height * (1 - padding)
            
            if content_width <= available_width and content_height <= available_height:
                best_zoom = z
                break
        
        # Animate to the new view with a longer duration for smoothness
        self.smooth_zoom_to(best_zoom, center_lat, center_lon, duration=5000)
        
        # Disable follow mode
        if self.follow_drone:
            self.follow_drone = False
            self.btn_follow.configure(text="Follow: OFF", fg_color="#4b5563")

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
                    
                    # Auto-center on active drone with smooth follow
                    if d_name == self.active_drone and self.follow_drone:
                        # Use smooth panning for a nicer follow experience
                        # Only animate if not already animating or if target moved significantly
                        dist_to_target = math.hypot(lat - self.target_lat, lon - self.target_lon)
                        if not self.pan_animation_active or dist_to_target > 0.0001:
                            self.pan_start_lat = self.center_lat
                            self.pan_start_lon = self.center_lon
                            self.target_lat = lat
                            self.target_lon = lon
                            self.pan_anim_start_time = time.time() * 1000
                            self.pan_animation_active = True

        self._draw_map()
        self.after(500, self._update)

    def _background_fetch_tile(self, z, x, y):
        key = (z, x, y)
        filename = f"{z}_{x}_{y}.img"
        filepath = os.path.join(self.cache_dir, filename)
        
        try:
            # 1. Check disk cache (use cached index for speed)
            if filename in self.disk_cache_set or os.path.exists(filepath):
                with open(filepath, "rb") as f:
                    data = f.read()
                self.tile_data[key] = data
                self.disk_cache_set.add(filename)
                self.pending_tiles.discard(key)
                self.after(0, self._draw_map)
                return

            # 2. Download with shorter timeout
            url = self.tile_server_url.format(z=z, x=x, y=y)
            req = urllib.request.Request(url, headers={"User-Agent": "Mozilla/5.0 (X11; Linux x86_64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/122.0.0.0 Safari/537.36"})
            with urllib.request.urlopen(req, timeout=3) as resp:
                data = resp.read()
            
            # 3. Save to disk
            with open(filepath, "wb") as f:
                f.write(data)
            
            self.disk_cache_set.add(filename)
            self.tile_data[key] = data
            self.pending_tiles.discard(key)
            self.after(0, self._draw_map)
        except Exception:
            # Remove from pending so it can be retried
            self.tile_data.pop(key, None)
            self.pending_tiles.discard(key)

    def _draw_map(self):
        self.canvas.delete("all")
        width = self.winfo_width()
        height = self.winfo_height()
        if width < 10 or height < 10: return

        # Use integer zoom for tile fetching (tiles are only available at integer levels)
        display_zoom = int(self.zoom)
        
        # Calculate scale factor for smooth zoom animation
        # When zoom is fractional (e.g., 15.3), scale tiles from zoom 15 by 2^0.3
        zoom_fraction = self.zoom - display_zoom
        tile_scale = 2 ** zoom_fraction  # Scale factor for smooth visual transition
        scaled_tile_size = int(256 * tile_scale)
        
        # Center pixel at the base zoom level
        center_px, center_py = lonlat_to_pixels(self.center_lon, self.center_lat, display_zoom)
        
        # Adjust for scale - tile positions are scaled from center
        tl_px = center_px - (width / 2) / tile_scale
        tl_py = center_py - (height / 2) / tile_scale
        
        # Determine visible tiles
        start_tx = int(tl_px / 256) - 1
        start_ty = int(tl_py / 256) - 1
        end_tx = int((tl_px + width / tile_scale) / 256) + 1
        end_ty = int((tl_py + height / tile_scale) / 256) + 1
        
        # Draw tiles with scaling
        import io
        for tx in range(start_tx, end_tx + 1):
            for ty in range(start_ty, end_ty + 1):
                max_tile = 2**display_zoom
                if ty < 0 or ty >= max_tile:
                    continue
                tx_wrapped = tx % max_tile
                key = (display_zoom, tx_wrapped, ty)
                
                # Calculate tile position with scaling
                tile_x = (tx * 256 - tl_px) * tile_scale
                tile_y = (ty * 256 - tl_py) * tile_scale
                
                if key in self.tile_data and self.tile_data[key] is not None:
                    # Create or get cached PhotoImage
                    # For smooth scaling, we need to resize the image during animations
                    if tile_scale != 1.0:
                        # Create scaled version for animation frames
                        scale_key = (display_zoom, tx_wrapped, ty, round(tile_scale * 100))
                        if scale_key not in self.tile_cache:
                            try:
                                img = Image.open(io.BytesIO(self.tile_data[key]))
                                if tile_scale != 1.0:
                                    img = img.resize((scaled_tile_size, scaled_tile_size), Image.Resampling.NEAREST)
                                self.tile_cache[scale_key] = ImageTk.PhotoImage(img)
                            except: pass
                        
                        if scale_key in self.tile_cache:
                            self.canvas.create_image(int(tile_x), int(tile_y), image=self.tile_cache[scale_key], anchor="nw")
                    else:
                        # Normal unscaled tile
                        if key not in self.tile_cache:
                            try:
                                img = Image.open(io.BytesIO(self.tile_data[key]))
                                self.tile_cache[key] = ImageTk.PhotoImage(img)
                            except: pass
                        
                        if key in self.tile_cache:
                            self.canvas.create_image(int(tile_x), int(tile_y), image=self.tile_cache[key], anchor="nw")
                else:
                    # Only fetch if not already pending or fetched
                    if key not in self.pending_tiles and key not in self.tile_data:
                        self.pending_tiles.add(key)
                        self.tile_executor.submit(self._background_fetch_tile, display_zoom, tx_wrapped, ty)
        
        # Also use the actual fractional zoom for drawing paths/markers for smooth visual
        actual_zoom = self.zoom
        # Recalculate top-left in screen coordinates for overlays
        overlay_center_px, overlay_center_py = lonlat_to_pixels(self.center_lon, self.center_lat, actual_zoom)
        overlay_tl_px = overlay_center_px - width / 2
        overlay_tl_py = overlay_center_py - height / 2
                    
        # Draw Paths & Markers
        for d_name in [DroneName.Scanner, DroneName.Sprayer]:
            path = self.paths.get(d_name, [])
            if not path: continue
            
            is_active = (d_name == self.active_drone)
            coords = []
            for lat, lon in path:
                px, py = lonlat_to_pixels(lon, lat, actual_zoom)
                coords.extend([px - overlay_tl_px, py - overlay_tl_py])
            
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
        info_text = f"Lat: {self.center_lat:.6f}  Lon: {self.center_lon:.6f}"
        self.canvas.create_text(width - 10, 10, anchor="ne", text=info_text, fill="#cbd5e1", font=("Arial", 10, "bold"))
