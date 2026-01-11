import customtkinter as ctk
import tkinter as tk
import math
import urllib.request
import io
from core.drone_state import DroneName
import traceback
from concurrent.futures import ThreadPoolExecutor
import os

try:
    from PIL import Image, ImageTk
    PIL_AVAILABLE = True
except Exception:
    PIL_AVAILABLE = False


class MapView(ctk.CTkFrame):
    UPDATE_INTERVAL_MS = 500

    def __init__(self, master, controller=None, **kwargs):
        super().__init__(master, **kwargs)
        self.controller = controller

        # Canvas for drawing map and paths
        self.canvas = tk.Canvas(self, bg="#0b0f14", highlightthickness=0)
        self.canvas.pack(fill="both", expand=True)

        # Storage for recorded paths (meters relative to each drone's origin)
        self.paths = {
            DroneName.Scanner: [],
            DroneName.Sprayer: []
        }
        # Also store absolute lat/lon history for map overlay (list of (lat, lon))
        self.raw_paths = {
            DroneName.Scanner: [],
            DroneName.Sprayer: []
        }

        # Tile cache and images to prevent GC
        self.tile_cache = {}
        self.tile_images = {}
        # Raw tile bytes cache (filled by background fetches)
        self.tile_data = {}
        # Thread pool for background tile fetches
        self.tile_executor = ThreadPoolExecutor(max_workers=4)
        
        # Disk cache directory
        self.cache_dir = os.path.join(os.getcwd(), "map_cache")
        os.makedirs(self.cache_dir, exist_ok=True)

        # Allow overriding tile server via env var or default to OSM
        # Use Google Satellite for a realistic "3D coloured" look (more reliable availability)
        self.tile_server_url = os.environ.get("NIDAR_TILE_SERVER", "https://mt0.google.com/vt/lyrs=s&hl=en&x={x}&y={y}&z={z}")

        # tile load status
        self.last_tile_success = False

        # small reload button to retry fetching tiles
        try:
            self.reload_btn = ctk.CTkButton(self, text="Reload tiles", width=100, height=24, command=self._reload_tiles, fg_color="#0b1220")
            # place it so it floats over the canvas
            self.reload_btn.place(x=8, y=48)
        except Exception:
            self.reload_btn = None

        # Simulation button to inject sample GPS points for testing
        try:
            self.sim_btn = ctk.CTkButton(self, text="Simulate path", width=110, height=24, command=self._start_simulation, fg_color="#0b1220")
            self.sim_btn.place(x=8, y=80)
        except Exception:
            self.sim_btn = None

        # Zoom settings
        self.zoom = 16
        self.min_zoom = 1
        self.max_zoom = 19

        # Bind mouse wheel for zoom
        # Windows/Mac
        self.canvas.bind("<MouseWheel>", self._on_mousewheel)
        # Linux scroll
        self.canvas.bind("<Button-4>", self._on_mousewheel)
        self.canvas.bind("<Button-5>", self._on_mousewheel)

        # Which drone's path is shown on the map (toggle by click)
        self.display_mode = DroneName.Scanner

        # Bind click to toggle displayed drone
        self.canvas.bind("<Button-1>", self._on_click_toggle)

        # Schedule updates
        self.after(self.UPDATE_INTERVAL_MS, self._update)

    def _on_click_toggle(self, event):
        try:
            # Toggle between scanner and sprayer display
            if self.display_mode == DroneName.Scanner:
                self.display_mode = DroneName.Sprayer
            else:
                self.display_mode = DroneName.Scanner
            # Redraw immediately
            self._draw()
        except Exception:
            print("[MapView] _on_click_toggle exception:")
            traceback.print_exc()

    def _on_mousewheel(self, event):
        try:
            # Handle both Windows (event.delta) and X11 (Button-4/Button-5)
            old_zoom = self.zoom
            if hasattr(event, "delta") and event.delta:
                if event.delta > 0:
                    self.zoom = min(self.max_zoom, self.zoom + 1)
                else:
                    self.zoom = max(self.min_zoom, self.zoom - 1)
            else:
                # event.num == 4 -> up, 5 -> down on some Linux setups
                if getattr(event, "num", None) == 4:
                    self.zoom = min(self.max_zoom, self.zoom + 1)
                elif getattr(event, "num", None) == 5:
                    self.zoom = max(self.min_zoom, self.zoom - 1)

            if self.zoom != old_zoom:
                # clear tile_images to force redraw with new zoom
                self.tile_images.clear()
                self.tile_cache.clear()
                self._draw()
        except Exception:
            print("[MapView] _on_mousewheel exception:")
            traceback.print_exc()

    def _reload_tiles(self):
        try:
            self.tile_cache.clear()
            self.tile_images.clear()
            self.last_tile_success = False
            self._draw()
        except Exception:
            print("[MapView] _reload_tiles exception:")
            traceback.print_exc()

    def _update(self):
        # Grab latest telemetry for both drones and record their positions
        if self.controller is None:
            self.after(self.UPDATE_INTERVAL_MS, self._update)
            return

        try:
            scanner_state = self.controller.get_drone_state(DroneName.Scanner)
            sprayer_state = self.controller.get_drone_state(DroneName.Sprayer)

            # Convert lat/lon to meters relative to each drone's lat0/lon0
            self._record_position(DroneName.Scanner, scanner_state["telemetry"], self.controller.drone_states[DroneName.Scanner.value].lat0, self.controller.drone_states[DroneName.Scanner.value].lon0)
            self._record_position(DroneName.Sprayer, sprayer_state["telemetry"], self.controller.drone_states[DroneName.Sprayer.value].lat0, self.controller.drone_states[DroneName.Sprayer.value].lon0)
        except Exception:
            pass

        self._draw()
        self.after(self.UPDATE_INTERVAL_MS, self._update)

    def _record_position(self, drone_enum, telemetry, lat0, lon0):
        try:
            lat = telemetry.get("lat", 0.0)
            lon = telemetry.get("lon", 0.0)
        except Exception:
            return

        # Ignore invalid 0,0 coordinates (Null Island) so map doesn't jump there on startup
        if abs(lat) < 0.0001 and abs(lon) < 0.0001:
            return

        meters_per_deg_lat = 111320.0
        meters_per_deg_lon = 111320.0 * math.cos(math.radians(lat0 if lat0 else lat))

        dx = (lat - lat0) * meters_per_deg_lat
        dy = (lon - lon0) * meters_per_deg_lon

        path_list = self.paths.get(drone_enum)
        if path_list is None:
            self.paths[drone_enum] = [(dx, dy)]
        else:
            # Avoid appending duplicates (small movements still recorded)
            if not path_list or (abs(path_list[-1][0] - dx) > 0.01 or abs(path_list[-1][1] - dy) > 0.01):
                path_list.append((dx, dy))
        # store absolute lat/lon for map overlay
        raw_list = self.raw_paths.get(drone_enum)
        if raw_list is None:
            self.raw_paths[drone_enum] = [(lat, lon)]
        else:
            if not raw_list or (abs(raw_list[-1][0] - lat) > 1e-7 or abs(raw_list[-1][1] - lon) > 1e-7):
                raw_list.append((lat, lon))

    def _draw(self, no_color_updates=False, *args, **kwargs):
        # CTkFrame may call _draw during its own __init__ before MapView.__init__ runs
        # Guard against missing canvas to avoid AttributeError during widget setup.
        if not hasattr(self, "canvas") or self.canvas is None:
            return
        # Clear canvas
        try:
            self.canvas.delete("all")
        except Exception:
            print("[MapView] _draw: failed to clear canvas")
            traceback.print_exc()
            return

        width = self.canvas.winfo_width() or 400
        height = self.canvas.winfo_height() or 300

        # Choose which path to display
        active = self.display_mode
        points = self.paths.get(active, [])
        raw_points = self.raw_paths.get(active, [])

        # If there are no lat/lon points yet, default to 0.0,0.0 and indicate no GPS
        if not raw_points:
            # Default to 0,0
            center_lat, center_lon = 0.0, 0.0
            no_gps = True
        else:
            center_lat, center_lon = raw_points[-1]
            no_gps = False

        # use current zoom
        zoom = self.zoom

        # helper: lon/lat to global pixel coordinates at zoom
        def lonlat_to_pixels(lon, lat, z):
            n = 2 ** z
            x = (lon + 180.0) / 360.0 * 256.0 * n
            lat_rad = math.radians(lat)
            y = (1.0 - math.log(math.tan(lat_rad) + (1 / math.cos(lat_rad))) / math.pi) / 2.0 * 256.0 * n
            return x, y

        center_px, center_py = lonlat_to_pixels(center_lon, center_lat, zoom)

        # determine which tiles are needed to cover the canvas
        tile_size = 256
        tiles_x = math.ceil(width / tile_size) + 2
        tiles_y = math.ceil(height / tile_size) + 2

        n = 2 ** zoom
        center_xtile = center_px / tile_size
        center_ytile = center_py / tile_size
        min_xtile = int(math.floor(center_xtile - tiles_x // 2))
        max_xtile = int(math.floor(center_xtile + tiles_x // 2))
        min_ytile = int(math.floor(center_ytile - tiles_y // 2))
        max_ytile = int(math.floor(center_ytile + tiles_y // 2))

        # Draw map tiles
        drawn_tiles = 0
        try:
            if PIL_AVAILABLE:
                for tx in range(min_xtile, max_xtile + 1):
                    for ty in range(min_ytile, max_ytile + 1):
                        tx_wrapped = tx % n
                        if ty < 0 or ty >= n:
                            continue
                        tile_img = self._fetch_tile(zoom, tx_wrapped, ty)
                        if tile_img is None:
                            continue
                        # compute position
                        tile_px = (tx - center_xtile) * tile_size + width / 2
                        tile_py = (ty - center_ytile) * tile_size + height / 2
                        # keep reference to avoid GC
                        key = (zoom, tx_wrapped, ty)
                        self.tile_images[key] = tile_img
                        self.canvas.create_image(int(tile_px), int(tile_py), image=tile_img, anchor="nw")
                        drawn_tiles += 1
            else:
                # PIL not available — show a message instructing installation
                self.canvas.create_text(width // 2, height // 2 - 20, text="Install Pillow to view map tiles", fill="#f97316", font=("Arial", 12, "bold"))
                self.canvas.create_text(width // 2, height // 2 + 6, text="python -m pip install Pillow", fill="#9ca3af", font=("Arial", 10))

            # If no tiles were drawn, show helpful message
            if drawn_tiles == 0:
                self.last_tile_success = False
                self.canvas.create_text(width / 2, height / 2 - 30, text="Tiles unavailable — check network or tile server", fill="#f97316", font=("Arial", 12, "bold"))
                self.canvas.create_text(width / 2, height / 2 - 12, text="Click 'Reload tiles' or press mouse wheel to change zoom and retry", fill="#9ca3af", font=("Arial", 10))
            else:
                self.last_tile_success = True
        except Exception:
            print("[MapView] _draw tile rendering exception:")
            traceback.print_exc()
        
        # Draw paths for ALL drones (inactive first so active is on top)
        drones_to_draw = [d for d in DroneName if d != active] + [active]
        
        for drone in drones_to_draw:
            path = self.raw_paths.get(drone, [])
            if not path:
                continue

            is_active = (drone == active)
            
            # Convert lat/lon path to canvas coordinates
            coords_canvas = []
            for lat, lon in path:
                px, py = lonlat_to_pixels(lon, lat, zoom)
                cx = px - center_px + width / 2
                cy = py - center_py + height / 2
                coords_canvas.append((cx, cy))

            # Draw line
            if len(coords_canvas) >= 2:
                flat = [c for pt in coords_canvas for c in pt]
                # Bright colors for active, dim for inactive
                if drone == DroneName.Scanner:
                    color = "#3b82f6" if is_active else "#1e3a8a"
                else:
                    color = "#ef4444" if is_active else "#7f1d1d"
                
                width_line = 3 if is_active else 2
                self.canvas.create_line(*flat, fill=color, width=width_line, smooth=True)

            # Draw current position marker
            cur_x, cur_y = coords_canvas[-1]
            if drone == DroneName.Scanner:
                marker_fill = "#60a5fa" if is_active else "#1d4ed8"
            else:
                marker_fill = "#fca5a5" if is_active else "#991b1b"
            
            radius = 6 if is_active else 4
            outline = "white" if is_active else ""
            self.canvas.create_oval(cur_x - radius, cur_y - radius, cur_x + radius, cur_y + radius, fill=marker_fill, outline=outline, width=2)

        # If active drone has no GPS, draw a marker at 0,0 and a message
        if no_gps:
                # compute marker position at the default center
                zero_px, zero_py = lonlat_to_pixels(center_lon, center_lat, zoom)
                zx = zero_px - center_px + width / 2
                zy = zero_py - center_py + height / 2
                self.canvas.create_oval(zx - 6, zy - 6, zx + 6, zy + 6, fill="#f97316", outline="")
                self.canvas.create_text(width / 2, height / 2 + 20, text="No GPS — Waiting for fix", fill="#f97316", font=("Arial", 11, "bold"))

        # Overlay current coordinates and zoom level
        cur_lat, cur_lon = (raw_points[-1] if raw_points else (0.0, 0.0))
        status_text = f"Lat: {cur_lat:.6f}  Lon: {cur_lon:.6f}  Zoom: {zoom}"
        self.canvas.create_text(width - 10, 10, anchor="ne", text=status_text, fill="#cbd5e1", font=("Arial", 10))

        # Draw legend text
        self.canvas.create_rectangle(8, 8, 160, 46, fill="#021218", outline="")
        self.canvas.create_text(18, 18, anchor="nw", text=f"Showing: {active.name}", fill="#cbd5e1", font=("Arial", 10, "bold"))
        self.canvas.create_text(18, 32, anchor="nw", text=f"Click map to toggle", fill="#9ca3af", font=("Arial", 9))

    def _fetch_tile(self, z, x, y):
        # Return a PhotoImage for tile (z,x,y) or None on error
        key = (z, x, y)
        # If we already have a PhotoImage, return it
        if key in self.tile_cache:
            return self.tile_cache[key]

        # If raw data already fetched, create PhotoImage here (main thread)
        if key in self.tile_data and self.tile_data[key] is not None:
            try:
                img = Image.open(io.BytesIO(self.tile_data[key])).convert("RGBA")
                photo = ImageTk.PhotoImage(img)
                self.tile_cache[key] = photo
                return photo
            except Exception:
                # on failure, drop through to schedule a re-fetch
                self.tile_data.pop(key, None)

        # If a background fetch is not already scheduled for this tile, schedule it
        if key not in self.tile_data:
            self.tile_data[key] = None  # mark as scheduled
            self.tile_executor.submit(self._background_fetch_tile, z, x, y)

        return None

    def _background_fetch_tile(self, z, x, y):
        key = (z, x, y)
        url = self.tile_server_url.format(z=z, x=x, y=y)
        filename = f"{z}_{x}_{y}.img"
        filepath = os.path.join(self.cache_dir, filename)

        try:
            # Check disk cache first
            if os.path.exists(filepath):
                with open(filepath, "rb") as f:
                    data = f.read()
                self.tile_data[key] = data
                self.after(0, self._draw)
                return

            # Use a browser-like User-Agent to avoid blocking
            req = urllib.request.Request(url, headers={"User-Agent": "Mozilla/5.0 (X11; Linux x86_64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/122.0.0.0 Safari/537.36"})
            with urllib.request.urlopen(req, timeout=6) as resp:
                data = resp.read()
            
            # Save to disk cache
            with open(filepath, "wb") as f:
                f.write(data)

            # store raw bytes (thread-safe enough for our usage)
            self.tile_data[key] = data
            # schedule a redraw on the main thread to create PhotoImage and display
            try:
                self.after(0, self._draw)
            except Exception:
                pass
        except Exception as e:
            print(f"[MapView] Tile fetch error for {url}: {e}")
            # Remove from pending so it can be retried next draw cycle
            self.tile_data.pop(key, None)
            return

    # ----------------
    # Simulation helpers
    # ----------------
    def _start_simulation(self):
        # Populate raw_paths with a simple circle path near a real-world location (San Francisco)
        try:
            center = (37.7749, -122.4194)
            pts = []
            for i in range(60):
                ang = (i / 60.0) * 2 * math.pi
                lat = center[0] + 0.0015 * math.sin(ang)
                lon = center[1] + 0.0015 * math.cos(ang)
                pts.append((lat, lon))
            # assign to both drones for visible movement
            self.raw_paths[DroneName.Scanner] = pts[:]
            self.raw_paths[DroneName.Sprayer] = [(p[0] + 0.0005, p[1] + 0.0005) for p in pts]
            # start animating by scheduling periodic small rotations
            self._sim_index = 0
            self._simulate_step()
        except Exception:
            traceback.print_exc()

    def _simulate_step(self):
        try:
            # rotate lists so current point advances
            for dn in (DroneName.Scanner, DroneName.Sprayer):
                lst = self.raw_paths.get(dn, [])
                if lst:
                    self.raw_paths[dn] = lst[1:] + lst[:1]
            self._draw()
            self.after(300, self._simulate_step)
        except Exception:
            traceback.print_exc()
