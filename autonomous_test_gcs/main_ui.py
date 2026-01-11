import customtkinter as ctk
import math
import time
from core.gcs_controller import GCSController
from core.drone_state import DroneName
from ui.dashboard import Dashboard          #Handles the main dashboard UI layout (basically positions the widgets)

#Changes needed in map_view ui directory

class GCSUI(ctk.CTk):
    UPDATE_INTERVAL_MS = 500    #Interval after which logs and telemetry are updated
    GRAPH_UPDATE_INTERVAL_MS = 100  #Interval after which graphs are updated

    def __init__(self):
        super().__init__()

        ctk.set_appearance_mode("dark")         #If you make it light, I hate you
        ctk.set_default_color_theme("blue")     #Don't change this - it will look ugly

        self.title("GCS - Crop Scanner & Sprayer")
        self.geometry("1400x800")

        self.controller = GCSController()   # Initialize the GCS controller

        # Dashboard
        self.dashboard = Dashboard(self, controller=self.controller)
        self.dashboard.pack(fill="both", expand=True)

        self.after(self.UPDATE_INTERVAL_MS, self._periodic_update)   
        self.after(self.GRAPH_UPDATE_INTERVAL_MS, self._update_graphs)

        self.last_sprayer_log = None


    def _periodic_update(self) -> None:

        try:
            #Get drone state from the controller: (Controller handles all logic)
            scanner = self.controller.get_drone_state(DroneName.Scanner)      #dictionary
            sprayer = self.controller.get_drone_state(DroneName.Sprayer)      #dictionary
            '''
            State Format:
            "status": str,
            "battery": int,
            "raw_log": [],
            "telemetry": {
                "lat": float,
                "lon": float,
                "alt": float (in m),
                "vx": float (in m/s),
                "vy": float (in m/s),
                "vz": float (in m/s),
                "roll": float (in deg),
                "pitch": float (in deg),
                "yaw": float (in deg),
                "xacc": float (in m/s^2),
                "yacc": float (in m/s^2),
                
            ui_telemetry modifies telemetry to fit the desired ui format
            '''

            # Update drone panels
            self.dashboard.scanner_panel.update_status(scanner["status"], scanner["battery"])   #Update Battery and Status 
            self.dashboard.scanner_panel.update_telemetry(scanner["ui_telemetry"])              #Update Telemetry
            #self.dashboard.scanner_logs.append_text(scanner["raw_latest"])                      #Update Logs --> To be changed

            self.dashboard.sprayer_panel.update_status(sprayer["status"], sprayer["battery"])   #Update Battery and Status 
            self.dashboard.sprayer_panel.update_telemetry(sprayer["ui_telemetry"])              #Update Telemetry
            #self.dashboard.sprayer_logs.append_text(sprayer["raw_latest"])  

            #current_event = sprayer["log"]

            # if current_event and current_event != self.last_sprayer_log:
            #     self.dashboard.sprayer_logs.append_text(current_event)
            #     self.last_sprayer_log = current_event


        except Exception as e:
            print("UI update error:", e)

        self.after(self.UPDATE_INTERVAL_MS, self._periodic_update)    

    def _update_graphs(self) -> None:
        try:
            scanner = self.controller.get_drone_state(DroneName.Scanner)      #dictionary
            sprayer = self.controller.get_drone_state(DroneName.Sprayer)      #dictionary

            if not hasattr(self, "t0"):
                self.t0 = time.time()
            t = time.time() - self.t0

            sprayer_tele = sprayer["telemetry"]             #dictionary
            scanner_tele = scanner["telemetry"]

            # --- helpers to validate numbers ---
            def _num(v, default=None):
                # returns v if it's a real number, otherwise default
                return v if isinstance(v, (int, float)) else default

            # --- RPY graph ---
            sry = (_num(sprayer_tele.get("roll")),
                   _num(sprayer_tele.get("pitch")),
                   _num(sprayer_tele.get("yaw")))
            if all(v is not None for v in sry):
                self.dashboard.sprayer_rpy_graph.update_graph(t, *sry)

            scy = (_num(scanner_tele.get("roll")),
                   _num(scanner_tele.get("pitch")),
                   _num(scanner_tele.get("yaw")))
            if all(v is not None for v in scy):
                self.dashboard.scanner_rpy_graph.update_graph(t, *scy)

            # --- XYZ (position) graph ---
            meters_per_deg_lat = 111320

            # Sprayer position (guard None for lat/lon and lat0/lon0)
            spr_lat = _num(sprayer_tele.get("lat"))
            spr_lon = _num(sprayer_tele.get("lon"))
            spr_alt = _num(sprayer_tele.get("alt"))
            spr_lat0 = _num(getattr(self.controller.drone_states[DroneName.Sprayer.value], "lat0", None))
            spr_lon0 = _num(getattr(self.controller.drone_states[DroneName.Sprayer.value], "lon0", None))
            spr_started = getattr(self.controller.drone_states[DroneName.Sprayer.value], "started", False)

            if spr_started and None not in (spr_lat, spr_lon, spr_alt, spr_lat0, spr_lon0):
                spr_meters_per_deg_lon = 111320 * math.cos(math.radians(spr_lat0))
                spr_x = (spr_lat - spr_lat0) * meters_per_deg_lat
                spr_y = (spr_lon - spr_lon0) * spr_meters_per_deg_lon
                spr_z = spr_alt
                self.dashboard.sprayer_xyz_graph.update_graph(t, spr_x, spr_y, spr_z)

            # Scanner position
            scn_lat = _num(scanner_tele.get("lat"))
            scn_lon = _num(scanner_tele.get("lon"))
            scn_alt = _num(scanner_tele.get("alt"))
            scn_lat0 = _num(getattr(self.controller.drone_states[DroneName.Scanner.value], "lat0", None))
            scn_lon0 = _num(getattr(self.controller.drone_states[DroneName.Scanner.value], "lon0", None))
            scn_started = getattr(self.controller.drone_states[DroneName.Scanner.value], "started", False)

            if scn_started and None not in (scn_lat, scn_lon, scn_alt, scn_lat0, scn_lon0):
                scn_meters_per_deg_lon = 111320 * math.cos(math.radians(scn_lat0))
                scn_x = (scn_lat - scn_lat0) * meters_per_deg_lat
                scn_y = (scn_lon - scn_lon0) * scn_meters_per_deg_lon
                scn_z = scn_alt
                self.dashboard.scanner_xyz_graph.update_graph(t, scn_x, scn_y, scn_z)
        except Exception as e:
            print(f"[UI] UI Update Error: {e} \t {DroneName.Sprayer}")
        
        self.after(self.GRAPH_UPDATE_INTERVAL_MS, self._update_graphs)  

    def on_transfer_waypoints_clicked(self):
        """Handle Transfer Waypoints button click."""
        try:
            # Option 1: Simple transfer with default settings
            success = self.controller.transfer_waypoints_to_sprayer()
            
            # Option 2: Transfer with custom altitude (if you have an input field)
            # altitude = float(self.altitude_input.text() or "3.0")
            # success = self.gcs_controller.transfer_waypoints_detailed(
            #     altitude=altitude,
            #     use_weighted_centers=True
            # )
            
            if success:
                self.show_message("Success", "Waypoints transferred to Sprayer drone", "success")
            else:
                self.show_message("Error", "No waypoints to transfer or transfer failed", "error")
                
        except Exception as e:
            self.show_message("Error", f"Failed to transfer waypoints: {e}", "error")


def run_app() -> None:
    app = GCSUI()
    app.mainloop()

if __name__ == "__main__":
    run_app()
