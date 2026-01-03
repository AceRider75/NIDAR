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

        self.controller = GCSController()

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

            current_event = sprayer["log"]

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

            sprayer_tele = sprayer["telemetry"]
            scanner_tele = scanner["telemetry"]

            # --- RPY graph ---
            sprayer_roll  = sprayer_tele["roll"]
            sprayer_pitch = sprayer_tele["pitch"]
            sprayer_yaw   = sprayer_tele["yaw"]
            self.dashboard.sprayer_rpy_graph.update_graph(t, sprayer_roll, sprayer_pitch, sprayer_yaw)

            scanner_roll  = scanner_tele["roll"]
            scanner_pitch = scanner_tele["pitch"]
            scanner_yaw   = scanner_tele["yaw"]
            self.dashboard.scanner_rpy_graph.update_graph(t, scanner_roll, scanner_pitch, scanner_yaw)           

            # --- XYZ (position) graph ---
            meters_per_deg_lat = 111320
            sprayer_meters_per_deg_lon = 111320 * math.cos(math.radians(self.controller.drone_states[DroneName.Sprayer.value].lat0))
            sprayer_lat = sprayer_tele["lat"] - self.controller.drone_states[DroneName.Sprayer.value].lat0
            sprayer_lon = sprayer_tele["lon"] - self.controller.drone_states[DroneName.Sprayer.value].lon0
            sprayer_x = sprayer_lat * meters_per_deg_lat
            sprayer_y = sprayer_lon * sprayer_meters_per_deg_lon
            sprayer_z = sprayer_tele["alt"]
            if self.controller.drone_states[DroneName.Sprayer.value].started:
                self.dashboard.sprayer_xyz_graph.update_graph(t, sprayer_x, sprayer_y, sprayer_z)

            meters_per_deg_lat = 111320
            scanner_meters_per_deg_lon = 111320 * math.cos(math.radians(self.controller.drone_states[DroneName.Scanner.value].lat0))
            scanner_lat = scanner_tele["lat"] - self.controller.drone_states[DroneName.Scanner.value].lat0
            scanner_lon = scanner_tele["lon"] - self.controller.drone_states[DroneName.Scanner.value].lon0
            scanner_x = scanner_lat * meters_per_deg_lat
            scanner_y = scanner_lon * scanner_meters_per_deg_lon
            scanner_z = scanner_tele["alt"]
            if self.controller.drone_states[DroneName.Scanner.value].started:
                self.dashboard.scanner_xyz_graph.update_graph(t, scanner_x, scanner_y, scanner_z)

        except Exception as e:
            print(f"[UI] UI Update Error: {e}")
        
        self.after(self.GRAPH_UPDATE_INTERVAL_MS, self._update_graphs)  


def run_app() -> None:
    app = GCSUI()
    app.mainloop()

if __name__ == "__main__":
    run_app()