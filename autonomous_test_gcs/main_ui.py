import customtkinter as ctk
import math
import time
from core.gcs_controller import GCSController
from core.drone_state import DroneName
from ui.dashboard import Dashboard          #Handles the main dashboard UI layout (basically positions the widgets)

#Changes needed in map_view ui directory

class GCSUI(ctk.CTk):
    UPDATE_INTERVAL_MS = 500
    GRAPH_UPDATE_INTERVAL_MS = 100
    WAYPOINT_UPDATE_INTERVAL_MS = 2000  # Check for new waypoints every 2 seconds

    def __init__(self):
        super().__init__()

        ctk.set_appearance_mode("dark")
        ctk.set_default_color_theme("blue")

        self.title("GCS - Crop Scanner & Sprayer")
        self.geometry("1400x800")

        self.controller = GCSController()

        # Dashboard
        self.dashboard = Dashboard(self, controller=self.controller)
        self.dashboard.pack(fill="both", expand=True)

        self.after(self.UPDATE_INTERVAL_MS, self._periodic_update)   
        self.after(self.GRAPH_UPDATE_INTERVAL_MS, self._update_graphs)
        self.after(self.WAYPOINT_UPDATE_INTERVAL_MS, self._update_waypoints)

        self.last_sprayer_log = None
        self._last_waypoint_count = 0


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

            # Position graphs removed - waypoint manager now in that space

        except Exception as e:
            print(f"[UI] UI Update Error: {e} \t {DroneName.Sprayer}")
        
        self.after(self.GRAPH_UPDATE_INTERVAL_MS, self._update_graphs)  

    def _update_waypoints(self) -> None:
        """Periodically check for new waypoints and update display."""
        try:
            if hasattr(self.dashboard, 'waypoint_manager'):
                current_waypoints = self.controller.get_waypoints()
                if len(current_waypoints) != self._last_waypoint_count:
                    self.dashboard.waypoint_manager.set_waypoints(current_waypoints)
                    self._last_waypoint_count = len(current_waypoints)
        except Exception as e:
            print(f"[UI] Waypoint update error: {e}")
        
        self.after(self.WAYPOINT_UPDATE_INTERVAL_MS, self._update_waypoints)

    def show_message(self, title: str, message: str, msg_type: str = "info"):
        """Show a message dialog."""
        from tkinter import messagebox
        if msg_type == "success" or msg_type == "info":
            messagebox.showinfo(title, message)
        elif msg_type == "error":
            messagebox.showerror(title, message)
        elif msg_type == "warning":
            messagebox.showwarning(title, message)

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
