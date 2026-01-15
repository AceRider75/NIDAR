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
    WAYPOINT_UPDATE_INTERVAL_MS = 1000  # Check for new yellow spots every 1 second

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
        self.after(self.WAYPOINT_UPDATE_INTERVAL_MS, self._update_waypoints_from_spots)

        self.last_sprayer_log_str = ""
        self.last_scanner_log_str = ""
        self._last_spot_count = 0
        self._last_coord_count = 0


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
            
            scanner_log_raw = scanner.get("log", "")
            new_scanner_logs = self._process_new_logs(scanner_log_raw, self.last_scanner_log_str)
            if new_scanner_logs:
                self.dashboard.scanner_logs.append_text(new_scanner_logs)
                self.last_scanner_log_str = scanner_log_raw

            self.dashboard.sprayer_panel.update_status(sprayer["status"], sprayer["battery"])   #Update Battery and Status 
            self.dashboard.sprayer_panel.update_telemetry(sprayer["ui_telemetry"])              #Update Telemetry
            
            sprayer_log_raw = sprayer.get("log", "")
            new_sprayer_logs = self._process_new_logs(sprayer_log_raw, self.last_sprayer_log_str)
            if new_sprayer_logs:
                self.dashboard.sprayer_logs.append_text(new_sprayer_logs)
                self.last_sprayer_log_str = sprayer_log_raw

        except Exception as e:
            print("UI update error:", e)

        self.after(self.UPDATE_INTERVAL_MS, self._periodic_update)    

    def _process_new_logs(self, new_log_str: str, last_log_str: str) -> str:
        """
        Compare new log string (which may contain overlap) with last received log string.
        Return only the new lines.
        """
        if not new_log_str:
            return ""
        if not last_log_str:
            return new_log_str
            
        new_lines = new_log_str.split('\n')
        last_lines = last_log_str.split('\n')
        
        n_new = len(new_lines)
        n_last = len(last_lines)
        
        # Find longest suffix of last_lines that matches prefix of new_lines
        for k in range(min(n_new, n_last), 0, -1):
            if last_lines[-k:] == new_lines[:k]:
                return "\n".join(new_lines[k:])
        
        # If no overlap found, assume all new (or gap in data)
        return new_log_str

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

    def _update_waypoints_from_spots(self) -> None:
        """Periodically check for new yellow spots and update waypoint display."""
        try:
            if hasattr(self.dashboard, 'waypoint_manager'):
                # Get yellow spots from scanner
                spots_data = self.controller.get_yellow_spots(DroneName.Scanner)
                
                if spots_data:
                    current_spot_count = len(spots_data)
                    current_coord_count = sum(len(coords) for coords in spots_data.values())
                    
                    # Only update if spots changed
                    if (current_spot_count != self._last_spot_count or 
                        current_coord_count != self._last_coord_count):
                        
                        self.dashboard.waypoint_manager.update_from_spots(spots_data)
                        self._last_spot_count = current_spot_count
                        self._last_coord_count = current_coord_count
                        
        except Exception as e:
            print(f"[UI] Waypoint update error: {e}")
        
        self.after(self.WAYPOINT_UPDATE_INTERVAL_MS, self._update_waypoints_from_spots)

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
            # Check if there are waypoints to transfer (from scanner or imported)
            centers = self.controller.get_spot_centers(DroneName.Scanner)
            imported_waypoints = getattr(self.controller, 'sprayer_waypoints', [])
            
            total_waypoints = len(centers) + len(imported_waypoints)
            print(f"[UI] Transfer clicked - Found {len(centers)} spot centers, {len(imported_waypoints)} imported waypoints")
            
            if not centers and not imported_waypoints:
                self.show_message("Warning", "No waypoints to transfer.\n\nEither:\n- Wait for Scanner drone to detect spots, or\n- Import waypoints from CSV file", "warning")
                return
            
            # Check if sprayer radio is connected
            sprayer_radio = self.controller.drone_states[DroneName.Sprayer.value].radio
            if not sprayer_radio.serial or not sprayer_radio.serial.is_open:
                self.show_message("Error", "Sprayer radio not connected. Check USB connection.", "error")
                return
            
            source = "spots" if centers else "imported CSV"
            count = len(centers) if centers else len(imported_waypoints)
            print(f"[UI] Transferring {count} waypoints from {source} to Sprayer...")
            
            # Transfer waypoints (function now checks both sources)
            success = self.controller.transfer_waypoints_to_sprayer()
            
            if success:
                self.show_message("Success", f"Transferred {count} waypoints to Sprayer drone\n(Source: {source})", "success")
            else:
                self.show_message("Error", "Transfer failed - check logs for details", "error")
                
        except Exception as e:
            import traceback
            traceback.print_exc()
            self.show_message("Error", f"Failed to transfer waypoints: {e}", "error")


def run_app() -> None:
    app = GCSUI()
    app.mainloop()

if __name__ == "__main__":
    run_app()
