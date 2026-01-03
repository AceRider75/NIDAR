import customtkinter as ctk
from core.gcs_controller import GCSController
from ui.map_view import MapView                 #For the map (Needs to be fixed)
from ui.telemetry_view import TelemetryView     #For the logs (sorry for calling it telemetry)
from ui.graph_widgets import GraphPanel         #For the graphs (Needs to be fixed)
from ui.drone_panel import DronePanel           #For the drone panels (Telem, Battery, Status)

class Dashboard(ctk.CTkFrame):
    def __init__(self, master: ctk.CTk, controller: GCSController, **kwargs):
        super().__init__(master, **kwargs)
        self.controller = controller

        # Configure grid
        self.grid_rowconfigure(0, weight=2)         #Map + Baro/Pos Graphs
        self.grid_rowconfigure(1, weight=2)         #Map + Roll/Pitch/Yaw Graphs
        self.grid_rowconfigure(2, weight=2)         #Logs + Drone Panels
        self.grid_columnconfigure((0, 1, 2), weight=1, uniform="col")
    
        # Map
        self.map_view = MapView(self)
        self.map_view.grid(row=0, column=0, rowspan=2, sticky="nsew", padx=5, pady=5)

        # Graph panels
        self.scanner_xyz_graph = GraphPanel(self, "Scanner Position vs Time", "x", "y", "z")
        self.sprayer_xyz_graph = GraphPanel(self, "Sprayer Position vs Time", "x", "y", "z")
        self.scanner_rpy_graph = GraphPanel(self, "Scanner Roll, Pitch, Yaw vs Time", "roll", "pitch", "yaw")
        self.sprayer_rpy_graph = GraphPanel(self, "Sprayer Roll, Pitch, Yaw vs Time", "roll", "pitch", "yaw")

        self.scanner_xyz_graph.grid(row=0, column=1, sticky="nsew", padx=5, pady=5)
        self.sprayer_xyz_graph.grid(row=0, column=2, sticky="nsew", padx=5, pady=5)
        self.scanner_rpy_graph.grid(row=1, column=1, sticky="nsew", padx=5, pady=5)
        self.sprayer_rpy_graph.grid(row=1, column=2, sticky="nsew", padx=5, pady=5)

        # Lower section (logs + control panels)
        self.logs_frame = ctk.CTkFrame(self)
        self.logs_frame.grid(row=2, column=0, columnspan=3, sticky="nsew", padx=5, pady=5)
        self.logs_frame.grid_columnconfigure(0, weight=5)   # Logs column
        self.logs_frame.grid_columnconfigure(1, weight=1)   # Sprayer drone
        self.logs_frame.grid_columnconfigure(2, weight=1)   # Scanner drone
        self.logs_frame.grid_rowconfigure(0, weight=1)
        self.logs_frame.grid_rowconfigure(1, weight=1)

        self.scanner_logs = TelemetryView(self.logs_frame, "Scanner Logs")
        self.sprayer_logs = TelemetryView(self.logs_frame, "Sprayer Logs")

        self.sprayer_panel = DronePanel(self.logs_frame, "Sprayer Drone", controller)
        self.scanner_panel = DronePanel(self.logs_frame, "Scanner Drone", controller)

        self.scanner_logs.grid(row=0, column=0, sticky="nsew", padx=5, pady=5)
        self.sprayer_logs.grid(row=1, column=0, sticky="nsew", padx=5, pady=5)
        self.sprayer_panel.grid(row=0, column=2, rowspan=2, sticky="nsew", padx=5, pady=5)
        self.scanner_panel.grid(row=0, column=1, rowspan=2, sticky="nsew", padx=5, pady=5)
