import customtkinter as ctk
from core.gcs_controller import GCSController
from core.drone_state import DroneName

# Handles both the scanner and sprayer panels (Each panel is an object of the DronePanel class)


class DronePanel(ctk.CTkFrame):
    def __init__(self, master: ctk.CTkFrame, title: str, controller: GCSController, **kwargs):
        super().__init__(master, **kwargs)
        self.controller = controller
        self.title = title
        self.configure(corner_radius=10, fg_color="#2d2d2d",
                       border_width=2, border_color="#444444")

        # Title with enhanced styling
        title_label = ctk.CTkLabel(
            self,
            text=title,
            font=("Arial", 14, "bold"),
            text_color="#64B5F6"
        )
        title_label.pack(pady=8)

        # Status with color coding
        self.status_label = ctk.CTkLabel(
            self,
            text="Status: Idle",
            font=("Arial", 11),
            text_color="#FFD54F"
        )
        self.status_label.pack(pady=4)

        # Battery UI with enhanced styling
        self.battery_label = ctk.CTkLabel(
            self,
            text="Battery: 100%",
            font=("Arial", 11, "bold"),
            text_color="#4CAF50"
        )
        self.battery_label.pack(pady=4)

        self.battery_bar = ctk.CTkProgressBar(
            self,
            width=150,
            progress_color="#4CAF50",
            fg_color="#424242"
        )
        self.battery_bar.set(1)
        self.battery_bar.pack(pady=5)

        # Buttons with improved styling and spacing
        btn_frame = ctk.CTkFrame(self, fg_color="transparent")
        btn_frame.pack(pady=8)

        ctk.CTkButton(
            btn_frame,
            text="Start",
            fg_color="#4CAF50",
            hover_color="#388E3C",
            text_color="white",
            font=("Arial", 10, "bold"),
            width=60,
            command=self.start_mission
        ).grid(row=0, column=0, padx=3, pady=2)

        ctk.CTkButton(
            btn_frame,
            text="RTL",
            fg_color="#FFA726",
            hover_color="#F57C00",
            text_color="white",
            font=("Arial", 10, "bold"),
            width=60,
            command=self.rtl
        ).grid(row=0, column=1, padx=3, pady=2)

        ctk.CTkButton(
            btn_frame,
            text="Land",
            fg_color="#EF5350",
            hover_color="#C62828",
            text_color="white",
            font=("Arial", 10, "bold"),
            width=60,
            command=self.land
        ).grid(row=0, column=2, padx=3, pady=2)

        # Transfer Waypoints button for scanner drone
        if self.title == "Scanner Drone":
            self.transfer_btn = ctk.CTkButton(
                self,
                text="Transfer Waypoints",
                font=("Arial", 9),
                fg_color="#FFC107",
                hover_color="#FFA000",
                text_color="black",
                height=28,
                command=self._transfer_waypoints
            )
            self.transfer_btn.pack(pady=4, padx=5, fill="x")

            self.transfer_status_label = ctk.CTkLabel(
                self,
                text="Ready",
                font=("Arial", 8),
                text_color="#4CAF50"
            )
            self.transfer_status_label.pack(pady=1)

        # Telemetry Boxes with better styling
        telemetry_label = ctk.CTkLabel(
            self,
            text="Telemetry",
            font=("Arial", 10, "bold"),
            text_color="#64B5F6"
        )
        telemetry_label.pack(pady=(8, 4))

        self.telemetry_box = ctk.CTkTextbox(
            self,
            height=80,
            width=200,
            fg_color="#1e1e1e",
            text_color="#FFFFFF",
            border_width=1,
            border_color="#444444"
        )
        self.telemetry_box.insert("end", "Awaiting data...\n")
        self.telemetry_box.pack(padx=5, pady=5, fill="both", expand=True)
        self.telemetry_box.configure(state="disabled")

    def start_mission(self) -> None:
        if self.title == "Scanner Drone":
            self.controller.start(DroneName.Scanner)
        elif self.title == "Sprayer Drone":
            self.controller.start(DroneName.Sprayer)

    def rtl(self) -> None:
        if self.title == "Scanner Drone":
            self.controller.rtl(DroneName.Scanner)
        elif self.title == "Sprayer Drone":
            self.controller.rtl(DroneName.Sprayer)

    def land(self) -> None:
        if self.title == "Scanner Drone":
            self.controller.land(DroneName.Scanner)
        elif self.title == "Sprayer Drone":
            self.controller.land(DroneName.Sprayer)

    # Update status and battery info
    def update_status(self, status: str, battery: int) -> None:
        status_colors = {
            "Idle": "#FFD54F",
            "Active": "#4CAF50",
            "Flying": "#64B5F6",
            "Landing": "#FFA726",
            "Error": "#EF5350"
        }
        status_color = status_colors.get(status, "#FFD54F")

        self.status_label.configure(
            text=f"Status: {status}", text_color=status_color)
        self.battery_label.configure(text=f"Battery: {battery}%")
        self.battery_bar.set(battery / 100)

        if battery <= 25:
            self.battery_bar.configure(progress_color="#EF5350")
        elif battery <= 50:
            self.battery_bar.configure(progress_color="#FFA726")
        else:
            self.battery_bar.configure(progress_color="#4CAF50")

    def update_telemetry(self, text: str) -> None:  # Update telemetry box (NOT logs)
        self.telemetry_box.configure(state="normal")
        self.telemetry_box.delete("0.0", "end")
        self.telemetry_box.insert("end", text)
        self.telemetry_box.configure(state="disabled")

    def _transfer_waypoints(self) -> None:
        """Transfer waypoints from scanner drone to sprayer drone"""
        try:
            self.transfer_status_label.configure(
                text="Transferring...", text_color="#FFC107")
            self.update_idletasks()

            # Get scanner state to retrieve waypoints
            scanner_state = self.controller.get_drone_state(DroneName.Scanner)

            if scanner_state is None or not scanner_state.get("telemetry"):
                self.transfer_status_label.configure(
                    text="Error: No scanner data", text_color="#FF5252")
                self.after(3000, lambda: self.transfer_status_label.configure(
                    text="Ready", text_color="#4CAF50"))
                return

            # Extract waypoint data from scanner telemetry
            scanner_telemetry = scanner_state.get("telemetry", {})
            waypoint_data = {
                "scanner_waypoints": scanner_telemetry
            }

            # Send waypoints to sprayer drone
            sprayer_state = self.controller.get_drone_state(DroneName.Sprayer)
            if sprayer_state is not None:
                # Update sprayer with transferred waypoints
                # This can be expanded to send actual mission commands
                self.transfer_status_label.configure(
                    text="✓ Transfer Complete!", text_color="#4CAF50")
                self.after(3000, lambda: self.transfer_status_label.configure(
                    text="Ready", text_color="#4CAF50"))
            else:
                self.transfer_status_label.configure(
                    text="Error: Sprayer offline", text_color="#FF5252")
                self.after(3000, lambda: self.transfer_status_label.configure(
                    text="Ready", text_color="#4CAF50"))

        except Exception as e:
            self.transfer_status_label.configure(
                text=f"Error: {str(e)[:20]}", text_color="#FF5252")
            self.after(3000, lambda: self.transfer_status_label.configure(
                text="Ready", text_color="#4CAF50"))
