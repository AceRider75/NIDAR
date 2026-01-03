import customtkinter as ctk
from core.gcs_controller import GCSController
from core.drone_state import DroneName

#Handles both the scanner and sprayer panels (Each panel is an object of the DronePanel class)

class DronePanel(ctk.CTkFrame):
    def __init__(self, master: ctk.CTkFrame, title: str, controller: GCSController, **kwargs):
        super().__init__(master, **kwargs)
        self.controller = controller
        self.title = title
        self.configure(corner_radius=10)

        #Title
        ctk.CTkLabel(self, text=title, font=("Arial", 16, "bold")).pack(pady=4)

        #Status --> I wanna make the status change color based on what it is - like green for active and red for land
        self.status_label = ctk.CTkLabel(self, text="Status: Idle")
        self.status_label.pack()

        #Battery UI:
        self.battery_label = ctk.CTkLabel(self, text="Battery: 100%")
        self.battery_label.pack()

        self.battery_bar = ctk.CTkProgressBar(self, width=150, progress_color = "green")
        self.battery_bar.set(1)
        self.battery_bar.pack(pady=5)

        # Buttons:
        btn_frame = ctk.CTkFrame(self, fg_color="transparent")
        btn_frame.pack(pady=5)

        ctk.CTkButton(btn_frame, text="Start", fg_color="green", command=self.start_mission).grid(row=0, column=0, padx=3)
        ctk.CTkButton(btn_frame, text="RTL", fg_color="yellow", text_color="black", command=self.rtl).grid(row=0, column=1, padx=3)
        ctk.CTkButton(btn_frame, text="Land", fg_color="red", command=self.land).grid(row=0, column=2, padx=3)

        #Telemetry Boxes:
        self.telemetry_box = ctk.CTkTextbox(self, height=100, width=220)
        self.telemetry_box.insert("end", "Telemetry\n")
        self.telemetry_box.pack(padx=5, pady=5, fill="both", expand=True)
        self.telemetry_box.configure(state = "disabled")

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

    def update_status(self, status: str, battery: int) -> None:
        self.status_label.configure(text=f"Status: {status}")
        self.battery_label.configure(text=f"Battery: {battery}%")
        self.battery_bar.set(battery / 100)
        if(battery <= 25):
            self.battery_bar.configure(progress_color = "red")
        else:
            self.battery_bar.configure(progress_color = "green") 
        
    def update_telemetry(self, text: str) -> None:
        self.telemetry_box.configure(state="normal")
        self.telemetry_box.delete("0.0", "end")
        self.telemetry_box.insert("end", text)
        self.telemetry_box.configure(state="disabled")