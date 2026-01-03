import customtkinter as ctk

#Handles the log text boxes

class TelemetryView(ctk.CTkFrame):
    def __init__(self, master: ctk.CTkFrame, title: str, **kwargs):
        super().__init__(master, **kwargs)
        ctk.CTkLabel(self, text=title, font=("Arial", 14, "bold")).pack(anchor="w", padx=5, pady=2)
        self.textbox = ctk.CTkTextbox(self, wrap="word")
        self.textbox.pack(fill="both", expand=True, padx=5, pady=5)
        self.textbox.configure(state = "disabled")

    def append_text(self, text: str) -> None:
        if(text != ""):
            self.textbox.configure(state="normal")
            self.textbox.insert("end", text + "\n")
            self.textbox.configure(state="disabled")
            self.textbox.see("end")