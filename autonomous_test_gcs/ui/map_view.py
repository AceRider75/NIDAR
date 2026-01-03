import customtkinter as ctk

#Adds the map widget (self explanatory)
#Need to make this actually work - rn  it's just a placeholder

class MapView(ctk.CTkFrame):
    def __init__(self, master, **kwargs):
        super().__init__(master, **kwargs)
        self.configure(fg_color="green")
        ctk.CTkLabel(self, text="Map", font=("Arial", 18, "bold"), text_color="white").pack(expand=True)
