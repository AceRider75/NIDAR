import customtkinter as ctk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg 
from matplotlib.figure import Figure

class GraphPanel(ctk.CTkFrame):
    def __init__(self, master: ctk.CTk, title: str, label1: str, label2: str, label3: str, **kwargs):
        super().__init__(master, **kwargs)

        ctk.CTkLabel(self, text=title, font=("Arial", 12, "bold")).pack(anchor="w", padx=5)

        self.fig = Figure(figsize=(4, 2), dpi=80)
        self.ax = self.fig.add_subplot(111)

        # dark theme
        self.fig.patch.set_facecolor("#1e1e1e")
        self.ax.set_facecolor("#1e1e1e")
        self.ax.tick_params(colors="white")
        self.ax.xaxis.label.set_color("white")
        self.ax.yaxis.label.set_color("white")
        for spine in self.ax.spines.values():
            spine.set_color("white")

        self.ax.set_xlabel("Time (s)")
        self.ax.set_ylabel("Value")

        # buffers
        self.t_data = []
        self.d1 = []
        self.d2 = []
        self.d3 = []

        # 3 colored lines
        self.line1, = self.ax.plot([], [], color="cyan", label=label1)
        self.line2, = self.ax.plot([], [], color="magenta", label=label2)
        self.line3, = self.ax.plot([], [], color="yellow", label=label3)

        self.ax.legend(facecolor="#1e1e1e", labelcolor="white")

        self.canvas = FigureCanvasTkAgg(self.fig, master=self)
        self.canvas.get_tk_widget().pack(fill="both", expand=True, padx=5, pady=5)

    def update_graph(self, t: float, v1: float, v2: float, v3: float) -> None:
        self.t_data.append(t)
        self.d1.append(v1)
        self.d2.append(v2)
        self.d3.append(v3)

        # rolling window of 300 samples
        if len(self.t_data) > 300:
            self.t_data = self.t_data[-300:]
            self.d1 = self.d1[-300:]
            self.d2 = self.d2[-300:]
            self.d3 = self.d3[-300:]

        # update lines
        self.line1.set_data(self.t_data, self.d1)
        self.line2.set_data(self.t_data, self.d2)
        self.line3.set_data(self.t_data, self.d3)

        self.ax.relim()
        self.ax.autoscale_view()
        self.canvas.draw()