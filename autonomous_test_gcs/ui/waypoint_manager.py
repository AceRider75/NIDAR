import customtkinter as ctk
from tkinter import filedialog, messagebox
import csv
import os
from typing import List, Dict, Optional


class WaypointManager(ctk.CTkFrame):
    """Widget for displaying and managing waypoints with import/export functionality."""
    
    def __init__(self, parent, controller, **kwargs):
        super().__init__(parent, **kwargs)
        self.controller = controller
        self.waypoints: List[Dict] = []
        
        self._setup_ui()
    
    def _setup_ui(self):
        """Setup the waypoint manager UI."""
        # Title
        title_label = ctk.CTkLabel(
            self, 
            text="Waypoint Manager", 
            font=ctk.CTkFont(size=16, weight="bold")
        )
        title_label.pack(pady=(10, 5))
        
        # Button frame
        button_frame = ctk.CTkFrame(self, fg_color="transparent")
        button_frame.pack(fill="x", padx=10, pady=5)
        
        # Import button
        self.import_btn = ctk.CTkButton(
            button_frame,
            text="📂 Import CSV",
            command=self._import_waypoints,
            width=120,
            height=32
        )
        self.import_btn.pack(side="left", padx=5)
        
        # Export button
        self.export_btn = ctk.CTkButton(
            button_frame,
            text="💾 Export CSV",
            command=self._export_waypoints,
            width=120,
            height=32
        )
        self.export_btn.pack(side="left", padx=5)
        
        # Clear button
        self.clear_btn = ctk.CTkButton(
            button_frame,
            text="🗑️ Clear",
            command=self._clear_waypoints,
            width=80,
            height=32,
            fg_color="#dc3545",
            hover_color="#c82333"
        )
        self.clear_btn.pack(side="left", padx=5)
        
        # Refresh button
        self.refresh_btn = ctk.CTkButton(
            button_frame,
            text="🔄 Refresh",
            command=self._refresh_waypoints,
            width=100,
            height=32,
            fg_color="#28a745",
            hover_color="#218838"
        )
        self.refresh_btn.pack(side="left", padx=5)
        
        # Waypoint count label
        self.count_label = ctk.CTkLabel(
            self,
            text="Waypoints: 0",
            font=ctk.CTkFont(size=12)
        )
        self.count_label.pack(pady=5)
        
        # Waypoint display frame with scrollbar
        self.waypoint_frame = ctk.CTkScrollableFrame(
            self,
            height=200,
            label_text="Waypoints"
        )
        self.waypoint_frame.pack(fill="both", expand=True, padx=10, pady=5)
        
        # Header row
        self._create_header()
    
    def _create_header(self):
        """Create the header row for waypoint display."""
        header_frame = ctk.CTkFrame(self.waypoint_frame, fg_color="#2b2b2b")
        header_frame.pack(fill="x", pady=(0, 5))
        
        headers = ["#", "Latitude", "Longitude", "Altitude", "Type"]
        widths = [40, 120, 120, 80, 80]
        
        for header, width in zip(headers, widths):
            label = ctk.CTkLabel(
                header_frame,
                text=header,
                font=ctk.CTkFont(size=11, weight="bold"),
                width=width
            )
            label.pack(side="left", padx=2)
    
    def _import_waypoints(self):
        """Import waypoints from a CSV file."""
        filepath = filedialog.askopenfilename(
            title="Import Waypoints",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")],
            initialdir=os.path.expanduser("~")
        )
        
        if not filepath:
            return
        
        try:
            imported_waypoints = []
            with open(filepath, 'r', newline='') as csvfile:
                reader = csv.DictReader(csvfile)
                for row in reader:
                    waypoint = {
                        'lat': float(row.get('latitude', row.get('lat', 0))),
                        'lon': float(row.get('longitude', row.get('lon', 0))),
                        'alt': float(row.get('altitude', row.get('alt', 3.0))),
                        'type': row.get('type', 'waypoint')
                    }
                    imported_waypoints.append(waypoint)
            
            self.waypoints = imported_waypoints
            self._update_display()
            
            # Send waypoints to controller if available
            if hasattr(self.controller, 'set_waypoints'):
                self.controller.set_waypoints(imported_waypoints)
            
            messagebox.showinfo(
                "Import Successful",
                f"Imported {len(imported_waypoints)} waypoints from CSV."
            )
            
        except Exception as e:
            messagebox.showerror("Import Error", f"Failed to import waypoints:\n{e}")
    
    def _export_waypoints(self):
        """Export waypoints to a CSV file."""
        if not self.waypoints:
            # Try to get waypoints from controller
            self._refresh_waypoints()
        
        if not self.waypoints:
            messagebox.showwarning("No Waypoints", "No waypoints to export.")
            return
        
        filepath = filedialog.asksaveasfilename(
            title="Export Waypoints",
            defaultextension=".csv",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")],
            initialdir=os.path.expanduser("~"),
            initialfile="waypoints.csv"
        )
        
        if not filepath:
            return
        
        try:
            with open(filepath, 'w', newline='') as csvfile:
                fieldnames = ['index', 'latitude', 'longitude', 'altitude', 'type']
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                writer.writeheader()
                
                for i, wp in enumerate(self.waypoints):
                    writer.writerow({
                        'index': i + 1,
                        'latitude': wp.get('lat', 0),
                        'longitude': wp.get('lon', 0),
                        'altitude': wp.get('alt', 3.0),
                        'type': wp.get('type', 'waypoint')
                    })
            
            messagebox.showinfo(
                "Export Successful",
                f"Exported {len(self.waypoints)} waypoints to:\n{filepath}"
            )
            
        except Exception as e:
            messagebox.showerror("Export Error", f"Failed to export waypoints:\n{e}")
    
    def _clear_waypoints(self):
        """Clear all waypoints."""
        if self.waypoints:
            confirm = messagebox.askyesno(
                "Confirm Clear",
                "Are you sure you want to clear all waypoints?"
            )
            if confirm:
                self.waypoints = []
                self._update_display()
                
                if hasattr(self.controller, 'clear_waypoints'):
                    self.controller.clear_waypoints()
    
    def _refresh_waypoints(self):
        """Refresh waypoints from the controller."""
        try:
            # Try different methods to get waypoints from controller
            if hasattr(self.controller, 'get_waypoints'):
                self.waypoints = self.controller.get_waypoints() or []
            elif hasattr(self.controller, 'waypoints'):
                self.waypoints = self.controller.waypoints or []
            elif hasattr(self.controller, 'sprayer_waypoints'):
                self.waypoints = self.controller.sprayer_waypoints or []
            
            self._update_display()
            
        except Exception as e:
            print(f"Error refreshing waypoints: {e}")
    
    def set_waypoints(self, waypoints: List[Dict]):
        """Set waypoints externally and update display."""
        self.waypoints = waypoints or []
        self._update_display()
    
    def _update_display(self):
        """Update the waypoint display."""
        # Clear existing waypoint rows (keep header)
        for widget in self.waypoint_frame.winfo_children()[1:]:
            widget.destroy()
        
        # Update count
        self.count_label.configure(text=f"Waypoints: {len(self.waypoints)}")
        
        # Add waypoint rows
        for i, wp in enumerate(self.waypoints):
            self._add_waypoint_row(i + 1, wp)
    
    def _add_waypoint_row(self, index: int, waypoint: Dict):
        """Add a single waypoint row to the display."""
        row_color = "#1e1e1e" if index % 2 == 0 else "#2d2d2d"
        row_frame = ctk.CTkFrame(self.waypoint_frame, fg_color=row_color, height=28)
        row_frame.pack(fill="x", pady=1)
        row_frame.pack_propagate(False)
        
        # Index
        ctk.CTkLabel(
            row_frame,
            text=str(index),
            width=40,
            font=ctk.CTkFont(size=10)
        ).pack(side="left", padx=2)
        
        # Latitude
        ctk.CTkLabel(
            row_frame,
            text=f"{waypoint.get('lat', 0):.7f}",
            width=120,
            font=ctk.CTkFont(size=10)
        ).pack(side="left", padx=2)
        
        # Longitude
        ctk.CTkLabel(
            row_frame,
            text=f"{waypoint.get('lon', 0):.7f}",
            width=120,
            font=ctk.CTkFont(size=10)
        ).pack(side="left", padx=2)
        
        # Altitude
        ctk.CTkLabel(
            row_frame,
            text=f"{waypoint.get('alt', 0):.1f}m",
            width=80,
            font=ctk.CTkFont(size=10)
        ).pack(side="left", padx=2)
        
        # Type
        wp_type = waypoint.get('type', 'waypoint')
        type_color = "#28a745" if wp_type == "spray" else "#17a2b8"
        type_label = ctk.CTkLabel(
            row_frame,
            text=wp_type,
            width=80,
            font=ctk.CTkFont(size=10),
            text_color=type_color
        )
        type_label.pack(side="left", padx=2)
    
    def get_waypoints(self) -> List[Dict]:
        """Get current waypoints."""
        return self.waypoints