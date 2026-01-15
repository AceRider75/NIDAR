import customtkinter as ctk
from tkinter import filedialog, messagebox
import csv
import os
from typing import List, Dict, Optional
from core.drone_state import DroneName

# Minimum detections required for a spot to be considered valid
MIN_DETECTION_COUNT = 2


class WaypointManager(ctk.CTkFrame):
    """Widget for displaying and managing waypoints from detected yellow spots."""
    
    def __init__(self, parent, controller, **kwargs):
        super().__init__(parent, **kwargs)
        self.controller = controller
        self.waypoints: List[Dict] = []  # All waypoints (including filtered out)
        self.valid_waypoints: List[Dict] = []  # Only waypoints with enough detections
        
        self._setup_ui()
    
    def _setup_ui(self):
        """Setup the waypoint manager UI."""
        # Configure grid
        self.grid_columnconfigure(0, weight=1)
        self.grid_rowconfigure(2, weight=1)
        
        # Title and count frame
        header_frame = ctk.CTkFrame(self, fg_color="transparent")
        header_frame.grid(row=0, column=0, sticky="ew", padx=5, pady=(5, 0))
        header_frame.grid_columnconfigure(1, weight=1)
        
        title_label = ctk.CTkLabel(
            header_frame, 
            text="Waypoint Manager", 
            font=ctk.CTkFont(size=14, weight="bold")
        )
        title_label.grid(row=0, column=0, sticky="w")
        
        # Waypoint count label
        self.count_label = ctk.CTkLabel(
            header_frame,
            text=f"Spots: 0 | Valid (≥{MIN_DETECTION_COUNT} detections): 0",
            font=ctk.CTkFont(size=11),
            text_color="#888888"
        )
        self.count_label.grid(row=0, column=1, sticky="e", padx=10)
        
        # Button frame
        button_frame = ctk.CTkFrame(self, fg_color="transparent")
        button_frame.grid(row=1, column=0, sticky="ew", padx=5, pady=5)
        
        # Import button
        self.import_btn = ctk.CTkButton(
            button_frame,
            text="📂 Import",
            command=self._import_waypoints,
            width=80,
            height=28,
            font=ctk.CTkFont(size=11)
        )
        self.import_btn.pack(side="left", padx=2)
        
        # Export button
        self.export_btn = ctk.CTkButton(
            button_frame,
            text="💾 Export",
            command=self._export_waypoints,
            width=80,
            height=28,
            font=ctk.CTkFont(size=11)
        )
        self.export_btn.pack(side="left", padx=2)
        
        # Clear button
        self.clear_btn = ctk.CTkButton(
            button_frame,
            text="🗑️ Clear",
            command=self._clear_waypoints,
            width=70,
            height=28,
            font=ctk.CTkFont(size=11),
            fg_color="#dc3545",
            hover_color="#c82333"
        )
        self.clear_btn.pack(side="left", padx=2)
        
        # Refresh button
        self.refresh_btn = ctk.CTkButton(
            button_frame,
            text="🔄 Refresh",
            command=self._refresh_from_spots,
            width=80,
            height=28,
            font=ctk.CTkFont(size=11),
            fg_color="#28a745",
            hover_color="#218838"
        )
        self.refresh_btn.pack(side="left", padx=2)
        
        # Transfer to Sprayer button
        self.transfer_btn = ctk.CTkButton(
            button_frame,
            text="📡 Transfer",
            command=self._transfer_to_sprayer,
            width=85,
            height=28,
            font=ctk.CTkFont(size=11),
            fg_color="#fd7e14",
            hover_color="#e76b00"
        )
        self.transfer_btn.pack(side="left", padx=2)
        
        # Waypoint display frame with scrollbar
        self.waypoint_frame = ctk.CTkScrollableFrame(
            self,
            label_text=f"Confirmed Yellow Spots (≥{MIN_DETECTION_COUNT} detections)",
            label_font=ctk.CTkFont(size=11)
        )
        self.waypoint_frame.grid(row=2, column=0, sticky="nsew", padx=5, pady=(0, 5))
        self.waypoint_frame.grid_columnconfigure((0, 1, 2, 3, 4, 5), weight=1)
        
        # Header row
        self._create_header()
    
    def _create_header(self):
        """Create the header row for waypoint display."""
        header_frame = ctk.CTkFrame(self.waypoint_frame, fg_color="#2b2b2b", height=25)
        header_frame.pack(fill="x", pady=(0, 3))
        header_frame.pack_propagate(False)
        
        headers = ["#", "Spot ID", "Latitude", "Longitude", "Area", "Detections"]
        widths = [30, 60, 100, 100, 60, 70]
        
        for header, width in zip(headers, widths):
            label = ctk.CTkLabel(
                header_frame,
                text=header,
                font=ctk.CTkFont(size=10, weight="bold"),
                width=width
            )
            label.pack(side="left", padx=1)
    
    def _filter_valid_waypoints(self, waypoints: List[Dict]) -> List[Dict]:
        """Filter waypoints to only include those with detection_count >= MIN_DETECTION_COUNT."""
        return [wp for wp in waypoints if wp.get('detection_count', 0) >= MIN_DETECTION_COUNT]
    
    def _refresh_from_spots(self):
        """Refresh waypoints from detected yellow spots."""
        try:
            # Get yellow spots from scanner drone
            spots_data = self.controller.get_yellow_spots(DroneName.Scanner)
            
            if not spots_data:
                self.waypoints = []
                self.valid_waypoints = []
                self._update_display()
                return
            
            # Convert spots to waypoints
            waypoints = []
            for spot_id, coords in spots_data.items():
                if not coords:
                    continue
                
                # Calculate average position for this spot
                avg_lat = sum(c["lat"] for c in coords) / len(coords)
                avg_lon = sum(c["lon"] for c in coords) / len(coords)
                avg_area = sum(c["area"] for c in coords) / len(coords)
                latest_rank = coords[-1].get("rank", 0)
                
                waypoint = {
                    'spot_id': spot_id,
                    'lat': avg_lat,
                    'lon': avg_lon,
                    'alt': 4.0,  # Default altitude
                    'area': avg_area,
                    'detection_count': len(coords),
                    'rank': latest_rank,
                    'type': 'yellow_spot'
                }
                waypoints.append(waypoint)
            
            # Sort by spot_id
            waypoints.sort(key=lambda x: int(x['spot_id']) if x['spot_id'].isdigit() else 0)
            
            self.waypoints = waypoints
            self.valid_waypoints = self._filter_valid_waypoints(waypoints)
            self._update_display()
            
            # Update controller with only valid waypoints
            self.controller.set_waypoints(self.valid_waypoints)
            
        except Exception as e:
            print(f"Error refreshing from spots: {e}")
            messagebox.showerror("Error", f"Failed to refresh waypoints:\n{e}")
    
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
                for i, row in enumerate(reader):
                    waypoint = {
                        'spot_id': row.get('spot_id', str(i + 1)),
                        'lat': float(row.get('latitude', row.get('lat', 0))),
                        'lon': float(row.get('longitude', row.get('lon', 0))),
                        'alt': float(row.get('altitude', row.get('alt', 4.0))),
                        'area': float(row.get('area', 0)),
                        'detection_count': int(row.get('detection_count', row.get('detections', MIN_DETECTION_COUNT))),
                        'rank': int(row.get('rank', 0)),
                        'type': row.get('type', 'imported')
                    }
                    imported_waypoints.append(waypoint)
            
            self.waypoints = imported_waypoints
            self.valid_waypoints = self._filter_valid_waypoints(imported_waypoints)
            self._update_display()
            
            # Update controller with only valid waypoints
            self.controller.set_waypoints(self.valid_waypoints)
            
            total_imported = len(imported_waypoints)
            valid_count = len(self.valid_waypoints)
            filtered_count = total_imported - valid_count
            
            msg = f"Imported {total_imported} waypoints from CSV.\n"
            msg += f"Valid waypoints (≥{MIN_DETECTION_COUNT} detections): {valid_count}\n"
            if filtered_count > 0:
                msg += f"Filtered out: {filtered_count} (insufficient detections)"
            
            messagebox.showinfo("Import Successful", msg)
            
        except Exception as e:
            messagebox.showerror("Import Error", f"Failed to import waypoints:\n{e}")
    
    def _export_waypoints(self):
        """Export only valid waypoints to a CSV file."""
        # Refresh from spots first if no waypoints
        if not self.waypoints:
            self._refresh_from_spots()
        
        if not self.valid_waypoints:
            messagebox.showwarning(
                "No Valid Waypoints", 
                f"No waypoints with ≥{MIN_DETECTION_COUNT} detections to export."
            )
            return
        
        filepath = filedialog.asksaveasfilename(
            title="Export Valid Waypoints",
            defaultextension=".csv",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")],
            initialdir=os.path.expanduser("~"),
            initialfile="yellow_spots_waypoints.csv"
        )
        
        if not filepath:
            return
        
        try:
            with open(filepath, 'w', newline='') as csvfile:
                fieldnames = ['index', 'spot_id', 'latitude', 'longitude', 'altitude', 
                             'area', 'detection_count', 'rank', 'type']
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                writer.writeheader()
                
                # Export only valid waypoints
                for i, wp in enumerate(self.valid_waypoints):
                    writer.writerow({
                        'index': i + 1,
                        'spot_id': wp.get('spot_id', ''),
                        'latitude': f"{wp.get('lat', 0):.7f}",
                        'longitude': f"{wp.get('lon', 0):.7f}",
                        'altitude': wp.get('alt', 4.0),
                        'area': f"{wp.get('area', 0):.2f}",
                        'detection_count': wp.get('detection_count', MIN_DETECTION_COUNT),
                        'rank': wp.get('rank', 0),
                        'type': wp.get('type', 'waypoint')
                    })
            
            messagebox.showinfo(
                "Export Successful",
                f"Exported {len(self.valid_waypoints)} valid waypoints to:\n{filepath}"
            )
            
        except Exception as e:
            messagebox.showerror("Export Error", f"Failed to export waypoints:\n{e}")
    
    def _clear_waypoints(self):
        """Clear all waypoints."""
        if self.waypoints:
            confirm = messagebox.askyesno(
                "Confirm Clear",
                "Clear waypoints from display?\n\n(This does not clear detected spots from scanner)"
            )
            if confirm:
                self.waypoints = []
                self.valid_waypoints = []
                self._update_display()
                self.controller.clear_waypoints()
    
    def _transfer_to_sprayer(self):
        """Transfer only valid waypoints to the sprayer drone."""
        if not self.waypoints:
            self._refresh_from_spots()
        
        if not self.valid_waypoints:
            messagebox.showwarning(
                "No Valid Waypoints", 
                f"No waypoints with ≥{MIN_DETECTION_COUNT} detections to transfer."
            )
            return
        
        try:
            # Ensure controller has valid waypoints
            self.controller.set_waypoints(self.valid_waypoints)
            success = self.controller.transfer_waypoints_to_sprayer()
            
            if success:
                messagebox.showinfo(
                    "Transfer Successful",
                    f"Transferred {len(self.valid_waypoints)} valid waypoints to Sprayer drone.\n"
                    f"(Spots with ≥{MIN_DETECTION_COUNT} detections only)"
                )
            else:
                messagebox.showerror(
                    "Transfer Failed",
                    "Failed to transfer waypoints to Sprayer drone."
                )
        except Exception as e:
            messagebox.showerror("Transfer Error", f"Failed to transfer waypoints:\n{e}")
    
    def set_waypoints(self, waypoints: List[Dict]):
        """Set waypoints externally and update display."""
        self.waypoints = waypoints or []
        self.valid_waypoints = self._filter_valid_waypoints(self.waypoints)
        self._update_display()
    
    def update_from_spots(self, spots_data: Dict[str, List[Dict]]):
        """Update waypoints from yellow spots data."""
        if not spots_data:
            return
        
        waypoints = []
        for spot_id, coords in spots_data.items():
            if not coords:
                continue
            
            avg_lat = sum(c["lat"] for c in coords) / len(coords)
            avg_lon = sum(c["lon"] for c in coords) / len(coords)
            avg_area = sum(c["area"] for c in coords) / len(coords)
            
            waypoint = {
                'spot_id': spot_id,
                'lat': avg_lat,
                'lon': avg_lon,
                'alt': 4.0,
                'area': avg_area,
                'detection_count': len(coords),
                'rank': coords[-1].get("rank", 0),
                'type': 'yellow_spot'
            }
            waypoints.append(waypoint)
        
        waypoints.sort(key=lambda x: int(x['spot_id']) if x['spot_id'].isdigit() else 0)
        self.waypoints = waypoints
        self.valid_waypoints = self._filter_valid_waypoints(waypoints)
        self._update_display()
    
    def _update_display(self):
        """Update the waypoint display - shows only valid waypoints."""
        # Clear existing waypoint rows (keep header)
        for widget in self.waypoint_frame.winfo_children()[1:]:
            widget.destroy()
        
        # Update count label
        total_spots = len(self.waypoints)
        valid_count = len(self.valid_waypoints)
        self.count_label.configure(
            text=f"Spots: {total_spots} | Valid (≥{MIN_DETECTION_COUNT} detections): {valid_count}"
        )
        
        # Add only valid waypoint rows
        for i, wp in enumerate(self.valid_waypoints):
            self._add_waypoint_row(i + 1, wp)
        
        # Show message if there are spots but none are valid yet
        if total_spots > 0 and valid_count == 0:
            info_frame = ctk.CTkFrame(self.waypoint_frame, fg_color="#2d2d2d")
            info_frame.pack(fill="x", pady=10, padx=5)
            
            ctk.CTkLabel(
                info_frame,
                text=f"⏳ Waiting for spots to be detected {MIN_DETECTION_COUNT}+ times...",
                font=ctk.CTkFont(size=11),
                text_color="#ffc107"
            ).pack(pady=10)
            
            ctk.CTkLabel(
                info_frame,
                text=f"({total_spots} spot(s) detected but need more confirmations)",
                font=ctk.CTkFont(size=10),
                text_color="#888888"
            ).pack(pady=(0, 10))
    
    def _add_waypoint_row(self, index: int, waypoint: Dict):
        """Add a single waypoint row to the display."""
        row_color = "#1e1e1e" if index % 2 == 0 else "#252525"
        row_frame = ctk.CTkFrame(self.waypoint_frame, fg_color=row_color, height=24)
        row_frame.pack(fill="x", pady=1)
        row_frame.pack_propagate(False)
        
        # Index
        ctk.CTkLabel(
            row_frame,
            text=str(index),
            width=30,
            font=ctk.CTkFont(size=10)
        ).pack(side="left", padx=1)
        
        # Spot ID
        ctk.CTkLabel(
            row_frame,
            text=str(waypoint.get('spot_id', '-')),
            width=60,
            font=ctk.CTkFont(size=10),
            text_color="#ffc107"
        ).pack(side="left", padx=1)
        
        # Latitude
        ctk.CTkLabel(
            row_frame,
            text=f"{waypoint.get('lat', 0):.6f}",
            width=100,
            font=ctk.CTkFont(size=10)
        ).pack(side="left", padx=1)
        
        # Longitude
        ctk.CTkLabel(
            row_frame,
            text=f"{waypoint.get('lon', 0):.6f}",
            width=100,
            font=ctk.CTkFont(size=10)
        ).pack(side="left", padx=1)
        
        # Area
        ctk.CTkLabel(
            row_frame,
            text=f"{waypoint.get('area', 0):.0f}",
            width=60,
            font=ctk.CTkFont(size=10)
        ).pack(side="left", padx=1)
        
        # Detection count with color coding
        det_count = waypoint.get('detection_count', 0)
        if det_count >= 5:
            det_color = "#28a745"  # Green - high confidence
        elif det_count >= MIN_DETECTION_COUNT:
            det_color = "#17a2b8"  # Blue - valid
        else:
            det_color = "#6c757d"  # Gray - below threshold (shouldn't appear)
        
        ctk.CTkLabel(
            row_frame,
            text=str(det_count),
            width=70,
            font=ctk.CTkFont(size=10, weight="bold"),
            text_color=det_color
        ).pack(side="left", padx=1)
    
    def get_waypoints(self) -> List[Dict]:
        """Get all waypoints (including those below threshold)."""
        return self.waypoints
    
    def get_valid_waypoints(self) -> List[Dict]:
        """Get only valid waypoints (detection_count >= MIN_DETECTION_COUNT)."""
        return self.valid_waypoints