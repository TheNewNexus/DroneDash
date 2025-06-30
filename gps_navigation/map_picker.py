import tkinter as tk
from tkinter import ttk, messagebox
import webbrowser
import json
import threading
import time
from pymavlink import mavutil
import folium
import tempfile
import os

class MAVLinkCoordinatePicker:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("MAVLink GPS Coordinate Picker")
        self.root.geometry("800x600")
        
        # MAVLink connection
        self.master = None
        self.connection_status = "Disconnected"
        
        # Coordinate storage
        self.waypoints = []
        self.current_destination = None
        
        self.setup_ui()
        self.load_waypoints()
    
    def setup_ui(self):
        # Connection frame
        conn_frame = tk.LabelFrame(self.root, text="MAVLink Connection", padx=10, pady=10)
        conn_frame.pack(fill=tk.X, padx=10, pady=5)
        
        tk.Label(conn_frame, text="Connection String:").grid(row=0, column=0, sticky=tk.W)
        self.conn_entry = tk.Entry(conn_frame, width=40)
        self.conn_entry.insert(0, "/dev/ttyTHS1")  # Your default connection
        self.conn_entry.grid(row=0, column=1, padx=5)
        
        tk.Label(conn_frame, text="Baud Rate:").grid(row=0, column=2, sticky=tk.W, padx=(10,0))
        self.baud_entry = tk.Entry(conn_frame, width=10)
        self.baud_entry.insert(0, "921600")
        self.baud_entry.grid(row=0, column=3, padx=5)
        
        self.connect_btn = tk.Button(conn_frame, text="Connect", command=self.connect_mavlink)
        self.connect_btn.grid(row=0, column=4, padx=10)
        
        self.status_label = tk.Label(conn_frame, text="Status: Disconnected", fg="red")
        self.status_label.grid(row=1, column=0, columnspan=5, pady=5)
        
        # Map selection frame
        map_frame = tk.LabelFrame(self.root, text="Coordinate Selection", padx=10, pady=10)
        map_frame.pack(fill=tk.X, padx=10, pady=5)
        
        tk.Button(map_frame, text="Open Interactive Map", command=self.open_interactive_map,
                 bg="#4285f4", fg="white", font=("Arial", 12), padx=20, pady=5).pack(pady=10)
        
        # Manual coordinate entry
        manual_frame = tk.Frame(map_frame)
        manual_frame.pack(pady=10)
        
        tk.Label(manual_frame, text="Latitude:").grid(row=0, column=0, padx=5)
        self.lat_entry = tk.Entry(manual_frame, width=15)
        self.lat_entry.grid(row=0, column=1, padx=5)
        
        tk.Label(manual_frame, text="Longitude:").grid(row=0, column=2, padx=5)
        self.lng_entry = tk.Entry(manual_frame, width=15)
        self.lng_entry.grid(row=0, column=3, padx=5)
        
        tk.Label(manual_frame, text="Altitude (m):").grid(row=0, column=4, padx=5)
        self.alt_entry = tk.Entry(manual_frame, width=10)
        self.alt_entry.insert(0, "10")
        self.alt_entry.grid(row=0, column=5, padx=5)
        
        tk.Button(manual_frame, text="Add Waypoint", command=self.add_waypoint).grid(row=0, column=6, padx=10)
        
        # Waypoint list
        list_frame = tk.LabelFrame(self.root, text="Waypoints", padx=10, pady=10)
        list_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)
        
        # Treeview for waypoints
        columns = ("Index", "Latitude", "Longitude", "Altitude", "Status")
        self.tree = ttk.Treeview(list_frame, columns=columns, show="headings", height=10)
        
        for col in columns:
            self.tree.heading(col, text=col)
            self.tree.column(col, width=120)
        
        scrollbar = ttk.Scrollbar(list_frame, orient=tk.VERTICAL, command=self.tree.yview)
        self.tree.configure(yscrollcommand=scrollbar.set)
        
        self.tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        
        # Control buttons
        btn_frame = tk.Frame(self.root)
        btn_frame.pack(fill=tk.X, padx=10, pady=10)
        
        tk.Button(btn_frame, text="Send to Vehicle", command=self.send_selected_waypoint,
                 bg="#28a745", fg="white", font=("Arial", 10, "bold")).pack(side=tk.LEFT, padx=5)
        
        tk.Button(btn_frame, text="Set as Mission", command=self.upload_mission).pack(side=tk.LEFT, padx=5)
        
        tk.Button(btn_frame, text="Delete Selected", command=self.delete_waypoint).pack(side=tk.LEFT, padx=5)
        
        tk.Button(btn_frame, text="Clear All", command=self.clear_waypoints).pack(side=tk.LEFT, padx=5)
        
        tk.Button(btn_frame, text="Export JSON", command=self.export_waypoints).pack(side=tk.RIGHT, padx=5)
        
        # Vehicle status
        status_frame = tk.LabelFrame(self.root, text="Vehicle Status", padx=10, pady=5)
        status_frame.pack(fill=tk.X, padx=10, pady=5)
        
        self.vehicle_status = tk.Label(status_frame, text="Vehicle: Unknown | Mode: Unknown | GPS: Unknown")
        self.vehicle_status.pack()
        
        self.update_tree()
    
    def connect_mavlink(self):
        """Connect to MAVLink"""
        try:
            conn_str = self.conn_entry.get()
            baud = int(self.baud_entry.get())
            
            if conn_str.startswith('/dev/'):
                # Serial connection
                self.master = mavutil.mavlink_connection(conn_str, baud=baud)
            else:
                # Network connection
                self.master = mavutil.mavlink_connection(conn_str)
            
            # Wait for heartbeat
            self.master.wait_heartbeat()
            
            self.connection_status = "Connected"
            self.status_label.config(text=f"Status: Connected to System {self.master.target_system}", fg="green")
            self.connect_btn.config(text="Disconnect")
            
            # Start status monitoring
            threading.Thread(target=self.monitor_vehicle_status, daemon=True).start()
            
        except Exception as e:
            messagebox.showerror("Connection Error", f"Failed to connect: {str(e)}")
    
    def monitor_vehicle_status(self):
        """Monitor vehicle status in background"""
        while self.master and self.connection_status == "Connected":
            try:
                # Get vehicle status
                msg = self.master.recv_match(type=['HEARTBEAT', 'GPS_RAW_INT'], blocking=False, timeout=1)
                if msg:
                    if msg.get_type() == 'HEARTBEAT':
                        mode = mavutil.mode_string_v10(msg)
                        armed = "ARMED" if msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED else "DISARMED"
                        
                    elif msg.get_type() == 'GPS_RAW_INT':
                        gps_status = f"GPS: {msg.satellites_visible} sats, Fix: {msg.fix_type}"
                        
                        status_text = f"Vehicle: {armed} | Mode: {mode} | {gps_status}"
                        self.vehicle_status.config(text=status_text)
                
                time.sleep(1)
            except:
                break
    
    def open_interactive_map(self):
        """Open an interactive map for coordinate selection"""
        # Create a folium map
        m = folium.Map(
            location=[34.4208, -119.6982],  # Santa Barbara default
            zoom_start=15,
            tiles='OpenStreetMap'
        )
        
        # Add existing waypoints to map
        for i, wp in enumerate(self.waypoints):
            folium.Marker(
                [wp['lat'], wp['lng']],
                popup=f"Waypoint {i+1}: {wp['lat']:.6f}, {wp['lng']:.6f}",
                tooltip=f"Waypoint {i+1}",
                icon=folium.Icon(color='red', icon='info-sign')
            ).add_to(m)
        
        # Add click functionality with JavaScript
        click_js = """
        function onMapClick(e) {
            var lat = e.latlng.lat.toFixed(6);
            var lng = e.latlng.lng.toFixed(6);
            
            // Create popup with coordinates
            var popup = L.popup()
                .setLatLng(e.latlng)
                .setContent("Coordinates: " + lat + ", " + lng + 
                           "<br><button onclick='copyCoords(" + lat + "," + lng + ")'>Copy Coordinates</button>")
                .openOn(map);
        }
        
        function copyCoords(lat, lng) {
            navigator.clipboard.writeText(lat + "," + lng);
            alert("Coordinates copied: " + lat + ", " + lng);
        }
        
        map.on('click', onMapClick);
        """
        
        # Save map to temporary file
        temp_file = tempfile.NamedTemporaryFile(mode='w', suffix='.html', delete=False)
        m.save(temp_file.name)
        
        # Add JavaScript to the HTML
        with open(temp_file.name, 'r') as f:
            html_content = f.read()
        
        html_content = html_content.replace('</script>', click_js + '</script>')
        
        with open(temp_file.name, 'w') as f:
            f.write(html_content)
        
        # Open in browser
        webbrowser.open('file://' + temp_file.name)
        
        messagebox.showinfo(
            "Map Instructions",
            "Click on the map to get coordinates.\n"
            "Copy the coordinates and paste them in the manual entry fields.\n"
            "Then click 'Add Waypoint' to save them."
        )
    
    def add_waypoint(self):
        """Add waypoint from manual entry"""
        try:
            lat = float(self.lat_entry.get())
            lng = float(self.lng_entry.get())
            alt = float(self.alt_entry.get())
            
            if -90 <= lat <= 90 and -180 <= lng <= 180:
                waypoint = {
                    'lat': lat,
                    'lng': lng,
                    'alt': alt,
                    'status': 'Ready'
                }
                self.waypoints.append(waypoint)
                self.update_tree()
                self.save_waypoints()
                
                # Clear entries
                self.lat_entry.delete(0, tk.END)
                self.lng_entry.delete(0, tk.END)
                
                messagebox.showinfo("Success", f"Added waypoint: {lat:.6f}, {lng:.6f}")
            else:
                messagebox.showerror("Error", "Invalid coordinate range")
                
        except ValueError:
            messagebox.showerror("Error", "Please enter valid numbers")
    
    def send_selected_waypoint(self):
        """Send selected waypoint to vehicle"""
        if not self.master:
            messagebox.showerror("Error", "Not connected to vehicle")
            return
        
        selected = self.tree.selection()
        if not selected:
            messagebox.showerror("Error", "Please select a waypoint")
            return
        
        item = selected[0]
        index = int(self.tree.item(item)["values"][0]) - 1
        waypoint = self.waypoints[index]
        
        try:
            # Send waypoint as guided mode target
            self.master.mav.set_position_target_global_int_send(
                0,  # time_boot_ms
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                0b0000111111111000,  # Position only
                int(waypoint['lat'] * 1e7),  # lat
                int(waypoint['lng'] * 1e7),  # lng
                waypoint['alt'],  # alt
                0, 0, 0,  # velocity
                0, 0, 0,  # acceleration
                0, 0  # yaw, yaw_rate
            )
            
            # Update status
            self.waypoints[index]['status'] = 'Sent'
            self.update_tree()
            
            messagebox.showinfo("Success", f"Sent waypoint to vehicle: {waypoint['lat']:.6f}, {waypoint['lng']:.6f}")
            
        except Exception as e:
            messagebox.showerror("Error", f"Failed to send waypoint: {str(e)}")
    
    def upload_mission(self):
        """Upload all waypoints as a mission"""
        if not self.master:
            messagebox.showerror("Error", "Not connected to vehicle")
            return
        
        if not self.waypoints:
            messagebox.showerror("Error", "No waypoints to upload")
            return
        
        try:
            # Clear existing mission
            self.master.mav.mission_clear_all_send(
                self.master.target_system,
                self.master.target_component
            )
            
            # Send mission count
            self.master.mav.mission_count_send(
                self.master.target_system,
                self.master.target_component,
                len(self.waypoints) + 1  # +1 for home
            )
            
            # Send home waypoint (current position)
            self.master.mav.mission_item_send(
                self.master.target_system,
                self.master.target_component,
                0,  # sequence
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                1, 1,  # current, autocontinue
                0, 0, 0, 0,  # param1-4
                0, 0, 0  # home position (will be set by vehicle)
            )
            
            # Send waypoints
            for i, wp in enumerate(self.waypoints):
                self.master.mav.mission_item_send(
                    self.master.target_system,
                    self.master.target_component,
                    i + 1,  # sequence
                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                    mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                    0, 1,  # current, autocontinue
                    0, 0, 0, 0,  # param1-4
                    wp['lat'], wp['lng'], wp['alt']
                )
                
                self.waypoints[i]['status'] = 'Uploaded'
            
            self.update_tree()
            messagebox.showinfo("Success", f"Uploaded {len(self.waypoints)} waypoints as mission")
            
        except Exception as e:
            messagebox.showerror("Error", f"Failed to upload mission: {str(e)}")
    
    def delete_waypoint(self):
        """Delete selected waypoint"""
        selected = self.tree.selection()
        if selected:
            item = selected[0]
            index = int(self.tree.item(item)["values"][0]) - 1
            self.waypoints.pop(index)
            self.update_tree()
            self.save_waypoints()
    
    def clear_waypoints(self):
        """Clear all waypoints"""
        if messagebox.askyesno("Confirm", "Clear all waypoints?"):
            self.waypoints = []
            self.update_tree()
            self.save_waypoints()
    
    def update_tree(self):
        """Update waypoint display"""
        for item in self.tree.get_children():
            self.tree.delete(item)
        
        for i, wp in enumerate(self.waypoints):
            self.tree.insert("", "end", values=(
                i + 1,
                f"{wp['lat']:.6f}",
                f"{wp['lng']:.6f}",
                f"{wp['alt']:.1f}",
                wp['status']
            ))
    
    def save_waypoints(self):
        """Save waypoints to file"""
        with open("mavlink_waypoints.json", 'w') as f:
            json.dump(self.waypoints, f, indent=2)
    
    def load_waypoints(self):
        """Load waypoints from file"""
        try:
            if os.path.exists("mavlink_waypoints.json"):
                with open("mavlink_waypoints.json", 'r') as f:
                    self.waypoints = json.load(f)
        except:
            self.waypoints = []
    
    def export_waypoints(self):
        """Export waypoints for use in other scripts"""
        if self.waypoints:
            filename = "exported_waypoints.json"
            with open(filename, 'w') as f:
                json.dump(self.waypoints, f, indent=2)
            messagebox.showinfo("Success", f"Waypoints exported to {filename}")
    
    def run(self):
        """Start the application"""
        self.root.mainloop()

# Integration function for your existing scripts
def load_mission_waypoints(filename="mavlink_waypoints.json"):
    """Load waypoints for use in your ArUco/GPS scripts"""
    try:
        with open(filename, 'r') as f:
            waypoints = json.load(f)
        return waypoints
    except FileNotFoundError:
        print(f"No waypoint file found: {filename}")
        return []

def send_vehicle_to_coordinate(master, lat, lng, alt=10):
    """Send vehicle to specific coordinate (for integration with your scripts)"""
    try:
        master.mav.set_position_target_global_int_send(
            0,  # time_boot_ms
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            0b0000111111111000,  # Position only
            int(lat * 1e7),  # lat
            int(lng * 1e7),  # lng
            alt,  # alt
            0, 0, 0,  # velocity
            0, 0, 0,  # acceleration
            0, 0  # yaw, yaw_rate
        )
        return True
    except Exception as e:
        print(f"Failed to send coordinate: {e}")
        return False

if __name__ == "__main__":
    app = MAVLinkCoordinatePicker()
    app.run()
