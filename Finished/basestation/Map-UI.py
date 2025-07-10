"""
Simple LoRa Live Tracker - Version 5
===================================
This programme tracks people using LoRa devices and shows them on a map.
It receives location data through a serial port and displays it in real-time.

Recent changes (7th July 2025):
• Added theme selector: Dark / Light / Original map styles
• Made heading arrow shorter (4 metres instead of 15 metres)
• Kept all other features from version 4
"""

# Import all the libraries we need for this programme
from __future__ import annotations  # This helps with type hints

import math        # For mathematical calculations
import re          # For pattern matching in text
import time        # For time-related functions
import itertools   # For cycling through colours
from typing import Dict, Tuple  # For better code documentation

import serial                    # For talking to the serial port
import customtkinter as ctk      # For the modern-looking user interface
from tkintermapview import TkinterMapView  # For the map display
from PIL import Image, ImageDraw, ImageTk  # For creating custom icons


# ═══════════════════════════════════════════════════════════════════════════════
# SETTINGS - Change these values to match your setup
# ═══════════════════════════════════════════════════════════════════════════════

# Serial port settings (where your LoRa device is connected)
SERIAL_PORT = "COM7"        # Change this to match your computer's port
BAUD_RATE = 115200          # How fast data is sent (bits per second)
SERIAL_TIMEOUT = 0.1        # How long to wait for data (seconds)

# How the programme behaves
UPDATE_SPEED = 100          # How often to check for new data (milliseconds)
MAP_ZOOM_LEVEL = 17         # How zoomed in the map should be (higher = closer)
ARROW_LENGTH_METRES = 4     # How long the direction arrow should be
ARROW_THICKNESS = 2         # How thick the arrow line should be
ARROW_COLOUR = "#FFA500"    # Orange colour for the direction arrow
PATH_LINE_THICKNESS = 3     # How thick the path line should be
MAX_PATH_POINTS = 250       # How many location points to remember
STALE_DATA_SECONDS = 5      # When to consider data as "old"
FLOOR_HEIGHT_METRES = 3      # ≈ storey height 


# Different map styles (tile servers)
DARK_MAP_TILES = "https://a.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}.png"
LIGHT_MAP_TILES = "https://a.basemaps.cartocdn.com/light_all/{z}/{x}/{y}.png"
STANDARD_MAP_TILES = "https://tile.openstreetmap.org/{z}/{x}/{y}.png"

# Pattern for recognising data from the LoRa device
# This matches lines like: "Data: 1 2 0 51.5072 -0.1279 10.5 45.2 1234"
DATA_PATTERN = re.compile(
    r"^Data:\s+"                # Starts with "Data:"
    r"(\d+)\s+"                 # Unit number (device ID)
    r"(\d+)\s+"                 # Person number (person ID)
    r"([01])\s+"                # SOS flag (0 or 1)
    r"(-?\d+(?:\.\d+)?)\s+"     # Latitude (can be negative, can have decimals)
    r"(-?\d+(?:\.\d+)?)\s+"     # Longitude (can be negative, can have decimals)
    r"(-?\d+(?:\.\d+)?)\s+"     # Elevation/altitude
    r"(-?\d+(?:\.\d+)?)\s+"     # Heading/direction
    r"(\d+)"                    # Step count
    r"\s*$"                     # End of line
)

# Compass directions for converting degrees to text
COMPASS_DIRECTIONS = ("N", "NE", "E", "SE", "S", "SW", "W", "NW")

# Colours for different trackers (cycles through these)
TRACKER_COLOURS = itertools.cycle([
    "#F94144",  # Red
    "#F3722C",  # Orange-red
    "#F8961E",  # Orange
    "#F9C74F",  # Yellow
    "#43AA8B",  # Green
    "#577590",  # Blue-grey
    "#277DA1",  # Blue
    "#9B5DE5",  # Purple
])


# ═══════════════════════════════════════════════════════════════════════════════
# HELPER FUNCTIONS - Small functions that do specific jobs
# ═══════════════════════════════════════════════════════════════════════════════

def convert_heading_to_compass(degrees: float) -> str:
    """
    Convert a heading in degrees to a compass direction like "N" or "SE"
    For example: 0° = N, 45° = NE, 90° = E, etc.
    """
    # Add 22.5 degrees to shift the boundaries, then divide by 45
    direction_index = int(((degrees + 22.5) // 45) % 8)
    return COMPASS_DIRECTIONS[direction_index]


def elevation_to_floor(elevation: float,
                       floor_height: float = FLOOR_HEIGHT_METRES) -> str:
    """
    Convert elevation (m) to a human-readable floor label.
    •  <1 m above reference → “G”
    •  ≥1 m → numbered floors (1, 2, …)
    •  Below ground → “B1”, “B2”, …
    """
    if elevation < -0.5 * floor_height:                 # basement
        return f"B{int((-elevation) // floor_height) + 1}"
    if elevation < 1:                                   # ground floor tolerance
        return "G"
    return str(int((elevation + floor_height/2) // floor_height))


def calculate_new_position(latitude: float, longitude: float, heading_degrees: float,
                          distance_metres: float = ARROW_LENGTH_METRES) -> Tuple[float, float]:
    """
    Calculate where you'd end up if you walked a certain distance in a direction.
    This is used to draw the arrow showing which way someone is heading.
    """
    EARTH_RADIUS_METRES = 6_378_137  # Earth's radius in metres
    
    # Convert heading to radians (maths functions need radians, not degrees)
    heading_radians = math.radians(heading_degrees)
    latitude_radians = math.radians(latitude)
    
    # Calculate the new position
    new_latitude = latitude + (distance_metres * math.cos(heading_radians)) / EARTH_RADIUS_METRES * 180 / math.pi
    new_longitude = longitude + (distance_metres * math.sin(heading_radians)) / (EARTH_RADIUS_METRES * math.cos(latitude_radians)) * 180 / math.pi
    
    return new_latitude, new_longitude

def create_circular_icon(colour: str, size: int = 26, border_thickness: int = 2) -> ImageTk.PhotoImage:
    """
    Create a small circular icon for showing on the map.
    Each tracker gets a different colour so you can tell them apart.
    """
    # Create a new image with transparency
    icon_image = Image.new("RGBA", (size, size))
    drawing_tool = ImageDraw.Draw(icon_image)
    
    # Draw a circle with the specified colour and a white border
    drawing_tool.ellipse(
        (border_thickness, border_thickness, size - border_thickness, size - border_thickness),
        fill=colour,
        outline="white",
        width=border_thickness
    )
    
    # Convert to a format that tkinter can use
    return ImageTk.PhotoImage(icon_image)


# ═══════════════════════════════════════════════════════════════════════════════
# MAIN PROGRAMME CLASS - This is where all the magic happens
# ═══════════════════════════════════════════════════════════════════════════════

class SimpleTrackerApp:
    """
    The main application that handles everything:
    - Connecting to the serial port
    - Reading location data
    - Drawing the map
    - Managing the user interface
    """
    
    def __init__(self, main_window: ctk.CTk):
        """Set up the application when it first starts"""
        self.main_window = main_window
        
        # Set up the main window
        main_window.title("Simple LoRa Live Tracker")
        ctk.set_appearance_mode("dark")  # Start with dark theme
        ctk.set_default_color_theme("dark-blue")
        main_window.geometry("1800x980")  # Make the window quite large
        
        # Create different text styles for the interface
        self.title_font = ctk.CTkFont("Consolas", size=16, weight="bold")
        self.normal_font = ctk.CTkFont("Consolas", size=14)
        self.small_font = ctk.CTkFont("Consolas", size=12)
        
        # Create the main layout: map on the left, controls on the right
        self.setup_user_interface()
        
        # Set up serial port communication
        self.setup_serial_connection()
        
        # Dictionary to store information about each tracker
        self.tracker_data: Dict[str, Dict] = {}
        
        # Start the main loops that keep the programme running
        main_window.after(UPDATE_SPEED, self.check_for_new_data)
        main_window.after(1000, self.update_age_labels)
    
    def setup_user_interface(self):
        """Create all the visual elements of the programme"""
        
        # Create the map area (left side of the window)
        self.map_area = ctk.CTkFrame(self.main_window, corner_radius=0)
        self.map_area.pack(side="left", fill="both", expand=True)
        
        # Create the control panel (right side of the window)
        self.control_panel = ctk.CTkScrollableFrame(self.main_window, width=360, corner_radius=0)
        self.control_panel.pack(side="right", fill="y")
        
        # Create the theme selector dropdown
        self.selected_theme = ctk.StringVar(value="Dark")
        theme_selector = ctk.CTkOptionMenu(
            self.control_panel,
            variable=self.selected_theme,
            values=["Dark", "Light", "Original"],
            command=self.change_map_theme,
            width=120,
        )
        theme_selector.pack(pady=(10, 4), padx=8)
        
        # Create the actual map widget
        self.map_widget = TkinterMapView(self.map_area, corner_radius=0)
        self.map_widget.pack(fill="both", expand=True)
        
        # Set up the map with initial settings
        self.map_widget.set_tile_server(DARK_MAP_TILES)  # Start with dark theme
        self.map_widget.set_position(51.5072256, -0.1279288)  # London coordinates
        self.map_widget.set_zoom(MAP_ZOOM_LEVEL)
        
        # Create the serial connection status display
        self.setup_serial_status_display()
    
    def setup_serial_status_display(self):
        """Create the area that shows whether the serial connection is working"""
        
        # Create a frame for the serial status
        self.serial_status_frame = ctk.CTkFrame(self.control_panel, corner_radius=6)
        self.serial_status_frame.pack(fill="x", pady=6, padx=8)
        
        # Label showing the connection status
        self.serial_status_label = ctk.CTkLabel(
            self.serial_status_frame,
            text=f"Serial ({SERIAL_PORT}): Connecting...",
            font=self.small_font
        )
        self.serial_status_label.pack(pady=(4, 2))
        
        # Button to manually try reconnecting
        self.reconnect_button = ctk.CTkButton(
            self.serial_status_frame,
            text="Refresh",
            width=100,
            height=24,
            font=self.small_font,
            command=self.manually_reconnect
        )
        self.reconnect_button.pack(pady=(0, 4))
    
    def setup_serial_connection(self):
        """Set up the connection to the serial port where data comes from"""
        
        # Variables to track the connection
        self.serial_port = None
        self.serial_connected = False
        self.reconnect_attempts = 0
        self.max_reconnect_attempts = 5
        self.reconnect_delay = 2000  # Wait 2 seconds between attempts
        
        # Try to connect for the first time
        self.connect_to_serial_port()
    
    def connect_to_serial_port(self):
        """Try to connect to the serial port"""
        try:
            # Close any existing connection first
            if self.serial_port and self.serial_port.is_open:
                self.serial_port.close()
            
            # Try to open the serial port
            self.serial_port = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=SERIAL_TIMEOUT)
            
            # If we got here, the connection worked
            self.serial_connected = True
            self.reconnect_attempts = 0
            self.update_serial_status("Connected", "#43AA8B")  # Green colour
            print(f"Successfully connected to serial port {SERIAL_PORT}")
            
        except serial.SerialException as error:
            # The connection failed
            self.serial_connected = False
            self.serial_port = None
            self.reconnect_attempts += 1
            
            if self.reconnect_attempts <= self.max_reconnect_attempts:
                # Still trying to connect
                self.update_serial_status(
                    f"Trying to connect... ({self.reconnect_attempts}/{self.max_reconnect_attempts})",
                    "#F8961E"  # Orange colour
                )
                print(f"Serial connection failed, attempt {self.reconnect_attempts}: {error}")
                
                # Try again in a few seconds
                self.main_window.after(self.reconnect_delay, self.connect_to_serial_port)
            else:
                # Given up trying to connect
                self.update_serial_status("Connection Failed", "#F94144")  # Red colour
                print(f"Serial connection failed after {self.max_reconnect_attempts} attempts: {error}")
    
    def update_serial_status(self, status_text: str, colour: str):
        """Update the serial status display in the user interface"""
        self.serial_status_label.configure(
            text=f"Serial ({SERIAL_PORT}): {status_text}",
            text_color=colour
        )
    
    def check_serial_connection_health(self):
        """Check if the serial connection is still working properly"""
        if not self.serial_port or not self.serial_port.is_open:
            self.serial_connected = False
            return False
        
        try:
            # Try to check if there's data waiting
            self.serial_port.in_waiting
            return True
        except (serial.SerialException, OSError):
            # Something went wrong
            self.serial_connected = False
            return False
    
    def manually_reconnect(self):
        """Force a reconnection when the user clicks the Refresh button"""
        self.reconnect_attempts = 0  # Reset the attempt counter
        self.update_serial_status("Reconnecting...", "#F8961E")  # Orange colour
        self.connect_to_serial_port()
    
    def change_map_theme(self, selected_theme: str):
        """Change the map's visual style when the user selects a different theme"""
        if selected_theme == "Light":
            ctk.set_appearance_mode("light")
            self.map_widget.set_tile_server(LIGHT_MAP_TILES)
        elif selected_theme == "Original":
            ctk.set_appearance_mode("light")
            self.map_widget.set_tile_server(STANDARD_MAP_TILES)
        else:  # Dark theme
            ctk.set_appearance_mode("dark")
            self.map_widget.set_tile_server(DARK_MAP_TILES)
    
    def check_for_new_data(self):
        """Check if there's new location data from the serial port"""
        received_line = ""
        current_time = time.time()
        
        # Check if our serial connection is still working
        if not self.serial_connected or not self.check_serial_connection_health():
            if self.serial_connected:  # Connection was lost
                self.serial_connected = False
                self.update_serial_status("Disconnected", "#F94144")  # Red colour
                print("Serial connection lost, trying to reconnect...")
                self.connect_to_serial_port()
        else:
            # Try to read any new data
            try:
                if self.serial_port and self.serial_port.in_waiting > 0:
                    # Read a line of data and convert it to text
                    received_line = self.serial_port.readline().decode(errors="ignore").strip()
            except (serial.SerialException, OSError) as error:
                print(f"Error reading from serial port: {error}")
                self.serial_connected = False
                self.update_serial_status("Read Error", "#F94144")  # Red colour
                self.connect_to_serial_port()
                received_line = ""
        
        # If we got some data, try to understand it
        if received_line:
            pattern_match = DATA_PATTERN.match(received_line)
            if pattern_match:
                # Extract all the values from the line
                (unit_id, person_id, sos_flag, latitude, longitude,
                 elevation, heading, steps) = pattern_match.groups()
                
                # Process this new location data
                self.process_new_location_data(
                    timestamp=current_time,
                    unit_id=int(unit_id),
                    person_id=int(person_id),
                    sos_active=bool(int(sos_flag)),
                    latitude=float(latitude),
                    longitude=float(longitude),
                    elevation=float(elevation),
                    heading=float(heading),
                    steps=int(steps),
                )
        
        # Schedule the next check
        self.main_window.after(UPDATE_SPEED, self.check_for_new_data)
    
    def process_new_location_data(self, timestamp: float, **location_data):
        """Process new location data that we've received"""
        
        # Create a unique identifier for this tracker
        tracker_id = f"U{location_data['unit_id']}-P{location_data['person_id']}"
        
        # Get the tracker data, or create a new one if it doesn't exist
        if tracker_id not in self.tracker_data:
            tracker_info = self.create_new_tracker(tracker_id)
        else:
            tracker_info = self.tracker_data[tracker_id]
        
        # Update the tracker with the new data
        self.update_tracker_data(tracker_info, timestamp=timestamp, **location_data)
    
    def create_new_tracker(self, tracker_id: str):
        """Create a new tracker when we see a device for the first time"""
        
        # Get the next colour for this tracker
        tracker_colour = next(TRACKER_COLOURS)
        
        # Create icons for normal and SOS states
        normal_icon = create_circular_icon(tracker_colour)
        sos_icon = create_circular_icon("#FF2D55")  # Red for emergencies
        
        # Create a card in the control panel for this tracker
        tracker_card = ctk.CTkFrame(self.control_panel, corner_radius=6, fg_color="#1E1E1E")
        tracker_card.pack(fill="x", pady=6, padx=8)
        
        # Create the top row with the tracker name and buttons
        top_row = ctk.CTkFrame(tracker_card, fg_color="transparent")
        top_row.pack(fill="x", pady=(2, 0))
        
        # Label showing the tracker ID
        tracker_label = ctk.CTkLabel(top_row, text=tracker_id, font=self.title_font, text_color=tracker_colour)
        tracker_label.pack(side="left")
        
        # Button to centre the map on this tracker
        centre_button = ctk.CTkButton(
            top_row, text="Centre", width=70, font=self.small_font,
            command=lambda: self.centre_map_on_tracker(tracker_id))
        centre_button.pack(side="right", padx=2)
        
        # Button to hide/show this tracker
        visibility_button = ctk.CTkButton(
            top_row, text="Hide", width=70, font=self.small_font,
            command=lambda: self.toggle_tracker_visibility(tracker_id))
        visibility_button.pack(side="right", padx=2)
        
        # Label showing the current data
        data_label = ctk.CTkLabel(tracker_card, text="—", font=self.normal_font,
                                anchor="w", justify="left")
        data_label.pack(fill="x", padx=4)
        
        # Label showing how old the data is
        age_label = ctk.CTkLabel(tracker_card, text="", font=self.small_font, anchor="w")
        age_label.pack(fill="x", padx=4, pady=(0, 4))
        
        # Create the tracker data structure
        tracker_info = {
            "colour": tracker_colour,
            "normal_icon": normal_icon,
            "sos_icon": sos_icon,
            "map_marker": None,
            "path_line": None,
            "direction_arrow": None,
            "location_history": [],
            "last_update_time": time.time(),
            "interface_elements": {
                "card": tracker_card,
                "data_label": data_label,
                "age_label": age_label,
                "tracker_label": tracker_label,
                "visibility_button": visibility_button,
            },
            "is_visible": True,
        }
        
        # Store the tracker data
        self.tracker_data[tracker_id] = tracker_info
        return tracker_info
    
    def update_tracker_data(self, tracker_info: Dict, *, timestamp: float, sos_active: bool,
                            latitude: float, longitude: float, elevation: float,
                            heading: float, steps: int, **other_data):
        """Update a tracker with new location data"""

        tracker_colour = tracker_info["colour"]
        tracker_info["last_update_time"] = timestamp

        # ───── floor label ──────────────────────────────────────────────────────────
        floor_str = elevation_to_floor(elevation)  # "G", "1", "B1", … (see helper)
        # ────────────────────────────────────────────────────────────────────────────

        # Update the information display in the control panel
        compass_direction = convert_heading_to_compass(heading)
        tracker_info["interface_elements"]["data_label"].configure(
            text=(
                f"{compass_direction} {heading:.0f}°  "
                f"alt:{elevation:.0f} m (floor {floor_str})\n"
                f"steps:{steps}  SOS:{'YES' if sos_active else 'no'}"
            ),
            text_color="#FF2D55" if sos_active else "white",
        )

        # Update or create the marker on the map
        if tracker_info["map_marker"] is None:
            tracker_info["map_marker"] = self.map_widget.set_marker(
                latitude, longitude,
                icon=tracker_info["sos_icon"] if sos_active else tracker_info["normal_icon"],
                text="",
                text_color=tracker_colour,
            )
        else:
            tracker_info["map_marker"].set_position(latitude, longitude)
            try:
                tracker_info["map_marker"].set_icon(
                    tracker_info["sos_icon"] if sos_active else tracker_info["normal_icon"]
                )
            except AttributeError:
                tracker_info["map_marker"].delete()
                tracker_info["map_marker"] = self.map_widget.set_marker(
                    latitude, longitude,
                    icon=tracker_info["sos_icon"] if sos_active else tracker_info["normal_icon"],
                    text="",
                    text_color=tracker_colour,
                )

        # Add this location to the history
        tracker_info["location_history"].append((latitude, longitude))
        if len(tracker_info["location_history"]) > MAX_PATH_POINTS:
            tracker_info["location_history"].pop(0)

        # Redraw the path and direction arrow
        self.redraw_tracker_path(tracker_info)

        # Handle SOS flashing
        self.handle_sos_flashing(tracker_info, sos_active)

    
    def redraw_tracker_path(self, tracker_info: Dict):
        """Redraw the path and direction arrow for a tracker"""
        
        locations = tracker_info["location_history"]
        if len(locations) < 2:
            return  # Need at least 2 points to draw a path
        
        tracker_colour = tracker_info["colour"]
        
        # Draw the path line
        if tracker_info["path_line"]:
            tracker_info["path_line"].delete()
        tracker_info["path_line"] = self.map_widget.set_path(
            locations, width=PATH_LINE_THICKNESS, color=tracker_colour)
        
        # Draw the direction arrow
        current_latitude, current_longitude = locations[-1]
        previous_latitude, previous_longitude = locations[-2]
        
        # Calculate the direction based on the last two points
        heading_degrees = math.degrees(math.atan2(
            current_longitude - previous_longitude,
            current_latitude - previous_latitude
        ))
        
        # Calculate where the arrow should point to
        arrow_end_lat, arrow_end_lon = calculate_new_position(
            current_latitude, current_longitude, heading_degrees
        )
        
        # Draw the arrow
        arrow_points = [(current_latitude, current_longitude), (arrow_end_lat, arrow_end_lon)]
        if tracker_info["direction_arrow"]:
            tracker_info["direction_arrow"].delete()
        tracker_info["direction_arrow"] = self.map_widget.set_path(
            arrow_points, width=ARROW_THICKNESS, color=ARROW_COLOUR)
    
    def handle_sos_flashing(self, tracker_info: Dict, sos_active: bool):
        """Make the tracker card flash red when SOS is active"""
        
        tracker_card = tracker_info["interface_elements"]["card"]
        
        if not sos_active:
            # Not in SOS mode, so use normal colours
            tracker_card.configure(fg_color="#1E1E1E")
            return
        
        # SOS is active, so make it flash
        def flash_card(is_bright=[False]):
            if not tracker_info["is_visible"]:
                return  # Don't flash if the tracker is hidden
            
            # Toggle between bright and dim red
            is_bright[0] = not is_bright[0]
            tracker_card.configure(fg_color="#550000" if is_bright[0] else "#400000")
            
            # Keep flashing if SOS is still active
            if sos_active:
                tracker_card.after(400, flash_card)
        
        # Only start flashing if we're not already flashing
        if tracker_card.cget("fg_color") == "#1E1E1E":
            flash_card()
    
    def update_age_labels(self):
        """Update the labels showing how old each tracker's data is"""
        
        current_time = time.time()
        for tracker_info in self.tracker_data.values():
            # Calculate how old the data is
            age_seconds = int(current_time - tracker_info["last_update_time"])
            
            # Update the age label
            tracker_info["interface_elements"]["age_label"].configure(
                text=f"last: {age_seconds}s ago",
                text_color="#888" if age_seconds > STALE_DATA_SECONDS else "#AAA"
            )
        
        # Schedule the next update
        self.main_window.after(1000, self.update_age_labels)
    
    def centre_map_on_tracker(self, tracker_id: str):
        """Centre the map on a specific tracker"""
        
        tracker_info = self.tracker_data.get(tracker_id)
        if tracker_info and tracker_info["location_history"]:
            # Get the most recent location
            latest_latitude, latest_longitude = tracker_info["location_history"][-1]
            self.map_widget.set_position(latest_latitude, latest_longitude)
    
    def toggle_tracker_visibility(self, tracker_id: str):
        """Show or hide a tracker on the map"""
        
        tracker_info = self.tracker_data.get(tracker_id)
        if not tracker_info:
            return
        
        currently_visible = tracker_info["is_visible"]
        tracker_info["is_visible"] = not currently_visible
        
        # Update the button text
        visibility_button = tracker_info["interface_elements"]["visibility_button"]
        visibility_button.configure(text="Hide" if not currently_visible else "Show")
        
        # Elements that can be shown/hidden
        map_elements = ("map_marker", "path_line", "direction_arrow")
        
        if currently_visible:
            # Hide the tracker
            for element_name in map_elements:
                map_element = tracker_info.get(element_name)
                if map_element:
                    map_element.delete()
                    tracker_info[element_name] = None
            
            # Grey out the tracker name
            tracker_info["interface_elements"]["tracker_label"].configure(text_color="#555555")
        else:
            # Show the tracker
            tracker_colour = tracker_info["colour"]
            locations = tracker_info["location_history"]
            
            if locations:
                # Recreate the marker
                latest_latitude, latest_longitude = locations[-1]
                tracker_info["map_marker"] = self.map_widget.set_marker(
                    latest_latitude, latest_longitude,
                    icon=tracker_info["normal_icon"],
                    text="", text_color=tracker_colour)
                
                # Recreate the path if we have enough points
                if len(locations) >= 2:
                    tracker_info["path_line"] = self.map_widget.set_path(
                        locations, width=PATH_LINE_THICKNESS, color=tracker_colour)
                    
                    # Recreate the direction arrow
                    previous_latitude, previous_longitude = locations[-2]
                    heading_degrees = math.degrees(math.atan2(
                        latest_longitude - previous_longitude,
                        latest_latitude - previous_latitude
                    ))
                    arrow_end_lat, arrow_end_lon = calculate_new_position(
                        latest_latitude, latest_longitude, heading_degrees
                    )
                    tracker_info["direction_arrow"] = self.map_widget.set_path(
                        [(latest_latitude, latest_longitude), (arrow_end_lat, arrow_end_lon)],
                        width=ARROW_THICKNESS, color=ARROW_COLOUR)
            
            # Restore the tracker name colour
            tracker_info["interface_elements"]["tracker_label"].configure(text_color=tracker_colour)
    
    def cleanup_before_closing(self):
        """Clean up resources when the programme is closing"""
        
        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.close()
                print("Serial port closed successfully")
            except Exception as error:
                print(f"Error closing serial port: {error}")


# ═══════════════════════════════════════════════════════════════════════════════
# PROGRAMME ENTRY POINT - This is where the programme starts
# ═══════════════════════════════════════════════════════════════════════════════

if __name__ == "__main__":
    # Create the main window
    main_window = ctk.CTk()
    
    # Create the application
    tracker_app = SimpleTrackerApp(main_window)
    
    # Handle what happens when the user closes the window
    def on_window_closing():
        """This function runs when the user tries to close the programme"""
        tracker_app.cleanup_before_closing()  # Clean up resources
        main_window.destroy()  # Close the window
    
    # Tell the window what to do when it's closed
    main_window.protocol("WM_DELETE_WINDOW", on_window_closing)
    
    # Start the programme (this keeps it running until closed)
    main_window.mainloop()
