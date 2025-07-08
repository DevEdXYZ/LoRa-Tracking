"""LoRa Live-Tracker – v5
====================================================
Additions on 2025-07-08:

• Theme selector: Dark / Light / Original (OpenStreetMap).
• Heading indicator: thin orange segment, 4 m long (was 15 m).
• Refresh button
• Retry attempts after disconnect

All other v4 features retained.
"""


from __future__ import annotations

import math
import re
import time
import itertools
from typing import Dict, Tuple

import serial
import customtkinter as ctk
from tkintermapview import TkinterMapView
from PIL import Image, ImageDraw, ImageTk
import webbrowser, tempfile, textwrap

# ─── serial configuration ─────────────────────────────────────
SERIAL_PORT = "COM4"        # ← adjust to your port
BAUDRATE = 115200
TIMEOUT_S = 0.1             # non-blocking read

# ─── UI / behaviour constants ─────────────────────────────────
UPDATE_INTERVAL = 100        # ms
ZOOM_LEVEL = 17
ARROW_LEN_M   = 4            # metres (¼ of previous length)
ARROW_WIDTH   = 2
ARROW_COLOR   = "#FFA500"    # orange
PATH_WIDTH    = 3
POINT_RETENTION = 250
STALE_AFTER_S   = 5

# ─── map tile servers ─────────────────────────────────────────
DARK_TILES   = "https://a.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}.png"
LIGHT_TILES  = "https://a.basemaps.cartocdn.com/light_all/{z}/{x}/{y}.png"
ORIG_TILES   = "https://tile.openstreetmap.org/{z}/{x}/{y}.png"

# ─── packet regex ─────────────────────────────────────────────
DATA_RE = re.compile(
    r"^Data:\s+"                # header
    r"(\d+)\s+"                 # unit id
    r"(\d+)\s+"                 # person id
    r"([01])\s+"                # sos flag
    r"(-?\d+(?:\.\d+)?)\s+"     # lat
    r"(-?\d+(?:\.\d+)?)\s+"     # lon
    r"(-?\d+(?:\.\d+)?)\s+"     # elev
    r"(-?\d+(?:\.\d+)?)\s+"     # heading
    r"(\d+)"                    # steps
    r"\s*$"
)

DIRS = ("N", "NE", "E", "SE", "S", "SW", "W", "NW")
PALETTE = itertools.cycle([
    "#F94144", "#F3722C", "#F8961E", "#F9C74F",
    "#43AA8B", "#577590", "#277DA1", "#9B5DE5",
])

# ─── helpers ──────────────────────────────────────────────────
def heading_to_dir(deg: float) -> str:
    return DIRS[int(((deg + 22.5) // 45) % 8)]

def project(lat: float, lon: float, heading_deg: float,
            d_m: float = ARROW_LEN_M) -> Tuple[float, float]:
    """Project point d_m metres along heading."""
    R = 6_378_137  # Earth radius
    h   = math.radians(heading_deg)
    lat_rad = math.radians(lat)
    new_lat = lat + (d_m * math.cos(h)) / R * 180 / math.pi
    new_lon = lon + (d_m * math.sin(h)) / (R * math.cos(lat_rad)) * 180 / math.pi
    return new_lat, new_lon

def build_icon(color: str, size: int = 26, border: int = 2) -> ImageTk.PhotoImage:
    img = Image.new("RGBA", (size, size))
    draw = ImageDraw.Draw(img)
    draw.ellipse((border, border, size - border, size - border),
                 fill=color, outline="white", width=border)
    return ImageTk.PhotoImage(img)

# ─── main application ─────────────────────────────────────────
class TrackerApp:
    def __init__(self, root: ctk.CTk):
        self.root = root
        root.title("LoRa Live Tracker")
        ctk.set_appearance_mode("dark")
        ctk.set_default_color_theme("dark-blue")
        root.geometry("1800x980")

        # Fonts --------------------------------------------------
        self.f_title = ctk.CTkFont("Consolas", size=16, weight="bold")
        self.f_body  = ctk.CTkFont("Consolas", size=14)
        self.f_small = ctk.CTkFont("Consolas", size=12)

        # Layout -------------------------------------------------
        self.map_frame = ctk.CTkFrame(root, corner_radius=0)
        self.map_frame.pack(side="left", fill="both", expand=True)

        self.sidebar = ctk.CTkScrollableFrame(root, width=360, corner_radius=0)
        self.sidebar.pack(side="right", fill="y")

        # Theme selector ----------------------------------------
        self.theme_var = ctk.StringVar(value="Dark")
        theme_sel = ctk.CTkOptionMenu(
            self.sidebar,
            variable=self.theme_var,
            values=["Dark", "Light", "Original"],
            command=self.set_theme,
            width=120,
        )
        theme_sel.pack(pady=(10, 4), padx=8)

        # Map widget --------------------------------------------
        self.map = TkinterMapView(self.map_frame, corner_radius=0)
        self.map.pack(fill="both", expand=True)
        self.map.set_tile_server(DARK_TILES)
        self.map.set_position(51.5072256, -0.1279288)
        self.map.set_zoom(ZOOM_LEVEL)

        # Serial ------------------------------------------------
        self.ser = None
        self.serial_connected = False
        self.reconnect_attempts = 0
        self.max_reconnect_attempts = 5
        self.reconnect_delay = 2000  # ms between reconnection attempts
        
        # Serial status indicator
        self.serial_status_frame = ctk.CTkFrame(self.sidebar, corner_radius=6)
        self.serial_status_frame.pack(fill="x", pady=6, padx=8)
        
        self.serial_status_label = ctk.CTkLabel(
            self.serial_status_frame,
            text=f"Serial ({SERIAL_PORT}): Connecting...",
            font=self.f_small
        )
        self.serial_status_label.pack(pady=(4, 2))
        
        self.reconnect_button = ctk.CTkButton(
            self.serial_status_frame,
            text="Refresh",
            width=100,
            height=24,
            font=self.f_small,
            command=self.force_reconnect
        )
        self.reconnect_button.pack(pady=(0, 4))
        
        # Try initial connection
        self.connect_serial()

        # Data structures --------------------------------------
        self.tracks: Dict[str, Dict] = {}

        # Start loops -------------------------------------------
        root.after(UPDATE_INTERVAL, self.poll_serial)
        root.after(1000, self.refresh_age_labels)

    # ─── Serial connection management ------------------------
    def connect_serial(self):
        """Attempt to connect to the serial port."""
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
            
            self.ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=TIMEOUT_S)
            self.serial_connected = True
            self.reconnect_attempts = 0
            self.update_serial_status("Connected", "#43AA8B")
            print(f"Serial connected to {SERIAL_PORT}")
            
        except serial.SerialException as e:
            self.serial_connected = False
            self.ser = None
            self.reconnect_attempts += 1
            
            if self.reconnect_attempts <= self.max_reconnect_attempts:
                self.update_serial_status(
                    f"Reconnecting... ({self.reconnect_attempts}/{self.max_reconnect_attempts})",
                    "#F8961E"
                )
                print(f"Serial connection failed, attempt {self.reconnect_attempts}: {e}")
                # Schedule reconnection attempt
                self.root.after(self.reconnect_delay, self.connect_serial)
            else:
                self.update_serial_status("Connection Failed", "#F94144")
                print(f"Serial connection failed after {self.max_reconnect_attempts} attempts: {e}")
    
    def update_serial_status(self, status_text: str, color: str):
        """Update the serial status indicator in the UI."""
        self.serial_status_label.configure(
            text=f"Serial ({SERIAL_PORT}): {status_text}",
            text_color=color
        )
    
    def check_serial_connection(self):
        """Check if serial connection is still valid."""
        if not self.ser or not self.ser.is_open:
            self.serial_connected = False
            return False
        
        try:
            # Try to read the port's status
            self.ser.in_waiting
            return True
        except (serial.SerialException, OSError):
            self.serial_connected = False
            return False

    def force_reconnect(self):
        """Force a reconnection attempt (triggered by button click)."""
        self.reconnect_attempts = 0  # Reset attempt counter
        self.update_serial_status("Reconnecting...", "#F8961E")
        self.connect_serial()

    # ─── Theme switching --------------------------------------
    def set_theme(self, choice: str):
        if choice == "Light":
            ctk.set_appearance_mode("light")
            self.map.set_tile_server(LIGHT_TILES)
        elif choice == "Original":
            ctk.set_appearance_mode("light")
            self.map.set_tile_server(ORIG_TILES)
        else:  # Dark
            ctk.set_appearance_mode("dark")
            self.map.set_tile_server(DARK_TILES)

    # ─── Serial polling ---------------------------------------
    def poll_serial(self):
        line = ""
        ts = time.time()
        
        # Check if we have a valid connection
        if not self.serial_connected or not self.check_serial_connection():
            if self.serial_connected:  # Connection was lost
                self.serial_connected = False
                self.update_serial_status("Disconnected", "#F94144")
                print("Serial connection lost, attempting to reconnect...")
                self.connect_serial()
        else:
            # Try to read from serial port
            try:
                if self.ser and self.ser.in_waiting > 0:
                    line = self.ser.readline().decode(errors="ignore").strip()
            except (serial.SerialException, OSError) as e:
                print(f"Serial read error: {e}")
                self.serial_connected = False
                self.update_serial_status("Read Error", "#F94144")
                self.connect_serial()
                line = ""

        # Process any received data
        if line:
            m = DATA_RE.match(line)
            if m:
                (
                    unit_id, person_id, sos, lat, lon,
                    elev, heading, steps
                ) = m.groups()
                self.ingest_packet(
                    ts=ts,
                    unit_id=int(unit_id),
                    person_id=int(person_id),
                    sos=bool(int(sos)),
                    lat=float(lat),
                    lon=float(lon),
                    elev=float(elev),
                    heading=float(heading),
                    steps=int(steps),
                )
        
        # Schedule next poll
        self.root.after(UPDATE_INTERVAL, self.poll_serial)

    # ─── Packet handling --------------------------------------
    def ingest_packet(self, ts: float, **kw):
        key = f"U{kw['unit_id']}-P{kw['person_id']}"
        track = self.tracks.get(key) or self.create_unit(key)
        self.update_unit(track, ts=ts, **kw)

    # ─── Unit creation ----------------------------------------
    def create_unit(self, key: str):
        colour = next(PALETTE)
        icon_norm = build_icon(colour)
        icon_sos  = build_icon("#FF2D55")

        # Sidebar card
        card = ctk.CTkFrame(self.sidebar, corner_radius=6, fg_color="#1E1E1E")
        card.pack(fill="x", pady=6, padx=8)

        row = ctk.CTkFrame(card, fg_color="transparent")
        row.pack(fill="x", pady=(2, 0))

        lbl_key = ctk.CTkLabel(row, text=key, font=self.f_title, text_color=colour)
        lbl_key.pack(side="left")

        btn_center = ctk.CTkButton(
            row, text="Center", width=70, font=self.f_small,
            command=lambda k=key: self.center_on(k))
        btn_center.pack(side="right", padx=2)

        btn_toggle = ctk.CTkButton(
            row, text="Hide", width=70, font=self.f_small,
            command=lambda k=key: self.toggle_visibility(k))
        btn_toggle.pack(side="right", padx=2)

        lbl_data = ctk.CTkLabel(card, text="—", font=self.f_body,
                                anchor="w", justify="left")
        lbl_data.pack(fill="x", padx=4)

        lbl_age = ctk.CTkLabel(card, text="", font=self.f_small, anchor="w")
        lbl_age.pack(fill="x", padx=4, pady=(0, 4))

        track = {
            "colour": colour,
            "icon_norm": icon_norm,
            "icon_sos": icon_sos,
            "marker": None,
            "line": None,
            "arrow": None,
            "path": [],
            "last_ts": time.time(),
            "sidebar": {
                "card": card,
                "lbl_data": lbl_data,
                "lbl_age": lbl_age,
                "lbl_key": lbl_key,
                "btn_toggle": btn_toggle,
            },
            "visible": True,
        }
        self.tracks[key] = track
        return track

    # ─── Update unit ------------------------------------------
    def update_unit(self, t: Dict, *, ts: float, sos: bool,
                    lat: float, lon: float, elev: float,
                    heading: float, steps: int, **_):
        colour = t["colour"]
        t["last_ts"] = ts

        # Sidebar text
        t["sidebar"]["lbl_data"].configure(
            text=f"{heading_to_dir(heading)} {heading:.0f}°  "
                 f"alt:{elev:.0f}m\nsteps:{steps}  SOS:{'YES' if sos else 'no'}",
            text_color="#FF2D55" if sos else "white",
        )

        # Marker ------------------------------------------------
        if t["marker"] is None:
            t["marker"] = self.map.set_marker(
                lat, lon,
                icon=t["icon_sos"] if sos else t["icon_norm"],
                text="", text_color=colour)
        else:
            t["marker"].set_position(lat, lon)
            try:
                t["marker"].set_icon(t["icon_sos"] if sos else t["icon_norm"])
            except AttributeError:
                t["marker"].delete()
                t["marker"] = self.map.set_marker(
                    lat, lon,
                    icon=t["icon_sos"] if sos else t["icon_norm"],
                    text="", text_color=colour)

        # Path --------------------------------------------------
        t["path"].append((lat, lon))
        if len(t["path"]) > POINT_RETENTION:
            t["path"].pop(0)
        self.redraw_path(t)

        # SOS flash
        self.flash_card(t, sos)

    # ─── Draw / redraw path & arrow ---------------------------
    def redraw_path(self, t: Dict):
        pts = t["path"]
        if len(pts) < 2:
            return

        colour = t["colour"]

        # Path line
        if t["line"]:
            t["line"].delete()
        t["line"] = self.map.set_path(pts, width=PATH_WIDTH, color=colour)

        # Heading arrow (orange, constant length)
        lat, lon = pts[-1]
        prev_lat, prev_lon = pts[-2]
        heading_deg = math.degrees(math.atan2(lon - prev_lon, lat - prev_lat))
        h_lat, h_lon = project(lat, lon, heading_deg)
        arrow_pts = [(lat, lon), (h_lat, h_lon)]
        if t["arrow"]:
            t["arrow"].delete()
        t["arrow"] = self.map.set_path(
            arrow_pts, width=ARROW_WIDTH, color=ARROW_COLOR)

    # ─── Flash SOS background ---------------------------------
    def flash_card(self, t: Dict, sos: bool):
        card = t["sidebar"]["card"]
        if not sos:
            card.configure(fg_color="#1E1E1E")
            return

        def _pulse(state=[False]):
            if not t["visible"]:
                return
            state[0] = not state[0]
            card.configure(fg_color="#550000" if state[0] else "#400000")
            if sos:
                card.after(400, _pulse)

        if card.cget("fg_color") == "#1E1E1E":
            _pulse()

    # ─── Age label refresh ------------------------------------
    def refresh_age_labels(self):
        now = time.time()
        for t in self.tracks.values():
            age = int(now - t["last_ts"])
            t["sidebar"]["lbl_age"].configure(
                text=f"last: {age}s ago",
                text_color="#888" if age > STALE_AFTER_S else "#AAA")
        self.root.after(1000, self.refresh_age_labels)

    # ─── Centre map on unit -----------------------------------
    def center_on(self, key: str):
        t = self.tracks.get(key)
        if t and t["path"]:
            lat, lon = t["path"][-1]
            self.map.set_position(lat, lon)

    # ─── Toggle unit visibility -------------------------------
    def toggle_visibility(self, key: str):
        t = self.tracks.get(key)
        if not t:
            return
        vis = t["visible"]
        t["visible"] = not vis
        btn = t["sidebar"]["btn_toggle"]
        btn.configure(text="Hide" if not vis else "Show")

        elements = ("marker", "line", "arrow")
        if vis:          # hide
            for e in elements:
                obj = t.get(e)
                if obj:
                    obj.delete()
                    t[e] = None
            t["sidebar"]["lbl_key"].configure(text_color="#555555")
        else:            # show
            colour = t["colour"]
            pts = t["path"]
            if pts:
                lat, lon = pts[-1]
                t["marker"] = self.map.set_marker(
                    lat, lon, icon=t["icon_norm"],
                    text="", text_color=colour)
                if len(pts) >= 2:
                    t["line"] = self.map.set_path(
                        pts, width=PATH_WIDTH, color=colour)
                    prev_lat, prev_lon = pts[-2]
                    hdg = math.degrees(math.atan2(
                        lon - prev_lon, lat - prev_lat))
                    h_lat, h_lon = project(lat, lon, hdg)
                    t["arrow"] = self.map.set_path(
                        [(lat, lon), (h_lat, h_lon)],
                        width=ARROW_WIDTH, color=ARROW_COLOR)
            t["sidebar"]["lbl_key"].configure(text_color=colour)

    # ─── Cleanup ---------------------------------------------- 
    def cleanup(self):
        """Clean up resources when closing the application."""
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
                print("Serial port closed successfully")
            except Exception as e:
                print(f"Error closing serial port: {e}")

# ─── entry-point ──────────────────────────────────────────────
if __name__ == "__main__":
    root = ctk.CTk()
    app = TrackerApp(root)
    
    # Handle window close event
    def on_closing():
        app.cleanup()
        root.destroy()
    
    root.protocol("WM_DELETE_WINDOW", on_closing)
    root.mainloop()
