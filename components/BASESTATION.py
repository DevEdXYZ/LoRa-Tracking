import re
import serial
import customtkinter as ctk
from tkintermapview import TkinterMapView
from PIL import Image, ImageTk

#serial configuration
SERIAL_PORT   = "COM4"      # change to match your USB port
BAUDRATE      = 115200
TIMEOUT_S     = 0.1          # non‑blocking read

#display / update settings
UPDATE_INTERVAL = 100        # ms  ‑ tkinter after() cadence
ZOOM_LEVEL      = 10       # initial map zoom

#regex for “Data: lat lon heading steps”  :  "Data: 51.5014 -0.1419 135.0 1248"
DATA_RE = re.compile(r"Data:\s+(-?\d+\.\d+)\s+(-?\d+\.\d+)\s+(-?\d+\.\d+)\s+(\d+)")

DIRS = ("N", "NE", "E", "SE", "S", "SW", "W", "NW")

# Maps a degree angle to the nearest 8-point compass direction (e.g., N, NE, E, ...).
def heading_to_dir(deg: float) -> str:
    """Return 8‑point compass direction."""
    return DIRS[int(((deg + 22.5) // 45) % 8)]


class TrackerApp:
    def __init__(self, root):
        self.root = root
        root.title("LoRa Live Tracker")
        ctk.set_appearance_mode("dark")
        ctk.set_default_color_theme("dark-blue")

        #widgets
        frame = ctk.CTkFrame(root)
        frame.pack(fill="both", expand=True)

        self.map = TkinterMapView(frame, width=1280, height=720, corner_radius=1)
        self.map.pack(fill="both", expand=True)

        self.tracks = {}
        marker_img = Image.open("dot.png").resize((20, 20))
        self.marker_img_tk = ImageTk.PhotoImage(marker_img)

        #serial port
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=TIMEOUT_S)
        except serial.SerialException as e:
            ctk.CTkMessagebox(title="Serial error", message=str(e))
            raise SystemExit from e

        # Dummy centre until first packet arrives
        self.map.set_position(51.5072256 , -0.1279288)
        self.map.set_zoom(ZOOM_LEVEL)

        # schedule first poll
        self.root.after(UPDATE_INTERVAL, self.poll_serial)

    #data handling
    def poll_serial(self):
        try:
            line = self.ser.readline().decode().strip()
        except UnicodeDecodeError:
            line = ""

        if line:
            match = DATA_RE.match(line)
            if match:
                lat, lon, heading, steps = match.groups()
                lat, lon, heading, steps = float(lat), float(lon), float(heading), int(steps)
                self.update_track(lat, lon, heading, steps)

        self.root.after(UPDATE_INTERVAL, self.poll_serial)

    def update_track(self, lat: float, lon: float, heading: float, steps: int):
        key = "unit-01"  # single tracker; extend if multiple IDs are encoded
        track = self.tracks.get(key)

        label = f"{heading_to_dir(heading)} {heading:.0f}\u00B0\nsteps:{steps}"  # on‑marker text

        if track is None:
            marker = self.map.set_marker(
                lat, lon,
                text=label,
                font=("Consolas", 13, "bold"),
                text_color="#D36820",
                icon=self.marker_img_tk,
            )
            self.tracks[key] = {"marker": marker, "path": [(lat, lon)], "line": None}
            self.map.set_position(lat, lon)  # centre on first packet
            return

        # update marker + path
        track["marker"].set_position(lat, lon)
        track["marker"].set_text(label)
        track["path"].append((lat, lon))

        if len(track["path"]) >= 2:
            if track["line"] is None:
                track["line"] = self.map.set_path(track["path"], width=2)
            else:
                track["line"].delete()
                track["line"] = self.map.set_path(track["path"], width=2)


if __name__ == "__main__":
    root = ctk.CTk()
    TrackerApp(root)
    root.mainloop()
