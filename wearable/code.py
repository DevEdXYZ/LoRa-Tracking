import time
import math
import board
import busio
import digitalio
import adafruit_gps
from adafruit_bno08x.i2c import BNO08X_I2C
import adafruit_bno08x
import adafruit_rfm9x

# --- Radio configuration ---
RADIO_FREQ_MHZ = 868.0
CS = digitalio.DigitalInOut(board.RFM_CS)
RESET = digitalio.DigitalInOut(board.RFM_RST)
rfm95 = adafruit_rfm9x.RFM9x(board.SPI(), CS, RESET, RADIO_FREQ_MHZ)
rfm95.tx_power = 5  # lowest power

# --- GPS configuration ---
uart = busio.UART(board.TX, board.RX, baudrate=9600, timeout=1000)
gps = adafruit_gps.GPS(uart, debug=False)
gps.send_command(b"PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0")
gps.send_command(b"PMTK220,1000")  # 1 Hz update rate

# --- IMU configuration ---
i2c = busio.I2C(board.SCL, board.SDA, frequency=800_000)
bno = BNO08X_I2C(i2c)
bno.enable_feature(adafruit_bno08x.BNO_REPORT_MAGNETOMETER)
bno.enable_feature(adafruit_bno08x.BNO_REPORT_STEP_COUNTER)

# --- Tracking state ---
STEP_LENGTH_M = 0.75  # approximate step length
last_lat = None
last_lon = None
last_steps = 0

def calculate_heading() -> float:
    """Return heading in degrees from magnetometer."""
    mx, my, _ = bno.magnetic
    heading = math.degrees(math.atan2(my, mx))
    if heading < 0:
        heading += 360.0
    return heading

while True:
    gps.update()
    steps = bno.steps
    heading = calculate_heading()

    if gps.has_fix:
        lat = gps.latitude
        lon = gps.longitude
        last_lat, last_lon = lat, lon
    elif last_lat is not None:
        step_delta = steps - last_steps
        if step_delta < 0:
            step_delta = 0
        distance = step_delta * STEP_LENGTH_M
        delta_lat = distance * math.cos(math.radians(heading)) / 111111
        delta_lon = distance * math.sin(math.radians(heading)) / (
            111111 * math.cos(math.radians(last_lat))
        )
        last_lat += delta_lat
        last_lon += delta_lon
        lat, lon = last_lat, last_lon
    else:
        lat = lon = None

    last_steps = steps

    if lat is not None:
        message = f"Data: {lat:.6f} {lon:.6f} {heading:.1f} {steps}"
    else:
        message = f"NoFix {heading:.1f} {steps}"

    rfm95.send(bytes(message, "utf-8"))
    time.sleep(1)
