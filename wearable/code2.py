# wearable_tracker_v2.py  –  GPS-bootstrap, then step-only
import time, math, busio, digitalio, board
import adafruit_gps, adafruit_rfm9x
from adafruit_bno08x.i2c import BNO08X_I2C
import adafruit_bno08x

# ─── user settings ───────────────────────────────────────────
UNIT_ID   = 1
PERSON_ID = 1
SOS       = 1                     # wire a button later

STEP_LENGTH_M = 0.77              # stride estimate
CONTINUOUS_GPS = False            # set True for normal mode
GPS_FIXES_BEFORE_LOCK = 60        # fixes to average before switch

# ─── hardware init (unchanged) ───────────────────────────────
RADIO_FREQ_MHZ = 868.0
CS, RESET = digitalio.DigitalInOut(board.RFM_CS), digitalio.DigitalInOut(board.RFM_RST)
rfm = adafruit_rfm9x.RFM9x(board.SPI(), CS, RESET, RADIO_FREQ_MHZ)
rfm.tx_power = 20

uart = busio.UART(board.TX, board.RX, baudrate=9600, timeout=100)
gps  = adafruit_gps.GPS(uart, debug=False)
gps.send_command(b"PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0")   # RMC+GGA
gps.send_command(b"PMTK220,500")   # 1 Hz

i2c = busio.I2C(board.SCL, board.SDA, frequency=800_000)
bno = BNO08X_I2C(i2c)
bno.enable_feature(adafruit_bno08x.BNO_REPORT_MAGNETOMETER)
bno.enable_feature(adafruit_bno08x.BNO_REPORT_STEP_COUNTER)

# ─── helpers ─────────────────────────────────────────────────
METRES_PER_DEG_LAT = 111_111
def heading_mag():
    mx, my, _ = bno.magnetic
    deg = math.degrees(math.atan2(my, mx))
    return deg + 360 if deg < 0 else deg

# ─── runtime state ───────────────────────────────────────────
last_lat = last_lon = None
last_steps = 0
gps_fix_count = 0
dr_only = False    # becomes True after the bootstrap phase

# ─── main loop ───────────────────────────────────────────────
while True:
    gps.update()
    steps   = bno.steps
    heading = heading_mag()

    if not dr_only and gps.has_fix and gps.latitude is not None:
        # good fix
        lat, lon = gps.latitude, gps.longitude
        last_lat, last_lon = lat, lon
        gps_fix_count += 1

        if (not CONTINUOUS_GPS) and gps_fix_count >= GPS_FIXES_BEFORE_LOCK:
            dr_only = True                    # lock position
            # optional: put GPS to standby, saves ~20 mA
            gps.send_command(b"PMTK161,0")

    elif last_lat is not None:
        # dead-reckon
        step_delta = max(steps - last_steps, 0)
        dist_m = step_delta * STEP_LENGTH_M
        d_lat = (dist_m * math.cos(math.radians(heading))) / METRES_PER_DEG_LAT
        d_lon = (dist_m * math.sin(math.radians(heading))) / (
            METRES_PER_DEG_LAT * math.cos(math.radians(last_lat))
        )
        lat, lon = last_lat + d_lat, last_lon + d_lon
        last_lat, last_lon = lat, lon
    else:
        # no initial fix yet
        last_steps = steps
        time.sleep(1)
        continue

    last_steps = steps

    pkt = f"Data: {UNIT_ID} {PERSON_ID:02d} {SOS} {lat:.6f} {lon:.6f} 0.0 {heading:.1f} {steps}"
    rfm.send(bytes(pkt, "utf-8"))
    # print(pkt)  # uncomment for USB debug

    time.sleep(1)

