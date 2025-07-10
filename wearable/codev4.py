# wearable_tracker_v4.py – GPS-bootstrap, then dead-reckoned, with BNO08x self-calibration and barometric height
import time, math, busio, digitalio, board
import adafruit_gps, adafruit_rfm9x
from adafruit_bno08x.i2c import BNO08X_I2C
import adafruit_bno08x
import adafruit_bmp280

# ─── user settings ───────────────────────────────────────────
UNIT_ID   = 1
PERSON_ID = 1
SOS       = 1                     # wire a button later

STEP_LENGTH_M = 0.77              # stride estimate
CONTINUOUS_GPS = False            # set True for normal mode
GPS_FIXES_BEFORE_LOCK = 60        # fixes to average before switch

# ─── hardware init ───────────────────────────────────────────
RADIO_FREQ_MHZ = 868.0
CS, RESET = digitalio.DigitalInOut(board.RFM_CS), digitalio.DigitalInOut(board.RFM_RST)
rfm = adafruit_rfm9x.RFM9x(board.SPI(), CS, RESET, RADIO_FREQ_MHZ)
rfm.tx_power = 20

uart = busio.UART(board.TX, board.RX, baudrate=9600, timeout=100)
gps  = adafruit_gps.GPS(uart, debug=False)
gps.send_command(b"PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0")   # RMC+GGA
gps.send_command(b"PMTK220,500")   # 1 Hz

# IMU on primary I2C
i2c = busio.I2C(board.SCL, board.SDA, frequency=800_000)
bno = BNO08X_I2C(i2c)

# BMP280 on spare I2C pins (A2/A3).  Change pins if you have everything on the main bus.
i2c_bar = busio.I2C(scl=board.A3, sda=board.A2)
bmp280  = adafruit_bmp280.Adafruit_BMP280_I2C(i2c_bar, address=0x76)
# Uncomment and adjust if you have a local QNH value available
# bmp280.sea_level_pressure = 1013.25 

# LED on D5 gives visual feedback during calibration
led = digitalio.DigitalInOut(board.D5)
led.direction = digitalio.Direction.OUTPUT
led.value = False

# ─── BNO08x calibration helper ───────────────────────────────
def calibrate_bno(bno, led):
    """Run on-the-fly self-calibration and persist offsets."""
    bno.enable_feature(adafruit_bno08x.BNO_REPORT_ACCELEROMETER)
    bno.enable_feature(adafruit_bno08x.BNO_REPORT_GYROSCOPE)
    bno.enable_feature(adafruit_bno08x.BNO_REPORT_MAGNETOMETER)
    bno.enable_feature(adafruit_bno08x.BNO_REPORT_ROTATION_VECTOR)
    bno.enable_feature(adafruit_bno08x.BNO_REPORT_STEP_COUNTER)

    bno.begin_calibration()
    print("→ Starting calibration. Move the sensor through all axes…")

    calib_start = None
    while True:
        # Force fresh samples; this tickles the SH-2 to update calib bits
        _ = bno.acceleration
        _ = bno.gyro
        _ = bno.magnetic
        _ = bno.quaternion

        status = bno.calibration_status  # 0-3, want 3
        print(f"Cal status = {status}")

        if status == 3:
            # Full calib – need to hold it for 5 s for good measure
            if calib_start is None:
                calib_start = time.monotonic()
            elif time.monotonic() - calib_start >= 5.0:
                break
        else:
            calib_start = None  # dropped below full - restart timer

        # Blink 1 Hz so user sees it's still running
        led.value = True
        time.sleep(0.1)
        led.value = False
        time.sleep(0.9)

    bno.save_calibration_data()
    led.value = True   # solid LED → ready
    print("✓ Calibration complete. Offsets saved.")

# ─── barometer helpers ───────────────────────────────────────
def get_barometric_altitude():
    """Return (altitude_m, delta_from_baseline_m)."""
    try:
        alt = bmp280.altitude
        return alt, alt - BASELINE_ALT
    except Exception as e:
        print(f"BMP280 read error: {e}")
        return None, None

# ─── run one-time self-cal, then establish altitude baseline ─
calibrate_bno(bno, led)

print("Setting baseline altitude… Hold device steady.")
BASELINE_ALT = sum(bmp280.altitude for _ in range(5)) / 5
print(f"Baseline: {BASELINE_ALT:.1f} m")

# For runtime we just need magnetometer + step counter
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

    # Altitude from barometer
    altitude_m, delta_alt_m = get_barometric_altitude()
    if altitude_m is None:
        # fallback if sensor failed
        delta_alt_m = 0.0

    # ── positioning ───────────────────────────────────────
    if not dr_only and gps.has_fix and gps.latitude is not None:
        # Good fix
        lat, lon = gps.latitude, gps.longitude
        last_lat, last_lon = lat, lon
        gps_fix_count += 1

        if (not CONTINUOUS_GPS) and gps_fix_count >= GPS_FIXES_BEFORE_LOCK:
            dr_only = True                    # lock position
            gps.send_command(b"PMTK161,0")  # standby saves ~20 mA

    elif last_lat is not None:
        # Dead-reckon
        step_delta = max(steps - last_steps, 0)
        dist_m = step_delta * STEP_LENGTH_M
        d_lat = (dist_m * math.cos(math.radians(heading))) / METRES_PER_DEG_LAT
        d_lon = (dist_m * math.sin(math.radians(heading))) / (
            METRES_PER_DEG_LAT * math.cos(math.radians(last_lat))
        )
        lat, lon = last_lat + d_lat, last_lon + d_lon
        last_lat, last_lon = lat, lon
    else:
        # No initial fix yet
        last_steps = steps
        time.sleep(1)
        continue

    last_steps = steps

    pkt = "Data: {} {:02d} {} {:.6f} {:.6f} {:.1f} {:.1f} {}".format(
              UNIT_ID, PERSON_ID, SOS, lat, lon, delta_alt_m, heading, steps)
    rfm.send(bytes(pkt, "utf-8"))
    # print(pkt)  # uncomment for USB debug

    time.sleep(1)

