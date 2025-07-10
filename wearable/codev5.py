# wearable_tracker_v5_encrypted.py
# – GPS bootstrap ➜ DR, BNO08x self-cal, barometric height,
#   AES-CTR encryption (nonce ‖ ciphertext) over LoRa.

import time, math, struct, busio, digitalio, board
import adafruit_gps, adafruit_rfm9x
from adafruit_bno08x.i2c import BNO08X_I2C
import adafruit_bno08x, adafruit_bmp280, aesio

# ─── user settings ───────────────────────────────────────────
UNIT_ID   = 1
PERSON_ID = 1
SOS       = 1                 # wire a button later

STEP_LENGTH_M       = 0.77
CONTINUOUS_GPS      = False
GPS_FIXES_BEFORE_LOCK = 60

# ─── radio init ──────────────────────────────────────────────
RADIO_FREQ_MHZ = 868.0
CS, RESET = digitalio.DigitalInOut(board.RFM_CS), digitalio.DigitalInOut(board.RFM_RST)
rfm = adafruit_rfm9x.RFM9x(board.SPI(), CS, RESET, RADIO_FREQ_MHZ)
rfm.tx_power = 20

# ─── peripherals ─────────────────────────────────────────────
uart = busio.UART(board.TX, board.RX, baudrate=9600, timeout=100)
gps  = adafruit_gps.GPS(uart, debug=False)
gps.send_command(b"PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0")   # RMC+GGA
gps.send_command(b"PMTK220,500")                                     # 1 Hz

i2c   = busio.I2C(board.SCL, board.SDA, frequency=800_000)           # IMU
bno   = BNO08X_I2C(i2c)

i2c_bar = busio.I2C(scl=board.A3, sda=board.A2)                      # BMP280
bmp280  = adafruit_bmp280.Adafruit_BMP280_I2C(i2c_bar, address=0x76)

led = digitalio.DigitalInOut(board.D5)                               # status LED
led.direction = digitalio.Direction.OUTPUT

# ─── encryption params ───────────────────────────────────────
KEY   = b"0123456789abcdef"        # 16 B AES-128 key (shared)
NONCE = bytearray(16)              # increment per packet (last 4 B counter)

FMT  = "<BBBffffI"                 # struct: unit, person, sos, lat, lon, Δalt, hdg, steps

def encrypt_payload(plain: bytes) -> bytes:
    ctr = int.from_bytes(NONCE[-4:], "little") + 1
    NONCE[-4:] = ctr.to_bytes(4, "little")

    aes = aesio.AES(KEY, aesio.MODE_CTR, NONCE)
    cipher = bytearray(len(plain))
    aes.encrypt_into(plain, cipher)
    return bytes(NONCE) + cipher     # nonce prepended

# ─── calibration & baseline ──────────────────────────────────
def calibrate_bno():
    bno.enable_feature(adafruit_bno08x.BNO_REPORT_ACCELEROMETER)
    bno.enable_feature(adafruit_bno08x.BNO_REPORT_GYROSCOPE)
    bno.enable_feature(adafruit_bno08x.BNO_REPORT_MAGNETOMETER)
    bno.enable_feature(adafruit_bno08x.BNO_REPORT_ROTATION_VECTOR)
    bno.enable_feature(adafruit_bno08x.BNO_REPORT_STEP_COUNTER)
    bno.begin_calibration()
    calib_start = None
    while True:
        _ = bno.acceleration; _ = bno.gyro; _ = bno.magnetic; _ = bno.quaternion
        if bno.calibration_status == 3:
            if calib_start is None:
                calib_start = time.monotonic()
            elif time.monotonic() - calib_start >= 5.0:
                break
        else:
            calib_start = None
        led.value = not led.value
        time.sleep(0.5)
    bno.save_calibration_data()
    led.value = True

def baro_alt():
    try:
        alt = bmp280.altitude
        return alt, alt - BASELINE_ALT
    except Exception:
        return None, None

calibrate_bno()
print("Setting baseline altitude… hold steady.")
BASELINE_ALT = sum(bmp280.altitude for _ in range(5)) / 5
print("Baseline:", BASELINE_ALT, "m")

bno.enable_feature(adafruit_bno08x.BNO_REPORT_MAGNETOMETER)
bno.enable_feature(adafruit_bno08x.BNO_REPORT_STEP_COUNTER)

# ─── helpers ─────────────────────────────────────────────────
METRES_PER_DEG_LAT = 111_111
def heading_mag():
    mx, my, _ = bno.magnetic
    h = math.degrees(math.atan2(my, mx))
    return h + 360 if h < 0 else h

# ─── state ───────────────────────────────────────────────────
last_lat = last_lon = None
last_steps = 0
gps_fix_count = 0
dr_only = False

# ─── main loop ───────────────────────────────────────────────
while True:
    gps.update()
    steps   = bno.steps
    heading = heading_mag()

    alt, d_alt = baro_alt()
    if alt is None:
        d_alt = 0.0

    # position -------------------------------------------------
    if not dr_only and gps.has_fix and gps.latitude is not None:
        lat, lon = gps.latitude, gps.longitude
        last_lat, last_lon = lat, lon
        gps_fix_count += 1
        if (not CONTINUOUS_GPS) and gps_fix_count >= GPS_FIXES_BEFORE_LOCK:
            dr_only = True
            gps.send_command(b"PMTK161,0")      # standby
    elif last_lat is not None:
        step_delta = max(steps - last_steps, 0)
        dist = step_delta * STEP_LENGTH_M
        d_lat = dist * math.cos(math.radians(heading)) / METRES_PER_DEG_LAT
        d_lon = dist * math.sin(math.radians(heading)) / (
            METRES_PER_DEG_LAT * math.cos(math.radians(last_lat))
        )
        lat, lon = last_lat + d_lat, last_lon + d_lon
        last_lat, last_lon = lat, lon
    else:
        last_steps = steps
        time.sleep(1)
        continue

    last_steps = steps

    # payload --------------------------------------------------
    plain = struct.pack(
        FMT,
        UNIT_ID, PERSON_ID, SOS,
        float(lat), float(lon),
        float(d_alt), float(heading),
        steps,
    )
    rfm.send(encrypt_payload(plain))
    time.sleep(1)
