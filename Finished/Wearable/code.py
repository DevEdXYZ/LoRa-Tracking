import time, math, struct, busio, digitalio, board  # import core modules and hardware interfaces
import adafruit_gps  # library for GPS module
import adafruit_rfm9x  # library for LoRa radio
from adafruit_bno08x.i2c import BNO08X_I2C  # import IMU over I2C
import adafruit_bno08x  # library for IMU sensors
import adafruit_bmp280  # library for barometric sensor
import aesio  # library for AES encryption

# ─── user settings ───────────────────────────────────────────
UNIT_ID   = 1  # unique identifier for this device
PERSON_ID = 1  # identifier for the user wearing the device
SOS       = 1  # flag for emergency signal (button to be added later)

STEP_LENGTH_M       = 0.77  # average stride length in metres
CONTINUOUS_GPS      = False  # whether to keep GPS on continuously
GPS_FIXES_BEFORE_LOCK = 60  # number of GPS fixes before switching off GPS

# ─── radio initialisation ────────────────────────────────────
RADIO_FREQ_MHZ = 868.0  # LoRa frequency in MHz
CS, RESET = digitalio.DigitalInOut(board.RFM_CS), digitalio.DigitalInOut(board.RFM_RST)  # chip select and reset pins
rfm = adafruit_rfm9x.RFM9x(board.SPI(), CS, RESET, RADIO_FREQ_MHZ)  # set up LoRa radio
rfm.tx_power = 20  # set transmit power in dBm

# ─── peripherals setup ───────────────────────────────────────
uart = busio.UART(board.TX, board.RX, baudrate=9600, timeout=100)  # UART for GPS
gps  = adafruit_gps.GPS(uart, debug=False)  # GPS instance
gps.send_command(b"PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0")  # enable RMC and GGA messages
gps.send_command(b"PMTK220,500")  # set GPS update rate to 1 Hz

i2c   = busio.I2C(board.SCL, board.SDA, frequency=800_000)  # I2C bus for IMU
bno   = BNO08X_I2C(i2c)  # initialise IMU on I2C

i2c_bar = busio.I2C(scl=board.A3, sda=board.A2)  # I2C bus for barometer
bmp280  = adafruit_bmp280.Adafruit_BMP280_I2C(i2c_bar, address=0x76)  # initialise BMP280

led = digitalio.DigitalInOut(board.D5)  # status LED pin
led.direction = digitalio.Direction.OUTPUT  # set LED as output

# ─── encryption parameters ───────────────────────────────────
KEY   = b"0123456789abcdef"  # 16-byte AES-128 key
NONCE = bytearray(16)  # counter for encryption

FMT  = "<BBBffffI"  # data format: unit, person, sos, lat, lon, altitude change, heading, steps

def encrypt_payload(plain: bytes) -> bytes:  # encrypt data payload
    ctr = int.from_bytes(NONCE[-4:], "little") + 1  # increment counter
    NONCE[-4:] = ctr.to_bytes(4, "little")  # update counter in nonce

    aes = aesio.AES(KEY, aesio.MODE_CTR, NONCE)  # create AES-CTR cipher
    cipher = bytearray(len(plain))  # buffer for ciphertext
    aes.encrypt_into(plain, cipher)  # perform encryption
    return bytes(NONCE) + cipher  # send counter plus encrypted data

# ─── sensor calibration and baseline ──────────────────────────
def calibrate_bno():  # calibrate IMU sensors
    bno.enable_feature(adafruit_bno08x.BNO_REPORT_ACCELEROMETER)  # enable accelerometer
    bno.enable_feature(adafruit_bno08x.BNO_REPORT_GYROSCOPE)  # enable gyroscope
    bno.enable_feature(adafruit_bno08x.BNO_REPORT_MAGNETOMETER)  # enable magnetometer
    bno.enable_feature(adafruit_bno08x.BNO_REPORT_ROTATION_VECTOR)  # enable orientation
    bno.enable_feature(adafruit_bno08x.BNO_REPORT_STEP_COUNTER)  # enable step counting
    bno.begin_calibration()  # start calibration process
    calib_start = None  # time marker for full calibration
    while True:
        _ = bno.acceleration; _ = bno.gyro; _ = bno.magnetic; _ = bno.quaternion  # read sensors
        if bno.calibration_status == 3:  # fully calibrated
            if calib_start is None:
                calib_start = time.monotonic()  # record start time
            elif time.monotonic() - calib_start >= 5.0:
                break  # keep stable for 5 seconds
        else:
            calib_start = None  # reset if status drops
        led.value = not led.value  # flash LED
        time.sleep(0.5)  # pause
    bno.save_calibration_data()  # store calibration
    led.value = True  # turn LED on when done


def baro_alt():  # read barometric altitude
    try:
        alt = bmp280.altitude  # get altitude
        return alt, alt - BASELINE_ALT  # return absolute and relative altitude
    except Exception:
        return None, None  # if read fails, return None

calibrate_bno()  # run IMU calibration
print("Setting baseline altitude… hold still.")  # prompt user
BASELINE_ALT = sum(bmp280.altitude for _ in range(5)) / 5  # average initial altitude
print(f"Baseline: {BASELINE_ALT:.2f} m")  # show baseline

bno.enable_feature(adafruit_bno08x.BNO_REPORT_MAGNETOMETER)  # re-enable magnetometer
bno.enable_feature(adafruit_bno08x.BNO_REPORT_STEP_COUNTER)  # re-enable step counter

# ─── helper functions ────────────────────────────────────────
METRES_PER_DEG_LAT = 111_111  # metres per degree latitude

def heading_mag():  # calculate heading from magnetometer
    mx, my, _ = bno.magnetic  # read magnetic field
    h = math.degrees(math.atan2(my, mx))  # get angle in degrees
    return h + 360 if h < 0 else h  # normalise to 0–360°

# ─── state variables ─────────────────────────────────────────
last_lat = last_lon = None  # store last known position
last_steps = 0  # store last step count
gps_fix_count = 0  # count GPS fixes
dr_only = False  # flag for dead reckoning only

# ─── main loop ───────────────────────────────────────────────
while True:
    gps.update()  # fetch new GPS data
    steps   = bno.steps  # get total steps
    heading = heading_mag()  # calculate current heading

    alt, d_alt = baro_alt()  # get altitude and change
    if alt is None:
        d_alt = 0.0  # if no data, set change to zero

    # determine position -------------------------------------
    if not dr_only and gps.has_fix and gps.latitude is not None:
        lat, lon = gps.latitude, gps.longitude  # use GPS position
        last_lat, last_lon = lat, lon  # update last position
        gps_fix_count += 1  # increment fix counter
        if (not CONTINUOUS_GPS) and gps_fix_count >= GPS_FIXES_BEFORE_LOCK:
            dr_only = True  # switch to dead reckoning
            gps.send_command(b"PMTK161,0")  # turn off GPS
    elif last_lat is not None:
        step_delta = max(steps - last_steps, 0)  # steps since last update
        dist = step_delta * STEP_LENGTH_M  # distance walked
        d_lat = dist * math.cos(math.radians(heading)) / METRES_PER_DEG_LAT  # change in latitude
        d_lon = dist * math.sin(math.radians(heading)) / (
            METRES_PER_DEG_LAT * math.cos(math.radians(last_lat))
        )  # change in longitude
        lat, lon = last_lat + d_lat, last_lon + d_lon  # estimated position
        last_lat, last_lon = lat, lon  # update last position
    else:
        last_steps = steps  # record steps and wait
        time.sleep(1)
        continue  # try again next loop

    last_steps = steps  # update step count

    # prepare and send data ----------------------------------
    plain = struct.pack(
        FMT,
        UNIT_ID, PERSON_ID, SOS,
        float(lat), float(lon),
        float(d_alt), float(heading),
        steps,
    )  # pack data into bytes
    rfm.send(encrypt_payload(plain))  # encrypt and transmit data
    time.sleep(1)  # wait before next cycle
