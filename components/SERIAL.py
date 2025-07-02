import serial
import time

#serial config
SERIAL_PORT = "COM4"     #change to match USB USED
BAUDRATE    = 115200
TIMEOUT_S   = 0.1        # non-blocking read

#open serial
try:
    ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=TIMEOUT_S)
except serial.SerialException as e:
    print("Serial error:", e)
    exit()

#loop
while True:
    line = ser.readline().decode(errors="ignore").strip()
    if line:
        print(line)
    time.sleep(0.05)
