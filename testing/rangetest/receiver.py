"""
LoRa RFM9x receiver – waits for “range test” packets, shows RSSI
"""

import time
import board
import digitalio
import adafruit_rfm9x

RADIO_FREQ_MHZ = 868.0

# Radio control pins
CS    = digitalio.DigitalInOut(board.RFM_CS)
RESET = digitalio.DigitalInOut(board.RFM_RST)

# Optional on-board LED for visual feedback
LED = digitalio.DigitalInOut(board.LED)
LED.direction = digitalio.Direction.OUTPUT

# Initialise radio
rfm9x = adafruit_rfm9x.RFM9x(board.SPI(), CS, RESET, RADIO_FREQ_MHZ)

print("Receiver ready – listening…")

while True:
    packet = rfm9x.receive(timeout=5.0)   # seconds

    if packet is None:
        LED.value = False
        print("-- no packet --")
        continue

    # Packet arrived
    LED.value = True
    timestamp = time.monotonic()

    try:
        text = packet.decode().strip()
    except UnicodeError:
        text = "<non-ASCII>"

    rssi = rfm9x.last_rssi   # dB-m

    print(f"[{timestamp:>10.3f}s] RSSI {rssi:>4} dB  Payload: {text}")
