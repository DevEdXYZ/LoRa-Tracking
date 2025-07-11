"""
LoRa RFM9x transmitter – repeats “range test” once per second
TX power: 20 dBm (max for RFM95/RFM96)
"""

import time
import board
import digitalio
import adafruit_rfm9x

RADIO_FREQ_MHZ = 868.0             # Must match receiver
TX_POWER_DBM   = 20                # 5-23 dBm allowed

# Radio control pins
CS    = digitalio.DigitalInOut(board.RFM_CS)
RESET = digitalio.DigitalInOut(board.RFM_RST)

# Initialise radio
rfm9x = adafruit_rfm9x.RFM9x(board.SPI(), CS, RESET, RADIO_FREQ_MHZ)
rfm9x.tx_power = TX_POWER_DBM

print("Transmitter ready – sending “range test” every 1 s")

sequence = 0
while True:
    payload = b"range test"
    rfm9x.send(payload)
    timestamp = time.monotonic()
    print(f"[{timestamp:>10.3f}s] Sent #{sequence} (len {len(payload)}) at {TX_POWER_DBM} dBm")
    sequence += 1
    time.sleep(1)
