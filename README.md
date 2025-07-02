# LoRa-Tracking

This repository contains a simple LoRa based tracking system.

The wearable unit obtains a GPS fix when available and periodically
transmits its position, heading and step count to a base station.
The receiver displays the last known location on a Tkinter map.

## Hardware

- Feather RP2040 with RFM95 LoRa module
- Adafruit Ultimate GPS
- Adafruit BNO085 9‑DoF IMU

## Directory layout

- `wearable/code.py` – CircuitPython firmware for the tracker
- `basestation/code.py` – desktop application showing the received location
