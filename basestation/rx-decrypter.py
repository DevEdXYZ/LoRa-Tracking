# lora_basestation_listener_v3.py
# – Receives LoRa packets, decrypts AES-CTR, parses struct, prints GUI-friendly line.

import struct, board, digitalio, adafruit_rfm9x, aesio

# ─── radio init ──────────────────────────────────────────────
RADIO_FREQ_MHZ = 868.0
CS    = digitalio.DigitalInOut(board.RFM_CS)
RESET = digitalio.DigitalInOut(board.RFM_RST)
rfm   = adafruit_rfm9x.RFM9x(board.SPI(), CS, RESET, RADIO_FREQ_MHZ)
rfm.sleep(); rfm.receive()

# ─── decryption params ──────────────────────────────────────
KEY  = b"0123456789abcdef"
FMT  = "<BBBffffI"                     # wearable payload
PLEN = struct.calcsize(FMT)

print("LoRa listener ready")

while True:
    pkt = rfm.receive(timeout=0.5)
    if pkt is None or len(pkt) < 16 + PLEN:
        continue

    nonce, ciphertext = pkt[:16], pkt[16:]
    try:
        aes    = aesio.AES(KEY, aesio.MODE_CTR, nonce)
        plain  = bytearray(len(ciphertext))
        aes.decrypt_into(ciphertext, plain)

        unit, person, sos, lat, lon, d_alt, heading, steps = struct.unpack(FMT, plain)

        # ---- line for the desktop GUI -----------------------
        print(
            "Data: {} {} {} {:.6f} {:.6f} {:.1f} {:.1f} {}".format(
                unit, person, sos, lat, lon, d_alt, heading, steps
            )
        )

    except Exception as e:
        # corrupt packet → ignore
        continue
