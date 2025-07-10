# lora_basestation_listener_v2b.py
# – Receives LoRa packets, decrypts AES-CTR, parses struct, prints one-line summary.

import time, struct, board, busio, digitalio
import adafruit_rfm9x, aesio

# ─── radio setup ─────────────────────────────────────────────
RADIO_FREQ_MHZ = 868.0
CS    = digitalio.DigitalInOut(board.RFM_CS)
RESET = digitalio.DigitalInOut(board.RFM_RST)

rfm = adafruit_rfm9x.RFM9x(board.SPI(), CS, RESET, RADIO_FREQ_MHZ)
rfm.tx_power = 5
rfm.sleep()
rfm.receive()              # clear buffer

# ─── decryption parameters ──────────────────────────────────
KEY = b"0123456789abcdef"   # must match the wearable
FMT = "<BBBffffI"
PLAINTEXT_LEN = struct.calcsize(FMT)

print("LoRa listener ready")

# ─── main loop ───────────────────────────────────────────────
while True:
    pkt = rfm.receive(timeout=0.5)
    if pkt is None or len(pkt) < 16 + PLAINTEXT_LEN:
        continue

    nonce      = pkt[:16]
    ciphertext = pkt[16:]

    try:
        cipher       = aesio.AES(KEY, aesio.MODE_CTR, nonce)
        plaintext    = bytearray(len(ciphertext))
        cipher.decrypt_into(ciphertext, plaintext)

        unit, person, sos, lat, lon, d_alt, heading, steps = struct.unpack(FMT, plaintext)

        print(
            "U{} P{} SOS={} lat={:.6f} lon={:.6f} Δalt={:.1f}m head={:.1f}° steps={}".format(
                unit, person, sos, lat, lon, d_alt, heading, steps
            )
        )

    except Exception as e:
        print("Packet error:", e)
