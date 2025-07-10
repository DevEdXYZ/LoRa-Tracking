import board
import busio
import digitalio
import adafruit_rfm9x

# Define radio frequency
RADIO_FREQ_MHZ = 868.0  # Must match transmitter's frequency

# Define radio control pins
CS = digitalio.DigitalInOut(board.RFM_CS)
RESET = digitalio.DigitalInOut(board.RFM_RST)

# Define onboard LED for visual feedback
LED = digitalio.DigitalInOut(board.LED)
LED.direction = digitalio.Direction.OUTPUT

# Initialize SPI interface
spi = board.SPI()  # Default SPI bus (SCK, MOSI, MISO)

# Initialize the RFM95 radio
rfm9x = adafruit_rfm9x.RFM9x(spi, CS, RESET, RADIO_FREQ_MHZ)

print("LoRa Receiver ready and listening...")

while True:
    packet = rfm9x.receive(timeout=5.0)

    if packet is None:
        LED.value = False
        print("No packet received.")
    else:
        LED.value = True
        try:
            packet_text = packet.decode("utf-8").strip()
            print("Received (ASCII): {}".format(packet_text))
        except Exception:
            print("Received undecodable data:")
            print("Raw bytes: {}".format(packet))
            print("Hex: {}".format(' '.join('{:02X}'.format(b) for b in packet)))

        print("Signal strength: {} dB".format(rfm9x.last_rssi))

