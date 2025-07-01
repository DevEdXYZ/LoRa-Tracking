import board
import digitalio
import adafruit_rfm9x


# Define radio frequency in MHz. Must match the frequency of the RFM95W radio
RADIO_FREQ_MHZ = 868.0

# Define Chip Select and Reset pins for the radio module.
CS = digitalio.DigitalInOut(board.RFM_CS)
RESET = digitalio.DigitalInOut(board.RFM_RST)

# Initialise RFM95 radio
rfm95 = adafruit_rfm9x.RFM9x(board.SPI(), CS, RESET, RADIO_FREQ_MHZ)
rfm95.txpower(0)


while True:
    # Look for a new packet - wait up to 5 seconds:
    packet = rfm95.receive(timeout=5.0)

    # If no packet was received during the timeout then None is returned.
    if packet is not None:

        # Print out the text of the packet:
        print("Received (ASCII): {0}".format(packet.decode("utf-8")))
