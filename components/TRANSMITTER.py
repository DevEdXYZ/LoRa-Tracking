import board
import digitalio
import adafruit_rfm9x
import time

#radio freq in MHz - must match receiver
RADIO_FREQ_MHZ = 868.0

#setup radio chip select and reset pins
CS    = digitalio.DigitalInOut(board.RFM_CS)
RESET = digitalio.DigitalInOut(board.RFM_RST)

#init radio
rfm95 = adafruit_rfm9x.RFM9x(board.SPI(), CS, RESET, RADIO_FREQ_MHZ)
rfm95.tx_power = 5 #5dBm transmit power lowst power setting


#message to send
message = ("Hello, world! ")

#send loop
while True:
    rfm95.send(bytes(f"{message}", "UTF-8"))
    time.sleep(1) #wait 1 second between sends

