"""
Simple LoRa Base Station Receiver - Version 3
==============================================
This programme runs on a feather rp2040 with a LoRa radio module.
It listens for encrypted location data from tracking devices, decrypts it,
and prints it in a format that the map programme can understand.

What this programme does:
• Receives radio packets from LoRa tracking devices
• Decrypts the encrypted data using AES encryption
• Converts the binary data into readable numbers
• Prints the location data for the map programme to read
"""

# Import all the libraries we need for this programme
import struct        # For converting binary data to numbers
import board         # For accessing the Raspberry Pi Pico pins
import digitalio     # For controlling digital pins
import adafruit_rfm9x  # For the LoRa radio module
import aesio         # For decrypting encrypted data


# ═══════════════════════════════════════════════════════════════════════════════
# RADIO SETUP - Configure the LoRa radio module
# ═══════════════════════════════════════════════════════════════════════════════

# Set the radio frequency (must match the transmitting devices)
RADIO_FREQUENCY_MHZ = 868.0  # Ofcom frequency for LoRa in the UK

# Set up the control pins for the LoRa radio module
CHIP_SELECT_PIN = digitalio.DigitalInOut(board.RFM_CS)    # Pin to select the radio chip
RESET_PIN = digitalio.DigitalInOut(board.RFM_RST)         # Pin to reset the radio chip

# Create the radio object and configure it
lora_radio = adafruit_rfm9x.RFM9x(board.SPI(), CHIP_SELECT_PIN, RESET_PIN, RADIO_FREQUENCY_MHZ)

# Put the radio to sleep first, then wake it up in receive mode
lora_radio.sleep()      # Save power when not actively transmitting
lora_radio.receive()    # Start listening for incoming packets


# ═══════════════════════════════════════════════════════════════════════════════
# DECRYPTION SETUP - Configure how to decrypt the received data
# ═══════════════════════════════════════════════════════════════════════════════

# The secret key used to decrypt the data (must match the transmitting devices)
DECRYPTION_KEY = b"0123456789abcdef"  # 16-byte key for AES encryption

# Format string that describes the structure of the data packet
# This tells us how to interpret the binary data from the tracking device
DATA_FORMAT = "<BBBffffI"
# What each letter means:
# < = little-endian byte order (least significant byte first)
# B = unsigned byte (0-255) for unit ID
# B = unsigned byte (0-255) for person ID  
# B = unsigned byte (0-255) for SOS flag
# f = 32-bit float for latitude
# f = 32-bit float for longitude
# f = 32-bit float for altitude change
# f = 32-bit float for heading/direction
# I = 32-bit unsigned integer for step count

# Calculate how many bytes the data packet should be
EXPECTED_PACKET_LENGTH = struct.calcsize(DATA_FORMAT)

# Let the user know the programme is ready
print("LoRa base station receiver is ready and listening...")


# ═══════════════════════════════════════════════════════════════════════════════
# MAIN LISTENING LOOP - Keep listening for packets forever
# ═══════════════════════════════════════════════════════════════════════════════

while True:
    # Try to receive a packet from the radio
    # Wait up to 0.5 seconds for a packet to arrive
    received_packet = lora_radio.receive(timeout=0.5)
    
    # Check if we actually received something
    if received_packet is None:
        continue  # No packet received, try again
    
    # Check if the packet is long enough to contain our data
    # We need at least 16 bytes for the encryption nonce plus our data
    minimum_packet_size = 16 + EXPECTED_PACKET_LENGTH
    if len(received_packet) < minimum_packet_size:
        continue  # Packet is too short, probably not from our devices
    
    # Split the packet into two parts:
    # - First 16 bytes: the nonce (random number used for encryption)
    # - Rest of the packet: the encrypted data
    encryption_nonce = received_packet[:16]           # First 16 bytes
    encrypted_data = received_packet[16:]             # Everything after byte 16
    
    try:
        # Try to decrypt the data
        # Create an AES decryption object using our key and the nonce
        aes_decryptor = aesio.AES(DECRYPTION_KEY, aesio.MODE_CTR, encryption_nonce)
        
        # Create a buffer to hold the decrypted data
        decrypted_data = bytearray(len(encrypted_data))
        
        # Decrypt the data into our buffer
        aes_decryptor.decrypt_into(encrypted_data, decrypted_data)
        
        # Convert the decrypted binary data into readable numbers
        # This extracts all the values according to our data format
        (unit_id, person_id, sos_flag, latitude, longitude, 
         altitude_change, heading_degrees, step_count) = struct.unpack(DATA_FORMAT, decrypted_data)
        
        # Print the data in a format that the map programme can understand
        # This creates a line like: "Data: 1 2 0 51.507225 -0.127753 10.5 45.2 1234"
        print("Data: {} {} {} {:.6f} {:.6f} {:.1f} {:.1f} {}".format(
            unit_id,              # Which tracking device sent this
            person_id,            # Which person is carrying the device
            sos_flag,             # Emergency flag (0 = normal, 1 = SOS)
            latitude,             # GPS latitude (6 decimal places)
            longitude,            # GPS longitude (6 decimal places)
            altitude_change,      # Change in altitude (1 decimal place)
            heading_degrees,      # Direction of travel in degrees (1 decimal place)
            step_count            # Number of steps taken
        ))
        
    except Exception as decryption_error:
        # Something went wrong during decryption
        # This usually means the packet was corrupted or not from our devices
        # We'll just ignore it and keep listening
        continue
    
    # The loop continues forever, constantly listening for new packets
