import time, board, busio
import adafruit_gps


#GPS init
try:
    #init UART on default TX(GPIO 0) / RX (GPIO 1)
    uart = busio.UART(board.TX, board.RX, baudrate=9600, timeout=100)
    
    #init gps object with uart
    gps = adafruit_gps.GPS(uart, debug=False)
    
    #configure which NMEA sentences are output (RMC and GGA only)
    gps.send_command(b"PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0")
    
    #set update rate to 1Hz (once per second)
    gps.send_command(b"PMTK220,1000")
    
#handle failure
except Exception as e:
    print("GPS init failed:", e)
    gps = None


#gps loop
while True:
    if gps is not None:
        #update internal GPS data (non-blocking)
        gps.update()
        
        #check for fix (i.e., valid location data)
        if gps.has_fix:
            #print latitude and longitude with 6 decimal places
            print(f"Lat: {gps.latitude:.6f}  Lon: {gps.longitude:.6f}")
            
    #repeat every n seconds
    time.sleep(10)

