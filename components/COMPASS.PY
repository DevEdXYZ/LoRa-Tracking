import time, math, board, busio
from adafruit_bno08x.i2c import BNO08X_I2C
import adafruit_bno08x


#BNO08X init
i2c = busio.I2C(board.SCL, board.SDA, frequency=800_000)
bno = BNO08X_I2C(i2c)
bno.enable_feature(adafruit_bno08x.BNO_REPORT_MAGNETOMETER)

    
#heading loop
while True:
    #mag x and y throw away z as it is for incline
    mx, my, _ = bno.magnetic
    
    #atan2 returns the angle of that vector in radians # math.degress returns it in degrees
    heading = math.degrees(math.atan2(my, mx))
    
    #atan2 can return negative angles for vectors in the lower half of the circle so this ensures it is always between 0 and 360
    if heading < 0:
        heading += 360.0
    print(f"Heading: {heading:.1f}°")
    time.sleep(0.1)

