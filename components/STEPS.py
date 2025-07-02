import time, board, busio
from adafruit_bno08x.i2c import BNO08X_I2C
import adafruit_bno08x


#BNO08X init
i2c = busio.I2C(board.SCL, board.SDA, frequency=800_000)
bno = BNO08X_I2C(i2c)
bno.enable_feature(adafruit_bno08x.BNO_REPORT_STEP_COUNTER)


#step count loop
#save last steps #
last_step = bno.steps

while True:
    #using bno.steps to get steps int
    steps = bno.steps
    #if steps is not not equal to last steps
    if steps != last_step:
        print(f"Steps: {steps}")
        #set last step variable to current step #
        last_step = steps

