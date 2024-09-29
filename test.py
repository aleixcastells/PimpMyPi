import board
import busio
import adafruit_ads1x15.ads1115 as ADS

i2c = busio.I2C(board.SCL, board.SDA)
ads = ADS.ADS1115(i2c)  # Include address if different

print("ADS1115 initialized successfully.")
