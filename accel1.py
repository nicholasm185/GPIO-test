from mpu6050 import mpu6050
import time
from Kalman import KalmanAngle
import math

sensor = mpu6050(0x68)

while True:
	print("accelerometer:",sensor.get_accel_data())
	print("gyroscope:", sensor.get_gyro_data())
	print("temp:", sensor.get_temp())
	time.sleep(1)