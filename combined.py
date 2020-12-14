"""
Math and KalmanAngle class thanks to rocheparadox on github
modified to work with mpu6050 module
"""
from mpu6050 import mpu6050
import time
from Kalman import KalmanAngle
import math
#from magnet import getMagnetStrength
import RPi.GPIO as GPIO

sensor = mpu6050(0x68)

TRESHOLD = 4000
mode = 0

def startRecord(channel):
    f = open('result.csv', 'w')
    rot = 0
    
    kalmanX = KalmanAngle()
    kalmanY = KalmanAngle()
    radToDeg = 57.2957786
    kalAngleX = 0
    kalAngleY = 0

    time.sleep(1)
    #Read Accelerometer raw value
    accdata = sensor.get_accel_data()
    accX = accdata["x"]
    accY = accdata["y"]
    accZ = accdata["z"]
    
    roll = math.atan2(accY,accZ) * radToDeg
    pitch = math.atan(-accX/math.sqrt((accY**2)+(accZ**2))) * radToDeg
    
    print(roll)
    kalmanX.setAngle(roll)
    kalmanY.setAngle(pitch)
    gyroXAngle = roll
    gyroYAngle = pitch
    compAngleX = roll
    compAngleY = pitch

    timer = time.time()
    i = 0

    while i < 200:
        #Read Accelerometer raw value
        accdata = sensor.get_accel_data()
        accX = accdata["x"]
        accY = accdata["y"]
        accZ = accdata["z"]

        #Read Gyroscope raw value
        gyrdata = sensor.get_gyro_data()
        gyroX = gyrdata["x"]
        gyroY = gyrdata["y"]
        gyroZ = gyrdata["z"]

        dt = time.time() - timer
        timer = time.time()
        
        roll = math.atan2(accY,accZ) * radToDeg
        pitch = math.atan(-accX/math.sqrt((accY**2)+(accZ**2))) * radToDeg

        gyroXRate = gyroX/131
        gyroYRate = gyroY/131
                
        if((roll < -90 and kalAngleX >90) or (roll > 90 and kalAngleX < -90)):
                kalmanX.setAngle(roll)
                complAngleX = roll
                kalAngleX   = roll
                gyroXAngle  = roll
        else:
            kalAngleX = kalmanX.getAngle(roll,gyroXRate,dt)

        gyroXAngle = gyroXRate * dt
        compAngleX = 0.93 * (compAngleX + gyroXRate * dt) + 0.07 * roll

        if ((gyroXAngle < -180) or (gyroXAngle > 180)):
            gyroXAngle = kalAngleX
        '''
        #datalist.append((kalAngleX,getMagnetStrength()))
        if(mode == 0):
            if(getMagnetStrength()> TRESHOLD):
                rot +=1
                mode = 1
        elif:
            if(getMagnetStrength()< TRESHOLD):
                mode = 0
        '''
                
        #.write(str(kalAngleX)+','+str(rot)+'\n')
        f.write(str(kalAngleX)+'\n')
        #print(kalAngleX)
        i += 1
    print('done recording')
    f.close()

GPIO.setwarnings(False) # Ignore warning for now
GPIO.setmode(GPIO.BOARD) # Use physical pin numbering
GPIO.setup(12, GPIO.IN, pull_up_down=GPIO.PUD_DOWN) # Set pin 18 to be an input pin and set initial value to be pulled low (off)
GPIO.add_event_detect(12,GPIO.RISING,callback=startRecord) # Setup event on pin 10 rising edge
message = input("Press enter to quit\n\n") # Run until someone presses enter
GPIO.cleanup() # Clean u