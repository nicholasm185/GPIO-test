from mpu6050 import mpu6050
import time
from Kalman import KalmanAngle
import math
from magnet import getMagnetStrength
import RPi.GPIO as GPIO
from threading import Thread
import requests

SERVER_URL = 'http://10.12.0.158:8080/sendResult'

class ReCycle:
    def __init__(self, serverURL):
        self.sensor = mpu6050(0x68)
        self.TRESHOLD = 100
        self.mode = 0
        self.on = False
        self.serverURL = serverURL
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(12, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(10,GPIO.OUT)
        GPIO.add_event_detect(12,GPIO.RISING,callback=self.turnOn)
        message = input("Press enter to quit\n\n")
        GPIO.cleanup()

    def RecordLoop(self):
        f = open('result.csv', 'w')
        rot = 0
        mode = 0
        
        kalmanX = KalmanAngle()
        radToDeg = 57.2957786
        kalAngleX = 0

        time.sleep(1)
        #Read Accelerometer raw value
        accdata = self.sensor.get_accel_data()
        accY = accdata["y"]
        accZ = accdata["z"]
        
        roll = math.atan2(accY,accZ) * radToDeg

        kalmanX.setAngle(roll)
        gyroXAngle = roll
        compAngleX = roll

        timer = time.time()
        f.write(str(timer)+'\n')
        print('recording commencing')
        while self.on:
            #Read Accelerometer raw value
            accdata = self.sensor.get_accel_data()
            accX = accdata["x"]
            accY = accdata["y"]
            accZ = accdata["z"]

            #Read Gyroscope raw value
            gyrdata = self.sensor.get_gyro_data()
            gyroX = gyrdata["x"]
            gyroY = gyrdata["y"]

            dt = time.time() - timer
            timer = time.time()
            
            roll = math.atan2(accY,accZ) * radToDeg

            gyroXRate = gyroX/131
                    
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

            mag = getMagnetStrength()
            if(mode == 0):
                if(mag > self.TRESHOLD):
                    rot +=1
                    mode = 1
            else:
                if(mag < self.TRESHOLD):
                    mode = 0
                    
            f.write(str(kalAngleX)+','+str(rot)+'\n')
            #f.write(str(kalAngleX)+'\n')
            #print(kalAngleX, getMagnetStrength())
        print('done recording')
        f.write(str(timer))
        f.close()
        
    def turnOn(self, channel):
        if not self.on:
            self.on = True
            print('Starting loop: ' + str(self.on))
            Thread(target = self.RecordLoop).start()
            GPIO.output(10,GPIO.HIGH)
        elif self.on:
            self.on = False
            print('loop stopping: ' + str(self.on))
            time.sleep(2)
            print('uploading')
            with open('result.csv', 'rb') as f:
                try:
                    r = requests.post(self.serverURL, files={'data':f})
                    print('upload finished')
                except:
                    print('upload failed')
            GPIO.output(10,GPIO.LOW)

x = ReCycle(SERVER_URL)
    
