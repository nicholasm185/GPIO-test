from gpiozero import LED
from gpiozero import PWMLED
from time import sleep

led1 = PWMLED(16)

"""
while True:
    led1.on()
    sleep(1)
    led1.off()
    sleep(1)
"""

for i in range(0,11,1):
    led1.value = (i/10)
    sleep(0.1)
    
for j in range(10,-1,-1):
    led1.value = (j/10)
    sleep(0.5)