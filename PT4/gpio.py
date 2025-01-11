import Jetson.GPIO as GPIO
import time

output_pins = [18, 19, 21, 22, 23, 24, 26, 31, 11, 7]

C1 = [26,31]
C2 = [18, 19]
C3 = [7,11,23,24,21,22]
C4 = [26, 31,23,24,18,19]
C5 = [7,11]

GPIO.setmode(GPIO.BOARD)
for pin in output_pins:
    GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)

try:
    for i in range(1):
        for pin in output_pins:
            GPIO.output(pin, GPIO.LOW)
        GPIO.output(C1, GPIO.HIGH)
        time.sleep(5)

        GPIO.output(C1, GPIO.LOW)
        GPIO.output(C2, GPIO.HIGH)
        time.sleep(5)

        GPIO.output(C2, GPIO.LOW)
        GPIO.output(C3, GPIO.HIGH)
        time.sleep(5)

        GPIO.output(C3, GPIO.LOW)	
        GPIO.output(C4, GPIO.HIGH)
        time.sleep(5)

        GPIO.output(C4, GPIO.LOW)
        GPIO.output(C5, GPIO.HIGH)
        time.sleep(5)

        GPIO.output(C5, GPIO.LOW)


except KeyboardInterrupt:
    GPIO.cleanup()
