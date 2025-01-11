import Jetson.GPIO as GPIO
import time

output_pins = [19,11,7,29,32,35,40]

C1 = [7,32]
C2 = [40,19]
C3 = [11,29,35]
C4 = [7,11,19]
C5 = [29,32,35,40]

GPIO.setmode(GPIO.BOARD)
for pin in output_pins:
    GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)

try:
    for i in range(2):
        for pin in output_pins:
            GPIO.output(pin, GPIO.LOW)
        GPIO.output(C1, GPIO.HIGH)
        time.sleep(4)

        GPIO.output(C1, GPIO.LOW)
        GPIO.output(C2, GPIO.HIGH)
        time.sleep(4)

        GPIO.output(C2, GPIO.LOW)
        GPIO.output(C3, GPIO.HIGH)
        time.sleep(4)

        GPIO.output(C3, GPIO.LOW)
        GPIO.output(C4, GPIO.HIGH)
        time.sleep(4)

        GPIO.output(C4, GPIO.LOW)
        GPIO.output(C5, GPIO.HIGH)
        time.sleep(4)

        GPIO.output(C5, GPIO.LOW)

except KeyboardInterrupt:
    for pin in output_pins:
    	GPIO.output(pin, GPIO.LOW)
    GPIO.cleanup()
