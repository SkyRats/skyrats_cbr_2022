import RPi.GPIO as GPIO
import time

class Periferico():
    def __init__(self, pin) -> None:
        self.pin = pin
        
    def ligar(self, delayTime):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pin, GPIO.OUT, initial=GPIO.LOW)
        GPIO.outout(self.pin, GPIO.HIGH)
        time.delay(delayTime)
        GPIO.output(self.pin, GPIO.LOW)


class Buzzer(Periferico):
    def __init__(self, pin):
        super().__init__(pin)

class Led(Periferico):
    def __init__(self, pin):
        super().__init__(pin)