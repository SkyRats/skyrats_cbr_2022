import RPi.GPIO as GPIO
import time

class Periferico():
    def __init__(self, pin) -> None:
        self.pin = pin
        #GPIO.setmode(GPIO.BCM)
        #GPIO.setup(self.pin,GPIO.OUT, initial=GPIO.LOW)

    def ligar(self, delayTime):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pin, GPIO.OUT, initial=GPIO.LOW)
        GPIO.output(self.pin, GPIO.HIGH)
        time.sleep(delayTime)
        GPIO.output(self.pin, GPIO.LOW)


class Buzzer(Periferico):
    def __init__(self, pin):
        super().__init__(pin)

class Led(Periferico):
    def __init__(self, pin):
        super().__init__(pin)
if __name__ == '__main__':
    #print('Ligando led verde...')
    led_verde = Led(17)
    led_vermelho = Led(27)
    buz = Buzzer(22)
    #led_verde.ligar(1)
    #led_vermelho.ligar(1)
    led_verde.ligar(0.2)
    led_vermelho.ligar(0.2)
    buz.ligar(0.2)
