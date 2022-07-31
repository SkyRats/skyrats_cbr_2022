import RPi.GPIO as GPIO #usar wpi?
import time

PIN = 1 #tem que decidir quais os PINs iniciais pro buzzer, led e eletroima

#se consumir muito pouca corrente:
class Eletroima:
    def __init__(self) -> None:
        self.pin = PIN
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.pin, GPIO.OUT, initial=GPIO.HIGH)
   
    def soltar(self):
        GPIO.output(self.pin, GPIO.LOW)
        time.sleep(5) #desliga por 5 segundos, testar se é o suficiente
        GPIO.output(self.pin, GPIO.HIGH)
    
    def setPin(self, pin):
        self.pin = pin
    
    def getPin(self):
        return self.pin