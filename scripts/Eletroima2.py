import RPi.GPIO as GPIO #usar wpi?

PIN = 1 #tem que decidir quais os PINs iniciais pro buzzer, led e eletroima

#se consumir muita corrente:
class Eletroima:
    def __init__(self) -> None:
        self.pin = PIN
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.pin, GPIO.OUT, initial=GPIO.LOW)
   
    def pegar(self):
        GPIO.output(self.pin, GPIO.HIGH)
    
    def soltar(self):
        GPIO.output(self.pin, GPIO.LOW)
    
    def setPin(self, pin):
        self.pin = pin
    
    def getPin(self):
        return self.pin