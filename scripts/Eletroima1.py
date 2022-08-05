import odroid_wiringpi as wpi

PIN = 1 #tem que decidir quais os PINs iniciais pro buzzer, led e eletroima

#se consumir muito pouca corrente:
class Eletroima:
    def __init__(self):
        self.pin = PIN
        wpi.wiringPiSetup()
		wpi.pinMode(self.pin, 1)
        wpi.digitalWrite(self.pin, 1)
   
    def soltar(self):
        wpi.digitalWrite(self.pin, 0)
        wpi.delay(5000) #desliga por 5 segundos, testar se é o suficiente
        wpi.digitalWrite(self.pin, 1)
    
    def setPin(self, pin):
        self.pin = pin
    
    def getPin(self):
        return self.pin