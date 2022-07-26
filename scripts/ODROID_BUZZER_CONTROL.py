import odroid_wiringpi as wpi

PIN = 26

class Buzzer:
	def __init__(self):
		self.pin = PIN
		wpi.wiringPiSetup()
		wpi.pinMode(self.pin, 1)

	def on(self, time):
		wpi.digitalWrite(self.pin, 1)
		print("Ligado")
		wpi.delay(time*1000)
		wpi.digitalWrite(self.pin, 0)
		print("Desligado")

buzzer = Buzzer()
time = int(input("tempo: "))
buzzer.on(time)

