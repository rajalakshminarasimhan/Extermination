import RPi.GPIO as GPIO
from time import sleep
import re

# global variable
pins = [8, 7, 1, 0, 5, 6, 16, 20, 21]

def setup():
	GPIO.setwarnings(False)
	GPIO.setmode(GPIO.BCM)


	# set pins as outputs

	for pin in pins:
		GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)

def blink_all_lights():
	while True:
		# turn on all LED's
		for pin in pins:
			GPIO.output(pin, GPIO.HIGH)
		sleep(1)

		# turn off all LED's
		for pin in pins:
			GPIO.output(pin, GPIO.LOW)
		sleep(1)

def led_on(led_idx):
	if type(led_idx) == str:
		led_idx = int(re.search('[0-9]', led_idx).group(0))
	GPIO.output(pins[led_idx], GPIO.HIGH)

def led_off(led_idx):
	if type(led_idx) == str:
		led_idx = int(re.search('[0-9]', led_idx).group(0))
	GPIO.output(pins[led_idx], GPIO.LOW)

def all_led_off():
	for pin in pins:
		GPIO.output(pin, GPIO.LOW)

def main():
	try:
		while True:
			for pin in range(len(pins)):
				led_on(pin)
				sleep(1)
#				led_off(pin)
				all_led_off()
	except KeyboardInterrupt:
		# exit cleanly
		print("Keyboard Interrupt")

	except: # catch all other exceptions
		print("Some other exception")

	finally:
		all_led_off()
		print("Cleaned up")
#		GPIO.cleanup() # always cleanup 


    
if __name__ == "__main__":
	
	setup()
	# main() # for testing
	
