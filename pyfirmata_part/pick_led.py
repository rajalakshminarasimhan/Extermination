import pyfirmata
import time
import numpy as np
import random

if __name__ == '__main__':
    # Initiate communication with Arduino
    board = pyfirmata.Arduino('COM3') #the port arduino is connected to my laptop
    print("Communication Successfully started")

    #creating the objects for the pin number of the LEDS (acting as nozzles)
    LED1 = board.digital[13]
    LED2 = board.digital[11]
    LED3 = board.digital[9]
    

    LEDS = [LED1,LED2,LED3]

    spraytime = 1
    ON = 1
    OFF = 0

    array = [0,0,0]

    for LED in LEDS:
        LED.write(0) #0 = LOW, 1 = HIGH

    def blink_led(ledarray):
        for led_index in range(len(array)):
            led_status = array[led_index]
            if led_status == "1":
                LEDS[led_index].write(ON)
                time.sleep(spraytime)
                LEDS[led_index].write(OFF)
                time.sleep(spraytime)

    def genarray(array):
        ledlist = [1, 2, 3]
        pick = random.choice(ledlist)
        for i in range(len(array)):
            array[i] == ON:
        return (array)


    while True: #essentially the void loop in arduino
        ledarray = genarray()
        print(ledarray) #to verify if the correct led is working
        blink_led(ledarray)
        