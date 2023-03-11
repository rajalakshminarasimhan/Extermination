''' This code is meant to test the valves of the Agrobot to ensure that the correct valve is turned on based on the array/string received from Imagerec
    This utilizes the python package "PYFIRMATA()" to communicate with the arduino and uses values of 0 (OFF) or 1 (ON) to represent the valves
    
    Author: Rajalakshmi Narasimhan (rajalakshmi.nr6@gmail.com)
'''

#importing the required packages

import pyfirmata
import numpy as np
import time # used to write the delay() statement in arduino
import random # not used but just in case we need it to test random valves

#Similar to the main body for arduino 
if __name__ == '__main__':

    '''INFO FOR SYNTAX 
        It treates the Arduino board as an object called BOARD and has functions that can be used with it.
        INTILIAZING PINS: object_at_pin = board.digital[PIN_NUMBER]
        WRITING TO A PIN: object_at_pin.write(ON/OFF)
    '''

    # Initiating communication with Arduino
    board = pyfirmata.Arduino('COM3') #the port arduino is connected to the laptop
    print("Communication Successfully started") 

    #defining constant values
    spraytime = 10 #the time for which the sprayer should spray
    delaytime = 1
    ON = 1
    OFF = 0
    no_of_valves = 8

    #defining some lists/arrays that we use
    pin_nos = [4, 5, 6, 7, 8, 9, 10, 11] # pin 13 had an issue in our arduino; replace with appropisate PIN nos
    valve_array = []

    #setting up pin info for all the valves
    for i in range(no_of_valves):
        valve_name = 'VALVE'+str(i+1)
        valve_name = board.digital[pin_nos[i]]
        valve_array.append(valve_name)

    ''' FUNCTION: Reset()
        PARAMETERS: none
        OBJECTIVE: to set the intial state - state where all the valves are off
        RETURNS: none 
    '''
    def reset():
        for valve in valve_array:
            valve.write(OFF)
    ''' FUNCTION: all_valves()
        PARAMETERS: spraytime - Time for which it sprays
        OBJECTIVE: To spray all the valves in sequence, without receiving an array
        RETURNS: none 
    '''
    def all_valves(spraytime):
        print("The sequential valve spray test has started")
        for index in range(10):
            valve_array[index].write(ON)
            time.sleep(spraytime)       # IN ARDUINO: delay(time)
            valve_array[index].write(OFF)
            time.sleep(delaytime)

    ''' FUNCTION: turn_on_valve()
        PARAMETERS: array - It is an array of 0/1 which indicates which valve should be turned on
                    spraytime - Time for which it sprays
                    delaytime - Time for which it turns off before spraying again
        OBJECTIVE: To spray the correct valve for a fixed time
        RETURNS: none 
    '''
    def turn_on_valve(array, spraytime,delaytime):
        print("The valve test has started")
        for index in range(len(array)):
            valve_status = array[index]
            if valve_status == 1:
                print(valve_status)         #just to verify it is at the correct value 
                valve_array[index].write(ON)
                time.sleep(spraytime)       # IN ARDUINO: delay(time)
                valve_array[index].write(OFF)
                time.sleep(delaytime)

    ''' Adapted from Gus' code
        FUNCTION: pulse_valve():
        PARAMETERS: array - It is an array of 0/1 which indicates which valve should be turned on
                    intial_spraytime - The start time for which the valve sprays
                    min_spraytime - The minimum spray time
                    percentReduction - The percentage by which thespray time decreases every iteration
                    no_of_pulses - no of pulses for each spray time 
        OBJECTIVE: Pulses specified valve on and off for a given number of times and then reduces the spray time by given percentage
    '''
    def pulse_valve(array, intial_spraytime=3, min_spraytime=1, percentReduction=25, no_of_pulses=3): #with arbitary default values
        print("Pulse valves test has started")
        spraytime = intial_spraytime
        while spraytime >= min_spraytime:
            for j in range(no_of_pulses):
                turn_on_valve(array,spraytime)
        spraytime -= spraytime*(percentReduction/100)

    ''' Adapted from Gus' code
        FUNCTION: maxvalvestest():
        PARAMETERS: spraytime - The start time for which the valve sprays
        OBJECTIVE: Turns increasing number of valves on and then off
    '''
    def maxvalvestest(valve_array, spraytime, delaytime):
        print("Max valves test has started")
        range_of_valve = []
        for v in range(len(valve_array)): 
            range_of_valve.append(v)
            turn_on_valve(range_of_valve)


    #This is essentially the loop in arduino, where the functions are execute.

    while True: 
        reset()
        arrayReceived = [0,1,0,0,0,0,0,0]   #the array of 0/1 we receive to know which valve to turn on
        all_valves(spraytime)

        #turn_on_valve(arrayReceived,spraytime,delaytime)    # using default values for spraytime, delaytime

        # pulse_valve(arrayReceived)
        # maxvalvestest(valve_array)

        reset()