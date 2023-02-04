/* This code is meant to test the Agrobot array of valves and their communication with the arduino
 * - This is based on the Arduio-Jetson communicasion and therefore uses a string of 0s to represent the valves
 * - counting of the vlaves starts with 1 (whenever a valve number is passed into a function it should follow this convention)
 * - See comments for details and further questions to be answered.  
 * Author: Gus Hawley (anguswymanhawley@gmail.com)
 */

// Pin information
int valve_pins[] = {3, 4, 5, 6, 7, 8, 9, 10, 11}; // These values should be replaced with the actual pins
int number_of_valves = 9;


void setup() {
  // set all valve pins to output mode
  for(int i = 0; i < number_of_valves; i++) {
    pinMode(valve_pins[i], OUTPUT);
  }
  Serial.begin(9600);

}

// Loops through characters in a passed string and actuates the specified valves
void actuateValves(String valve_string){
  for(int i = 0; i < valve_string.length(); i++) {
    if(valve_string.charAt(i) == '1') {
      digitalWrite(valve_pins[i], HIGH);
    }
    else {
      digitalWrite(valve_pins[i], LOW);
    }
  }
}

//Turns each valve on and off with specified delay between each action
void onOffTest(int delayTime = 1000){
  Serial.println("On/Off test has started");
  String valve_string = "000000000";
  for(int i = 0; i < valve_string.length(); i++){
    Serial.print("Now testing valve ");
    Serial.println(i+1);
    valve_string[i] = '1';
    actuateValves(valve_string);
    delay(delayTime);
    valve_string[i] = '0';
    actuateValves(valve_string);
    delay(delayTime);
  }
}

//Pulses specified valve on and off a specified number of times and then reduces the pulse delay by a specified percentage
void pulseSpeedTest(int valveNo, int startDelay, int minDelay, double percentReduction, int numberOfPulses = 4){
  Serial.print("Pulse speed test has started using valve ");
  Serial.println(valveNo);
  String valve_string = "000000000";
  double delayTime = startDelay;

  //reduces delayTime by specified percentage until it reaches the minimum specified delay.
  while(delayTime >= minDelay){
    Serial.print("Testing delay: ");
    Serial.println(delayTime);
    for(int j = 0; j < numberOfPulses; j++){
      valve_string[valveNo-1] = '1';
      actuateValves(valve_string);
      delay(delayTime);
      valve_string[valveNo-1] = '0';
      actuateValves(valve_string);
      delay(delayTime);
    }
    delayTime = delayTime * (1 - percentReduction/100);
    delay(3000);
  }
}

//Turns increasing number of valves on then off.
void maxValvesTest(int sprayTime = 1000, int delayTime = 3000){
  Serial.println("Max valves test has started");
  String valve_string = "000000000";
  for(int i = 0; i < valve_string.length(); i++){
    Serial.print("Now testing ");
    Serial.print(i+1);
    Serial.println(" vlave(s)");
    for(int j = 0; j <= i; j++){
      valve_string[j] = '1';
    }
    actuateValves(valve_string);
    delay(delayTime);
    valve_string = "000000000";
    actuateValves(valve_string);
    delay(delayTime);
  }
  
}

void loop() {
  onOffTest();
  //pulseSpeedTest(1, 4000, 100, 25, 5);
  //maxValvesTest(1000, 5000);
  Serial.println("Tests completed");
  delay(5000);
}

  
