/* Note: this code is based on this tutorial: https://maker.pro/arduino/tutorial/how-to-use-arduino-with-robot-operating-system-ros
 * - As a subscriber the code should recieve a string in the format "000000000" 
 *   where each 0 corresponds to a valve. When a valve should be turned on, the
 *   0 it corresponds to will change to 1 ("000100000" for valve 4).
 * - This code has not been tested but it does compile see comments for details 
 *   and further questions to be answered.  
 * Author: Gus Hawley (anguswymanhawley@gmail.com)
 */

#include <ros.h>
#include <std_msgs/String.h>

// Pin information
int valve_pins[] = {3, 4, 5, 6, 7, 8, 9, 10, 11, 12}; // These values should be replaced with the actual pins
int number_of_valves = 9;

ros::NodeHandle node_handle;

std_msgs::String valve_data;

void subscriberCallback(const std_msgs::String& valve_data) {
  // std_msgs::String does not have methods for length and character indexing so the information is coppied to a String
  String valve_string = valve_data.data;

  // loop through each character in the recieved string and actuate the valves as specified
  for(int i = 0; i < valve_string.length(); i++) {
    if(valve_string.charAt(i) == '1') {
      digitalWrite(valve_pins[i], HIGH);
    }
    else {
      digitalWrite(valve_pins[i], LOW);
    }
  }
}

// I am unsure whether "/sprayers_actuate" should include the / or not. This should be tested
ros::Subscriber<std_msgs::String> valve_subscriber("/sprayers_actuate", &subscriberCallback);

void setup() {
  // set all valve pins to output mode
  for(int i = 0; i < number_of_valves; i++) {
    pinMode(valve_pins[i], OUTPUT);
  }

  node_handle.initNode();
  node_handle.subscribe(valve_subscriber);

}

void loop() {
  // according to the library "This function goes in your loop() function, it handles serial input and callbacks for subscribers."
  node_handle.spinOnce();
  delay(100);
}
