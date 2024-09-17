// Uplinkrobotics Educational Kit Servo Control Example
// Program to demonstrate the use of RC-PWM signals to run the two servo control outputs
// Runs the top servo port and then the bottom servo in a sweep
//
// Copyright © 2024 UplinkRobotics LLC

#include <ESP32Servo.h>

Servo servo1;  // create servo object to control servo 1
Servo servo2;  // create servo object to control servo 2

const int servo1_io = 18; // top servo channel
const int servo2_io = 5;  // bottom servo channel

const int max_val = 180;  // max and minumum value sent to the servos 
const int min_val = 0;    
// if servos are erratic at the extreme values (0 or 180), lower the max_val and raise the min_val slightly

void setup() {
  // put your setup code here, to run once:
  servo1.attach(servo1_io); // attaches servo1 pin
  servo2.attach(servo2_io); // attaches servo2 pin
}

void loop() {
  // put your main code here, to run repeatedly:

  // sweep one direction with the top servo
  for (int servo1_pos = min_val; servo1_pos < max_val; servo1_pos++){
    servo1.write(servo1_pos);
    delay(10);
  }
  // sweep the other direction with the top servo
  for (int servo1_pos = max_val; servo1_pos > min_val; servo1_pos--){
    servo1.write(servo1_pos);
    delay(10);
  }
  // sweep one direction with the bottom servo
  for (int servo2_pos = min_val; servo2_pos < max_val; servo2_pos++){
    servo2.write(servo2_pos);
    delay(10);
  }
  // sweep the other direction with the bottom servo
  for (int servo2_pos = max_val; servo2_pos > min_val; servo2_pos--){
    servo2.write(servo2_pos);
    delay(10);
  }

}
