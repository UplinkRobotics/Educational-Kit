// Uplinkrobotics Educational Kit LED Control Example
// Program to demonstrate the use of PWM signals to control the brightness of the two LED driver outputs
// Runs the 12v LED driver in a brightness sweep and then the 3.3V current limited LED driver in a sweep
//
// Copyright © 2024 UplinkRobotics LLC

const int led1_io = 4;   // led channel 1 3.3V - Lower right, tied to reset button
const int led2_io = 0;   // led channel 2 12V

void setup() {
  // put your setup code here, to run once:
  // put the pins into output mode
  pinMode(led1_io, OUTPUT);
  pinMode(led2_io, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  
  // 12V LED driver - need external current limiting resistor (bottom screw terminal)
  // sweep through PWM duty cycle of 0% to 100% increasing the brightness of the LED
  for (int j = 0; j < 255; j++){
    analogWrite(led2_io, j);
    delay(10);
  }

  // 3.3V LED driver - internal current limiting resistor, 1ohm 2W - (bottom screw terminal)
  // sweep through PWM duty cycle of 0% to 100% increasing the brightness of the LED
  for (int i = 0; i < 255; i++){
    analogWrite(led1_io, i);
    delay(10);
  }

}
