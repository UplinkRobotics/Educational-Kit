// Uplinkrobotics Educational Kit I2C COR as slave device example
// Program to demonstrate recieving I2C Signals on the COR from an Arduino Uno acting as a master device
// 
// Copyright © 2024 UplinkRobotics LLC

#include <Wire.h> // include the library for I2C

int rec_val = 0;    // value to recieve

void setup() {
// Start the I2C Bus as Slave on address 9
  Wire.begin(9); 

  // Attach a function to trigger when something is received.
  Wire.onReceive(receiveEvent);
}

// Function that is called when the I2C line recieves a message
void receiveEvent(int bytes) {
  rec_val = Wire.read();    // read one character from the I2C
  Serial.println(rec_val);  // print the character to the serial monitor
}

void loop() {
  // do nothing but wait for i2c messages

}
