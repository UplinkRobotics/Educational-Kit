// Uplinkrobotics Educational Kit I2C Arduino Uno as master device example
// Program to demonstrate sending I2C Signals from an Arduino Uno to the COR to act as a master device
// 
// Copyright © 2024 UplinkRobotics LLC

#include <Wire.h> // include the library for I2C

// THESE ARE EXTREMELY IMPORTANT AND MUST DEFINED
// Since the Arduino UNO operates at 5V, we must disable the internal pullup resistors so that the 
// ESP32 on the COR does not get damaged. The COR will use its internal pullup resistors to pull
// the voltage to 3.3V. This will not interfere with the Arduinos digital input high 
#define sda_io 18
#define scl_io 19

int send_val = 0;   // value to send

// the setup function runs once when you press reset or power the board
void setup() {
  // THESE LINES DISABLE THE INTERNAL PULLUP RESISTORS IN THE ARDUINO
  // IF THESE ARE REMOVED, THE ESP32 ON THE COR WILL BE DAMAGED OVER TIME
  digitalWrite(sda_io, LOW);
  digitalWrite(scl_io, LOW);

  Serial.begin(115200); // start the serial monitor for "print statements"

  Wire.begin(); // intialize the I2C connection on the Arduino
}
  

// the loop function runs over and over again forever
void loop() {
  Wire.beginTransmission(9);  // Transmit to device at address 0x09
  Wire.write(send_val);       // Send the value
  Wire.endTransmission();     // Stop transmitting

  send_val++;                 // Increment the value

  if (send_val > 20){
    send_val = 0;             // reset x once it gets to 20
  } 

  Serial.println(send_val);   // print the value to the serial monitor

  delay(500); // wait half a second
} 
