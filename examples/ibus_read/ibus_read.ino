// Uplinkrobotics Educational Kit iBUS receiver read (FlySky Receiver)
// Program to demonstrate reading iBUS signals from an RC receiver on the COR Microcontroller
// 
// Uses the IBusBM library
// Copyright © 2024 UplinkRobotics LLC

#include <IBusBM.h> // import library
//#include <ESP32Servo.h>

IBusBM IBus; // create IBUS object

// variables to hold the channel inputs
int ch1, ch2, ch3, ch4, ch5, ch6, ch7, ch8, ch9, ch10;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);     // start the UART connection to send values to the serial monitor

  IBus.begin(Serial2,1);    // iBUS object connected to serial2 RX2 pin and use timer 1

}

void loop() {
  // put your main code here, to run repeatedly:

  // read the values from the iBUS RC receiver
  ch1 = IBus.readChannel(0);    // get latest value from channel 1
  ch2 = IBus.readChannel(1);    // get latest value from channel 2
  ch3 = IBus.readChannel(2);  // get latest value from channel 3
  ch4 = IBus.readChannel(3);  // get latest value from channel 4
  ch5 = IBus.readChannel(4);   // get latest value from channel 5
  ch6 = IBus.readChannel(5);    // get latest value from channel 6
  // only used if receiver supports more than 6 channels
  // ch7 = IBus.readChannel(6);
  // ch8 = IBus.readChannel(7);
  // ch9 = IBus.readChannel(8);
  // ch10 = IBus.readChannel(9);
 

  // print all the values to the serial monitor to ensure that they all work
  Serial.print("CH1:  ");
  Serial.print(ch1);
  Serial.print("   CH2:  ");
  Serial.print(ch2);
  Serial.print("   CH3:  ");
  Serial.print(ch3);
  Serial.print("   CH4:  ");
  Serial.print(ch4);
  Serial.print("   CH5:  ");
  Serial.print(ch5);
  Serial.print("   CH6:  ");
  Serial.print(ch6);
  Serial.print("   CH7:  ");
  Serial.print(ch7);
  Serial.print("   CH8:  ");
  Serial.print(ch8);
  Serial.print("   CH9:  ");
  Serial.print(ch9);
  Serial.print("   CH10:  ");
  Serial.println(ch10);

  delay(10);

}
