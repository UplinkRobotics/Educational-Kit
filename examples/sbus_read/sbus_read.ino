// Uplinkrobotics Educational Kit SBUS receiver read
// Program to demonstrate reading SBUS signals from an RC receiver on the COR Microcontroller
// 
// Uses the TheDIYGuy999_SBUS library
// Copyright © 2024 UplinkRobotics LLC

#include "SBUS.h" // include the SBUS library

#define RXD2 16   // pins used on th eCOR to connect to the RC Receiver
#define TXD2 17

SBUS rxsr(Serial2); // create the SBUS object
uint16_t channels[16];  // channels variable
bool failSafe;    // goes high if the failsafe is triggered
bool lostFrame;  

// variables to hold the channel inputs
int ch1, ch2, ch3, ch4, ch5, ch6, ch7, ch8, ch9, ch10, ch11, ch12, ch13, ch14, ch15, ch16;

void setup() {
  // put your setup code here, to run once:
  rxsr.begin(RXD2, TXD2, true, 100000); // optional parameters for ESP32: RX pin, TX pin, inverse mode, baudrate

}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.begin(115200);     // start the UART connection to send values to the serial monitor

  // read the values from the RC reciever SBUS signal
  if(rxsr.read(&channels[0], &failSafe, &lostFrame)){
    ch1 = channels[0];
    ch2 = channels[1];
    ch3 = channels[2];
    ch4 = channels[3];
    ch5 = channels[4];
    ch6 = channels[5];
    ch7 = channels[6];
    ch8 = channels[7];
    ch9 = channels[8];
    ch10 = channels[9];
    ch11 = channels[10];
    ch12 = channels[11];
    ch13 = channels[12];
    ch14 = channels[13];
    ch15 = channels[14];
    ch16 = channels[15];
  }

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
  Serial.print(ch10);
  Serial.print("   CH11:  ");
  Serial.print(ch11);
  Serial.print("   CH12:  ");
  Serial.print(ch12);
  Serial.print("   CH13:  ");
  Serial.print(ch13);
  Serial.print("   CH14:  ");
  Serial.print(ch14);
  Serial.print("   CH15:  ");
  Serial.print(ch15);
  Serial.print("   CH16:  ");
  Serial.println(ch16);


}
