// Uplinkrobotics Educational Kit ELRS receiver read
// Program to demonstrate reading ELRS signals from an RC receiver on the COR Microcontroller
// 
// Uses the esp32-elrs-crsf library
// Copyright © 2024 UplinkRobotics LLC

#include "crsf.h" // include the ELRS library

#define RXD2 16   // pins used on th eCOR to connect to the RC Receiver
#define TXD2 17

// required setup for the ELRS library
#define SBUS_BUFFER_SIZE 25
uint8_t _rcs_buf[25] {};
uint16_t _raw_rc_values[RC_INPUT_MAX_CHANNELS] {};
uint16_t _raw_rc_count{};

// variables to hold the channel inputs
int ch1, ch2, ch3, ch4, ch5, ch6, ch7, ch8, ch9, ch10, ch11, ch12, ch13, ch14, ch15, ch16;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);     // start the UART connection to send values to the serial monitor

  Serial1.begin(420000, SERIAL_8N1, RXD2, TXD2); // connect to the ELRS receiver
}

void loop() {
  // put your main code here, to run repeatedly:

  // read values from the RC reciever ELRS signal
  size_t numBytesRead = Serial1.readBytes(_rcs_buf, SBUS_BUFFER_SIZE);
  if(numBytesRead > 0){
    crsf_parse(&_rcs_buf[0], SBUS_BUFFER_SIZE, &_raw_rc_values[0], &_raw_rc_count, RC_INPUT_MAX_CHANNELS );
    ch1 = _raw_rc_values[0];
    ch2 = _raw_rc_values[1];
    ch3 = _raw_rc_values[2];
    ch4 = _raw_rc_values[3];
    ch5 = _raw_rc_values[4];
    ch6 = _raw_rc_values[5];
    ch7 = _raw_rc_values[6];
    ch8 = _raw_rc_values[7];
    ch9 = _raw_rc_values[8];
    ch10 = _raw_rc_values[9];
    ch11 = _raw_rc_values[10];
    ch12 = _raw_rc_values[11];
    ch13 = _raw_rc_values[12];
    ch14 = _raw_rc_values[13];
    ch15 = _raw_rc_values[14];
    ch16 = _raw_rc_values[15];
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
