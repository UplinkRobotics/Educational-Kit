// Uplinkrobotics Educational Kit Basic Rover Example
// Program to demonstrate the use the educational kit to drive around, control LEDs, control servos,
// and control the LED array, 
//
// Copyright © 2024 UplinkRobotics LLC

// Include libraries
#include <IBusBM.h>
#include "SBUS.h"
#include "crsf.h"
#include <ESP32Servo.h>

// Configurations Options
/***************************************************************************************************/

// Uncomment the type of communication protocol connected to the COR
#define I_BUS     // Ibus communication used in the FlySky receivers
// #define S_BUS  // Sbus communication used in FrSky recievers
// #define ELRS   // ELRS communication used by ELRS receivers

// Enable Debug mode
#define DEBUG

// Frequency that the main loop tries to run at - it can be slower
#define MAIN_LOOP_FREQ 60.0 // hertz = cycles/second
long int loop_interval_us = (1.0/MAIN_LOOP_FREQ) * 1000000;

// GPIO Pin Assignments
/***************************************************************************************************/
#define servo1_io 18  // servo1 (upper port)
#define servo2_io 5   // servo2 (lower port)

#define led1_io 0     // led channel 1 - top right - 12V - no resistor
#define led2_io 4     // led channel 2 - Lower right, tied to button - 3.3V - 1ohm 1W resistor in series

#define left_mot_dir_io 14    // left motor direction
#define left_mot_pwm_io 12    // left motor PWM
#define right_mot_dir_io 32   // right motor direction
#define right_mot_pwm_io 33   // right motor PWM

#define motors_nsleep_io 15   // motor sleep pin

#define current_lm1_io 36     // current sense left motor 1 
#define current_lm2_io 39     // current sense left motor 2 
#define current_rm1_io 34     // current sense right motor 1
#define current_rm2_io 35     // current sense right motor 2

#define fault_io 25           // fault sense - active low - low value indiates fault from one of the motor drivers

#define battery_volt_io 27    // analog pin to the battery level voltage

#define ledarray_clk_io  13   // pin to the clock input of the shift register that controls the 8 leds, rising edge trigger
#define ledarray_data_io 2    // pin to the data input of the shift register that controls the 8 leds

// define the rx and tx pins on the ESP
#define RXD2_IO 16
#define TXD2_IO 17

// pins for the ultrasonic sensor
#define ultrasonic_trig_io 26
#define ultrasonic_echo_io 22
/***************************************************************************************************/


// If using an IBus reciever
#ifdef I_BUS
IBusBM IBus; // IBUS object
// defines for uniformity across protocols
#define THR_CH 1
#define STR_CH 0
#define SER_CH 2
#define EXT_CH 3
#define LOW_VAL 1000
#define HIGH_VAL 2000
#endif

// If using an SBUS reciever
#ifdef S_BUS
SBUS rxsr(Serial2); //SBUS object
uint16_t channels[16];
bool failSafe;
bool lostFrame;
// defines for uniformity across protocols
#define THR_CH 2
#define STR_CH 1
#define SER_CH 0
#define EXT_CH 3
#define LOW_VAL 172
#define HIGH_VAL 1811
#endif

// If using an ELRS reciever
#ifdef ELRS
#define SBUS_BUFFER_SIZE 25
uint8_t _rcs_buf[25] {};
uint16_t _raw_rc_values[RC_INPUT_MAX_CHANNELS] {};
uint16_t _raw_rc_count{};
// defines for uniformity across protocols
#define THR_CH 2
#define STR_CH 3
#define SER_CH 1
#define EXT_CH 0
#define LOW_VAL 988
#define HIGH_VAL 2011
#endif

// center value for the control sticks
#define DEFAULT (HIGH_VAL + LOW_VAL)/2

//speed of sound in inches/us (microseconds)
#define SPEED_SOUND 0.013504

Servo servo1;  // create servo object to control servo channel 1
Servo servo2;  // create servo object to control servo channel 2

// Variables 
/***************************************************************************************************/
int fault;              // fault in motors indicator
int current_lm1;        // current values sensed from left motor 1
int current_lm2;        // current values sensed from left motor 2
int current_rm1;        // current values sensed from right motor 1
int current_rm2;        // current values sensed from right motor 2
 
// Radio channel integers - raw values from the radio
int ch1 = DEFAULT;  // set to middle value so crawler doesnt move on startup
int ch2 = DEFAULT; 
int ch3 = DEFAULT;
int ch4 = DEFAULT;
int ch5, ch6, ch7, ch8, ch9, ch10, ch11, ch12;

// Smoothed values to reduce current draw
int thr_smoothed = 0;
int str_smoothed = 0;
float servo1_smoothed = 1500;
float servo2_smoothed = 1500;

// Smoothing alpha values
float thr_alpha = 0.85;
float str_alpha = 0.85;
float servo_alpha = 0.7;

// Radio channel values mapped into useful numbers
int thr, str, servo1_val, servo2_val, led1, led2;
 
// Sensor values
int voltage_read = 0;
double battery_voltage = 0.0;

// LED array 
// led_array[8] = {orange, red, yellow, green, blue, red(rgb), green(rgb), blue(rgb)}
int led_array[8] = {0,0,0,0,0,0,0,1};

// Deadzone for the drive control stick - make larger if crawler is drifting with stick centered
const int deadzone_thr = 10;
const int deadzone_str = 10;

// values for the loop frequency locking
long int loop_time_elapsed, last_loop_time;


long int ultrasonic_duration;
float ultrasonic_distance;
int ultrasonic_loop_counter;
int ultrasonic_loop_speed = 5;

/***************************************************************************************************/

// Setup function that runs once as the ESP starts up
/***************************************************************************************************/
void setup() {
  #ifdef DEBUG
  Serial.begin(921600);     // debug info
  #endif

  #ifdef I_BUS
  IBus.begin(Serial2,1);    // iBUS object connected to serial2 RX2 pin and use timer 1
  #endif

  #ifdef S_BUS
  rxsr.begin(RXD2_IO, TXD2_IO, true, 100000); // optional parameters for ESP32: RX pin, TX pin, inverse mode, baudrate
  #endif

  #ifdef ELRS
  Serial1.begin(420000, SERIAL_8N1, RXD2_IO, TXD2_IO);  // connect to the ELRS Rx
  #endif
  
  // Setup the servo objects to the IO pins
  servo1.attach(servo1_io); // attaches servo1 pin
  servo2.attach(servo2_io); // attaches servo2 pin

  // Setup all output pins
  pinMode(led1_io, OUTPUT);
  pinMode(led2_io, OUTPUT);
  pinMode(ledarray_clk_io, OUTPUT);
  pinMode(ledarray_data_io, OUTPUT);
  pinMode(ultrasonic_trig_io, OUTPUT);
  pinMode(ultrasonic_echo_io, INPUT);

  // Motor driver pins set to OUTPUT
  pinMode(left_mot_dir_io, OUTPUT);
  pinMode(left_mot_pwm_io, OUTPUT);
  pinMode(right_mot_dir_io, OUTPUT);
  pinMode(right_mot_pwm_io, OUTPUT);
  pinMode(motors_nsleep_io, OUTPUT);

  // Setup all input pins
  pinMode(current_lm1_io, INPUT);
  pinMode(current_lm2_io, INPUT);
  pinMode(current_rm1_io, INPUT);
  pinMode(current_rm2_io, INPUT);
  pinMode(fault_io, INPUT);

  // Startup procedure required to intialize the motor drivers into the correct state
  digitalWrite(motors_nsleep_io, HIGH);
  delay(1);
  digitalWrite(motors_nsleep_io, LOW);
  delayMicroseconds(30);
  digitalWrite(motors_nsleep_io, HIGH);

  last_loop_time = micros();
} // end of setup()


// Loop that repeats forever within the ESP after setup runs once
/***************************************************************************************************/
void loop() {
  // 1st Section : READ VALUES AND GATHER INFORMATION 
  // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  read_receiver(&ch1, &ch2, &ch3, &ch4, &ch5, &ch6, &ch7, &ch8, &ch9, &ch10, &ch11, &ch12);

  // Read sensor values
  voltage_read = analogRead(battery_volt_io);
  current_lm1 = analogRead(current_lm1_io);
  current_lm2 = analogRead(current_lm2_io);
  current_rm1 = analogRead(current_rm1_io);
  current_rm2 = analogRead(current_rm2_io);
  fault = digitalRead(fault_io);


  // only measure the distance every ultrasonic_loop_speed loops
  if (ultrasonic_loop_counter == ultrasonic_loop_speed){
    digitalWrite(ultrasonic_trig_io, LOW);    // reset ultrasonic sensor
    delayMicroseconds(2);
    digitalWrite(ultrasonic_trig_io, HIGH);   // send trigger pulse
    delayMicroseconds(10);
    digitalWrite(ultrasonic_trig_io, LOW);

    // time echo pulse response with timeout for distance too far 
    ultrasonic_duration = pulseIn(ultrasonic_echo_io, HIGH, 5000);  

    // calculate the distance based on the round trip time it takes
    if(ultrasonic_duration != 0){
      ultrasonic_distance = ultrasonic_duration * (SPEED_SOUND/2);
    }
    else{ // ignore timouts from pulseIn
      ultrasonic_distance = 9999.0;
    }

    ultrasonic_loop_counter = 0;  // reset counter
  }
  else{
    ultrasonic_loop_counter++;    // increment counter
  }

  // 2nd Section : RE-MAP VALUES / MAKE VALUES CLEAN AND USEFUL
  // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  
  // Re-map all the collected values from the receiver into their useful ranges
  thr = constrain(map(ch1, LOW_VAL, HIGH_VAL, 110, -110), -100, 100);
  str = constrain(map(ch2, LOW_VAL, HIGH_VAL, 110, -110), -100, 100);
  servo1_val = constrain(map(ch3, LOW_VAL, HIGH_VAL, -5, 185), 0, 180);
  servo2_val = constrain(map(ch4, LOW_VAL, HIGH_VAL, -5, 185), 0, 180);
  led1 = constrain(map(ch5, LOW_VAL, HIGH_VAL, 0, 255), 0, 255);
  led2 = constrain(map(ch6, LOW_VAL, HIGH_VAL, 0, 255), 0, 255);

  // limit the throttle based on the ultrasonic distance
  if(ultrasonic_distance < 8.0){  // don't allow forward commands within this range
    thr = constrain(thr, 0, 100);
  }
  else if(ultrasonic_distance == 9999.0){ 
    // do nothing
  }
  else{ // limit max forward throttle to slow down close to objects - stronger when closer
    thr = constrain(thr, (-100 + ((24-ultrasonic_distance)*4)), 100);
  }

  // smooth out throttle and steering - higher alpha means smoother and slower reaction 
  thr_smoothed = (thr_smoothed * thr_alpha) + (thr * (1 - thr_alpha));
  str_smoothed = (str_smoothed * str_alpha) + (str * (1 - str_alpha));
  servo1_smoothed = (servo1_smoothed * servo_alpha) + (servo1_val * (1 - servo_alpha));
  servo2_smoothed = (servo2_smoothed * servo_alpha) + (servo2_val * (1 - servo_alpha));
 
  // Calculate the voltage based on the analog value
  battery_voltage = voltage_read / 270.0;

  


  // 3rd Section : ACTION TAKEN BASED VALUES / WRITE TO OUTPUTS
  // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 
  // send the values to the servos
  servo1.write(servo1_smoothed);
  servo2.write(servo2_smoothed);
 
  // control the LEDs
  analogWrite(led1_io, led1);
  analogWrite(led2_io, led2);

  // If battery is close to dead turn on red led
  if(battery_voltage < 10.0){
    led_array[1] = 1;
  }
  else{
    led_array[1] = 0;
  }

  // if fault from motor drivers, turn on orange led
  if(fault == 0){
    led_array[0] = 1;
  }
  else{
    led_array[0] = 0;
  }
 
  
  // handle motor controls
  if ((thr_smoothed > -deadzone_thr && thr_smoothed < deadzone_thr) && 
      (str_smoothed > -deadzone_thr && str_smoothed < deadzone_thr)){
    // deadzone to stop unwanted drift
    left_motors(0);
    right_motors(0);
  }
  else{ 
    // throttle mixing for tank/skid steering
    left_motors(thr_smoothed + str_smoothed);
    right_motors(thr_smoothed - str_smoothed);
  }

  // send the LED array values to the shift register
  ledarray_set(led_array);
 

  // 4th Section :  DEBUG VALUES TO THE SERIAL MONITOR
  // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  // only displays if DEBUG define is not commented out
  #ifdef DEBUG
  Serial.print("  Throttle: ");
  Serial.print(thr_smoothed);
  Serial.print("  Steering: ");
  Serial.print(str_smoothed);
  // Serial.print("  Servo1: " );
  // Serial.print(servo1_smoothed);
  // Serial.print("  Servo2: " );
  // Serial.print(servo2_smoothed);
  // Serial.print("  LED1: " );
  // Serial.print(led1);
  // Serial.print("  LED2: " );
  // Serial.print(led2);
  // Serial.print("  Fault: " );
  // Serial.print(fault);
  // Serial.print("  Cur lm1: " );
  // Serial.print(current_lm1);
  // Serial.print("  Cur lm2: " );
  // Serial.print(current_lm2);
  // Serial.print("  Cur rm1: " );
  // Serial.print(current_rm1);
  // Serial.print("  Cur rm2: " );
  // Serial.print(current_rm2);
  Serial.print("  Bat: " );
  Serial.print(battery_voltage);
  Serial.print("  Loop_time: " );
  Serial.print(loop_time_elapsed);
  Serial.print("  Dist: ");
  Serial.println(ultrasonic_distance);
  
  #endif

  // loop frequency timer
  loop_time_elapsed = micros() - last_loop_time;
  while(loop_time_elapsed < loop_interval_us){
    loop_time_elapsed = micros() - last_loop_time;
  }
  
  if (loop_time_elapsed < (loop_interval_us * 0.9)){
    led_array[2] = 1;
  }
  else{
    led_array[2] = 0;
  }

  last_loop_time = micros();

}// end of loop()
/***************************************************************************************************/


// Motor Control Functions
/***************************************************************************************************/
// Function to run the left motor channels
// Takes input from -100 to 100
// -100 = full reverse, 0 = hold, 100 = full forward
void left_motors(int l_thr){
  l_thr = constrain(l_thr, -100, 100);

  if (l_thr == 0){
    digitalWrite(left_mot_dir_io, LOW);
    analogWrite(left_mot_pwm_io, 0);
  }
  else if (l_thr > 0){
    int val = map(l_thr, 0, 100, 0, 255);
    digitalWrite(left_mot_dir_io, HIGH);
    analogWrite(left_mot_pwm_io, val);
  }
  else if (l_thr < 0){
    int val = map(l_thr, -100, 0, 255, 0);
    digitalWrite(left_mot_dir_io, LOW);
    analogWrite(left_mot_pwm_io, val);
  }
}

// Function to run the right motor channels
// Takes input from -100 to 100
// -100 = full reverse, 0 = hold, 100 = full forward
void right_motors(int r_thr){
  r_thr = constrain(r_thr, -100, 100);

 if (r_thr == 0){
    digitalWrite(right_mot_dir_io, LOW);
    analogWrite(right_mot_pwm_io, 0);
  }
  else if (r_thr > 0){
    int val = map(r_thr, 0, 100, 0, 255);
    digitalWrite(right_mot_dir_io, LOW);
    analogWrite(right_mot_pwm_io, val);
  }
  else if (r_thr < 0){
    int val = map(r_thr, -100, 0, 255, 0);
    digitalWrite(right_mot_dir_io, HIGH);
    analogWrite(right_mot_pwm_io, val);
  }
}


// Function to set the values in the led array
// leds[8] = {orange, red, yellow, green, blue, red(rgb), green(rgb), blue(rgb)}
// 1 = led on, 0 = led off
void ledarray_set(int leds[]){
  for (int i = 0; i < 8; i++){
    digitalWrite(ledarray_clk_io, LOW);
    digitalWrite(ledarray_data_io, leds[i]);
    digitalWrite(ledarray_clk_io, HIGH);
  }
}

// Function to reac values from the receivers and set them to channels
void read_receiver(int *ch1, int *ch2, int *ch3, int *ch4, int *ch5, int *ch6, int *ch7, int *ch8, int *ch9, int *ch10, int *ch11, int *ch12){
  // This section is called if the IBus protocol is selected
  #ifdef I_BUS
  *ch1 = IBus.readChannel(THR_CH);   // get latest value from channel 1
  if (*ch1 > 2100 || *ch1 < 500){    // lost connection, set to defaults
    *ch1 = DEFAULT;
  }
  *ch2 = IBus.readChannel(STR_CH);   // get latest value from channel 2
  if (*ch2 > 2100 || *ch2 < 500){    // lost connection, set to defaults
    *ch2 = DEFAULT;
  }
  *ch3 = IBus.readChannel(SER_CH);   // get latest value from channel 3
  if (*ch3 > 2100 || *ch3 < 500){    // lost connection, set to defaults
    *ch3 = DEFAULT;
  }
  *ch4 = IBus.readChannel(EXT_CH);   // get latest value from channel 4
  if (*ch4 > 2100 || *ch4 < 500){    // lost connection, set to defaults
    *ch4 = DEFAULT;
  }
  *ch5 = IBus.readChannel(4);  // get latest value from channel 5
  *ch6 = IBus.readChannel(5);  // get latest value from channel 6
  *ch7 = 0;
  *ch8 = 0;
  *ch9 = 0;
  *ch10 = 0;
  *ch11 = 0;
  *ch12 = 0;
  #endif

  // This section is called if the SBUS protocol is selected
  #ifdef S_BUS
  if(rxsr.read(&channels[0], &failSafe, &lostFrame)){
    *ch1 = channels[THR_CH];
    *ch2 = channels[STR_CH];
    *ch3 = channels[SER_CH];
    *ch4 = channels[EXT_CH];
    *ch5 = channels[4];
    *ch6 = channels[5];
    *ch7 = channels[6];
    *ch8 = channels[7];
    *ch9 = channels[8];
    *ch10 = channels[9];
    *ch11 = channels[10];
    *ch12 = channels[11];

    // sanitize values, usually only matters on controller turn off
    if(failSafe){
      *ch1 = DEFAULT;
      *ch2 = DEFAULT;
      *ch3 = GIMBAL_DEFAULT;
      *ch4 = DEFAULT;
      *ch8 = LOW_VAL;
      *ch9 = LOW_VAL;
      *ch11 = LOW_VAL;
    }
  }
  #endif

  // This section is called if the ELRS protocol is selected
  #ifdef ELRS
  size_t numBytesRead = Serial1.readBytes(_rcs_buf, BUFFER_SIZE);
  if(numBytesRead > 0){
    crsf_parse(&_rcs_buf[0], BUFFER_SIZE, &_raw_rc_values[0], &_raw_rc_count, RC_INPUT_MAX_CHANNELS );
    *ch1 = _raw_rc_values[THR_CH];
    *ch2 = _raw_rc_values[STR_CH];
    *ch3 = _raw_rc_values[SER_CH];
    *ch4 = _raw_rc_values[EXT_CH];
    *ch5 = _raw_rc_values[4];
    *ch6 = _raw_rc_values[5];
    *ch7 = _raw_rc_values[6];
    *ch8 = _raw_rc_values[7];
    *ch9 = _raw_rc_values[8];
    *ch10 = _raw_rc_values[9];
    *ch11 = _raw_rc_values[10];
    *ch12 = _raw_rc_values[11];
  }
  #endif
}

