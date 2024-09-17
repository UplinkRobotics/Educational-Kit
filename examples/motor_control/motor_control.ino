// Uplinkrobotics Educational Kit Motor Control Example
// Program to demonstrate the use of PWM signals to run the motor drivers and power brushed motors
// Runs the right motors and then the left motors at varying speeds
//
// Copyright © 2024 UplinkRobotics LLC

// hard-wired pin assignments for the COR microncontroller board
const int left_mot_dir_io = 14; // left motor direction
const int left_mot_pwm_io = 12; // left motor PWM
const int right_mot_dir_io = 32; // right motor direction
const int right_mot_pwm_io = 33; // right motor PWM

const int motors_nsleep_io = 15; // motor sleep pin

const int fault_io = 25;       // fault sense - active low - low value indicates a fault from one of the motor drivers

void setup() {
  // put your setup code here, to run once:
  // Motor driver pins set to OUTPUT
  pinMode(left_mot_dir_io, OUTPUT);
  pinMode(left_mot_pwm_io, OUTPUT);
  pinMode(right_mot_dir_io, OUTPUT);
  pinMode(right_mot_pwm_io, OUTPUT);
  pinMode(motors_nsleep_io, OUTPUT);

  // setup the fault detection pin
  pinMode(fault_io, INPUT);

  // Startup the motor drivers with a 30us pulse on the motor enable pin after holding high for 1 second
  digitalWrite(motors_nsleep_io, HIGH);   // hold high for 1 second
  delay(1);
  digitalWrite(motors_nsleep_io, LOW);    // pulse low for 30us to enable
  delayMicroseconds(30);
  digitalWrite(motors_nsleep_io, HIGH);
}


void loop() {
  // put your main code here, to run repeatedly:

  // run left motors in one direction
  digitalWrite(left_mot_dir_io, LOW);

  // run through the full PWM sweep from duty cycle 0% to 100%
  for (int thr_left = 0; thr_left < 255; thr_left++){
    analogWrite(left_mot_pwm_io, thr_left);
    delay(20);
  }
  analogWrite(left_mot_pwm_io, 0); // stop the motors

  delay(500);

  // run left motors in the other direction
  digitalWrite(left_mot_dir_io, HIGH);

  // run through the full PWM sweep from duty cycle 0% to 100%
  for (int thr_left = 0; thr_left < 255; thr_left++){
    analogWrite(left_mot_pwm_io, thr_left);
    delay(20);
  }
  analogWrite(left_mot_pwm_io, 0); // stop the motors

  delay(500);




  // run right motors in one direction
  digitalWrite(right_mot_dir_io, LOW);

  // run through the full PWM sweep from duty cycle 0% to 100%
  for (int thr_right = 0; thr_right < 255; thr_right++){
    analogWrite(right_mot_pwm_io, thr_right);
    delay(20);
  }
  analogWrite(right_mot_pwm_io, 0); // stop the motors

  delay(500);

  // run right motors in the other direction
  digitalWrite(right_mot_dir_io, HIGH);

  // run through the full PWM sweep from duty cycle 0% to 100%
  for (int thr_right = 0; thr_right < 255; thr_right++){
    analogWrite(right_mot_pwm_io, thr_right);
    delay(20);
  }
  analogWrite(right_mot_pwm_io, 0); // stop the motor

  delay(500);

}
