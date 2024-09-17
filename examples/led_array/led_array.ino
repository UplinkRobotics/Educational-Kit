// Uplinkrobotics Educational Kit LED Array Example
// Program to demonstrate the use of LED array onboard the COR microcontroller 
// LEDs are controlled with a shift register and an active low signal
//
// Copyright © 2024 UplinkRobotics LLC

const int ledarray_clk_io  = 13;      // pin to the clock input of the shift register that controls the 8 leds, rising edge trigger
const int ledarray_data_io = 2;     // pin to the data input of the shift register that controls the 8 leds

//LED array values
// led_array[8] = {orange, red, yellow, green, blue, blue(rgb), green(rgb), red(rgb)}
int led_array[8] = {0,0,0,0,0,0,0,1};

void setup() {
  // put your setup code here, to run once:
  // setup the pins as outputs
  pinMode(ledarray_clk_io, OUTPUT);
  pinMode(ledarray_data_io, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  
  // sweep through the LEDs turning them all on
  for(int i = 0; i < 8; i++){
    led_array[i] = 1;
    ledarray_set(led_array);
    delay(500);
  }

  // sweep through all the LEDs turning them all off
  for(int i = 0; i < 8; i++){
    led_array[i] = 0;
    ledarray_set(led_array);
    delay(500);
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