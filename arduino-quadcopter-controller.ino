// Arduino Quadcopter Controller
// Anish Athalye, Logan Engstrom
//
// Controller for Hubsan X4
// Using Arduino Uno

// using PWM output with low pass filter to output analog
// using analog inputs for feedback control to set values correctly

#define pin_out_roll (3)
#define pin_in_roll (A1)
#define pin_out_pitch (5)
#define pin_in_pitch (A2)
#define pin_out_yaw (9)
#define pin_in_yaw (A3)
#define pin_out_throttle (10)
#define pin_in_throttle (A4)

#define power_up_wait_sec 2

void setup() {
  Serial.begin(9600); // init serial connection at 9600 baud
  analogWrite(pin_out_throttle, 0); // throttle output to low
  delay(power_up_wait_sec * 1000);
}

void loop() {
  analogWrite(pin_out_throttle, 0);
  delay(500);
  float sensorValue = analogRead(pin_in_throttle);
  sensorValue = sensorValue * 5.0 / 1024.0;
  Serial.println(sensorValue);
}
