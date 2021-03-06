// Arduino Quadcopter Controller
// Anish Athalye, Logan Engstrom
//
// Controller for Hubsan X4
// Using Arduino Uno

// using PWM output with low pass filter to output analog
// using analog inputs for feedback control to set values correctly

#define PIN_OUT_ROLL (3)
#define PIN_IN_ROLL (A1)
#define PIN_OUT_PITCH (5)
#define PIN_IN_PITCH (A2)
#define PIN_OUT_YAW (9)
#define PIN_IN_YAW (A3)
#define PIN_OUT_THROTTLE (10)
#define PIN_IN_THROTTLE (A4)

#define VOLTAGE_SCALE (5.0 / 1023.0)

#define CONTROL_HIGH (3.25) // volts
#define CONTROL_LOW (0.0) // volts
#define CONTROL_MID ((CONTROL_HIGH - CONTROL_LOW) / 2.0) // volts

#define BYTE_MAX (255)

#define LOOP_STEP (2000) // microseconds

#define PIN_LED (13)
#define PIN_CONTROL (2) // ground to enable, high to disable

float pidUpdate(float setPoint,
                float measuredValue,
                float *previousError,
                float *integral,
                unsigned long *lastSet,
                float kp,
                float ki,
                float kd) {
  unsigned long currTime = millis();
  float dt = ((float) (currTime - *lastSet)) / 1000.0; // dt in seconds
  *lastSet = currTime;
  float error = setPoint - measuredValue;
  *integral = *integral + error * dt;
  float derivative = (error - *previousError) / dt;
  float output = kp * error + ki * (*integral) + kd * derivative;
  if (output < 0.0) {
    output = 0.0;
  }
  if (output > 255.0) {
    output = 255.0;
  }
  return output;
}

float rollTarget = CONTROL_MID;
float rollError = 0;
float rollIntegral = 0;
unsigned long rollLastSet = 0;
float pitchTarget = CONTROL_MID;
float pitchError = 0;
float pitchIntegral = 0;
unsigned long pitchLastSet = 0;
float yawTarget = CONTROL_MID;
float yawError = 0;
float yawIntegral = 0;
unsigned long yawLastSet = 0;
float throttleTarget = CONTROL_LOW;
float throttleError = 0;
float throttleIntegral = 0;
unsigned long throttleLastSet = 0;

#define Kp (50.0)
#define Ki (100.0)
#define Kd (0.4)

void pid() {
  float roll = analogRead(PIN_IN_ROLL) * VOLTAGE_SCALE;
  float pitch = analogRead(PIN_IN_PITCH) * VOLTAGE_SCALE;
  float yaw = analogRead(PIN_IN_YAW) * VOLTAGE_SCALE;
  float throttle = analogRead(PIN_IN_THROTTLE) * VOLTAGE_SCALE;

  float rollOutput = pidUpdate(rollTarget, roll, &rollError, &rollIntegral, &rollLastSet, Kp, Ki, Kd);
  float pitchOutput = pidUpdate(pitchTarget, pitch, &pitchError, &pitchIntegral, &pitchLastSet, Kp, Ki, Kd);
  float yawOutput = pidUpdate(yawTarget, yaw, &yawError, &yawIntegral, &yawLastSet, Kp, Ki, Kd);
  float throttleOutput = pidUpdate(throttleTarget, throttle, &throttleError, &throttleIntegral, &throttleLastSet, Kp, Ki, Kd);

  analogWrite(PIN_OUT_ROLL, rollOutput);
  analogWrite(PIN_OUT_PITCH, pitchOutput);
  analogWrite(PIN_OUT_YAW, yawOutput);
  analogWrite(PIN_OUT_THROTTLE, throttleOutput);
}

void receiveCommands() {
  // commands are two bytes, of the form XY
  // X is a single character, 'R', 'P', 'Y', 'T'
  // for roll, pitch, yaw, throttle
  // Y is a single byte, 0x00 to 0xff
  // for roll, pitch, yaw, 0x00 is min, 0x80 is center, 0xff is max
  // for throttle, 0x00 is min and 0xff is max
  while (Serial.available() > 1) {
    byte channel = Serial.read();
    if (channel != 'R' && channel != 'P' && channel != 'Y' && channel != 'T') {
      // this shouldn't happen
      // something weird is going on -- maybe we're out of sync
      // let's skip this byte and keep going
      continue;
    }
    byte value = Serial.read();
    switch (channel) {
      case 'R':
        // roll control is inverted
        rollTarget = CONTROL_HIGH * (BYTE_MAX - value) / BYTE_MAX;
        break;
      case 'P':
        // pitch control is inverted
        pitchTarget = CONTROL_HIGH * (BYTE_MAX - value) / BYTE_MAX;
        break;
      case 'Y':
        yawTarget = CONTROL_HIGH * value / BYTE_MAX;
        break;
      case 'T':
        throttleTarget = CONTROL_HIGH * value / BYTE_MAX;
        break;
    }
  }
}

bool controlEnabled = 0;

void controlOff() {
  analogWrite(PIN_OUT_ROLL, CONTROL_MID);
  analogWrite(PIN_OUT_PITCH, CONTROL_MID);
  analogWrite(PIN_OUT_YAW, CONTROL_MID);
  analogWrite(PIN_OUT_THROTTLE, CONTROL_LOW);
  controlEnabled = 0;
}

void controlOn() {
  if (controlEnabled == 0) {
    unsigned long now = millis();
    rollError = 0;
    rollIntegral = 0;
    rollLastSet = now;
    pitchError = 0;
    pitchIntegral = 0;
    pitchLastSet = now;
    yawError = 0;
    yawIntegral = 0;
    yawLastSet = now;
    throttleError = 0;
    throttleIntegral = 0;
    throttleLastSet = now;
  }
  controlEnabled = 1;
}


void loop() {
  static unsigned long prevLoopTime = 0;
  unsigned long time = micros();
  while (time - prevLoopTime < LOOP_STEP && time > prevLoopTime) {
    time = micros(); // busy wait
  }
  prevLoopTime = time;

  receiveCommands();
  if (digitalRead(PIN_CONTROL) == LOW) {
    digitalWrite(PIN_LED, HIGH);
    controlOn();
    pid();
  } else {
    digitalWrite(PIN_LED, LOW);
    controlOff();
  }
}

void setup() {
  // initialize the LED pin as an output:
  pinMode(PIN_LED, OUTPUT);
  // initialize the pushbutton pin as an input:
  pinMode(PIN_CONTROL, INPUT);
  controlOff();

  Serial.begin(9600); // init serial connection at 9600 baud
}
