const int ENCODER_A = 2;
const int ENCODER_B = 3; 

// Pins for Adafruit DRV8871 motor driver inputs
const int MOTOR_DRIVER_IN1 = 10;

// Encoder Data
volatile int encoderPos = 0;

void setup() {
  // Serial communication
  Serial.begin(115200);
  delay(1000); //Serial needs time to initialize connection
  // put your setup code here, to run once:
  // Setup for rotary encoder
  pinMode(ENCODER_A, INPUT);
  pinMode(ENCODER_B, INPUT);
  digitalWrite(ENCODER_A, HIGH);
  digitalWrite(ENCODER_B, HIGH);
  // Interrupt setup to call updateEncoder() on a rising edge from ENCODER_A pin
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), updateEncoder, RISING); 

  digitalWrite(7,LOW);
  pinMode(7, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(digitalRead(7)) {
    analogWrite(MOTOR_DRIVER_IN1,255);
    while(encoderPos > -700) {
    }
    analogWrite(MOTOR_DRIVER_IN1,0);
    delay(500);
    encoderPos = 0;
  }
}

/*! @brief Interrupt Service Routine on a rising edge from ENCODER_A
  */
void updateEncoder() {
  if (digitalRead(ENCODER_B) == LOW) { //encoder B contains direction data
    encoderPos++; // Encoder moved one click in CW direction
  } else {
    encoderPos--; // Encoder moved one click in the CCW direction
  }
}