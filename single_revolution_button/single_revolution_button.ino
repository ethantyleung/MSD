// Encoder Pins
const int ENCODER_A = 2;
const int ENCODER_B = 3; 

// Pins for Adafruit DRV8871 motor driver inputs
const int MOTOR_DRIVER_IN1 = 10;

// Encoder Data
volatile int encoderPos = 0;

void setup() {
  // Serial communication
  Serial.begin(115200);
  delay(1000); // Serial needs time to initialize connection
  // Setup for rotary encoder
  pinMode(ENCODER_A, INPUT);
  pinMode(ENCODER_B, INPUT);
  digitalWrite(ENCODER_A, HIGH);
  digitalWrite(ENCODER_B, HIGH);
  // Interrupt setup to call updateEncoder() on a rising edge from ENCODER_A pin
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), updateEncoder, RISING); 
  // Pull pin 6 low and set it as an input pin
  digitalWrite(6,LOW);
  pinMode(6, INPUT);
}

void loop() {
  // Check for a high signal indicating that the button has been pressed
  if(digitalRead(6)) {
    // Set motor output to max
    analogWrite(MOTOR_DRIVER_IN1,255);
    // Run the motor at max for one revolution (700 clicks per revolution)
    while(encoderPos > -69) {
    }
    // Turn the motor off after 700 clicks or one revolution
    analogWrite(MOTOR_DRIVER_IN1,0);
    encoderPos = 0;
    delay(50);
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