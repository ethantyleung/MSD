#include <math.h>

// Ultrasonic sensor pins
const int TRIG_PIN = 12;
const int ECHO_PIN = 13;

// Rotary encoder settings
#define CLICKS_PER_REVOLUTION 700
#define PI 3.1415926
#define ARM_LENGTH 1.3 // cm
const int ENCODER_A = 2;
const int ENCODER_B = 3; 

// Pins for Adafruit DRV8871 motor driver inputs
const int MOTOR_DRIVER_IN1 = 10;

// Encoder Data
volatile int encoderPos = 0; // Volatile type informs the compiler that this is a changing variable (hence the keyword volatile), so that it does not optimize it
int prevEncoderPos = 0;
double armHeight = 0;
double initDist = 0;

// z-axis acceleration data from two MMA 7361 accelerometers
 struct {
     double z_Rod;
     double z_Mass;
 
   } z_Values;


// Shared time variable
unsigned long currentTime;

// Filtered output variables
double filteredDist = 0;
double filteredRodAccel = 0;
double filteredMassAccel = 0;

// Alpha values for filtering (lower alpha => more filtering and data lag)
double ultrasonicAlpha = 0.10; 
double accelAlpha = 0.10;

void setup() {
  // Serial communication
  Serial.begin(115200);
  delay(1000); //Serial needs time to initialize connection

  // Setup for ultrasonic sensor
  pinMode(TRIG_PIN, OUTPUT); // Sets the trigPin as an Output
  pinMode(ECHO_PIN, INPUT);  // Sets the echoPin as an Input

  // Setup for rotary encoder
  pinMode(ENCODER_A, INPUT);
  pinMode(ENCODER_B, INPUT);
  digitalWrite(ENCODER_A, HIGH);
  digitalWrite(ENCODER_B, HIGH);
  // Interrupt setup to call updateEncoder() on a rising edge from ENCODER_A pin
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), updateEncoder, RISING); 

  // Setup for PWM output pin
  pinMode(MOTOR_DRIVER_IN1, OUTPUT);

  initDist = measureDistance();
}

void loop() {
  // Update time
  currentTime = micros();

  // Update motor speed from potentiometer
  motorSpeedControl();

  // Measure distance using the ultrasonic sensor
  double distance = measureDistance() - initDist;
  filteredDist = filteredDist*(1.0 - ultrasonicAlpha) + ultrasonicAlpha*distance; 

  // // Calculate arm height and update position using the rotary encoder
  calculateArmHeight();
 
  //  Get accelerometer data
  updateAccelValue();

  // Apply accelerometer filters
   filteredRodAccel = filteredRodAccel*(1.0 - accelAlpha) + accelAlpha*(z_Values.z_Rod);
   filteredMassAccel = filteredMassAccel*(1.0 - accelAlpha) + accelAlpha*(z_Values.z_Mass);

  // Print results in Space-Seperated format:
  // time (ms) distance (cm) arm height (cm) rod acceleration (m/s^2) mass acceleration (m/s^2)
  Serial.print(currentTime / 1e6);
  Serial.print(" ");
  Serial.print(filteredDist);
  Serial.print(" ");
  Serial.print(armHeight);
  Serial.print(" ");
  Serial.print(filteredRodAccel);
  Serial.print(" ");
  Serial.println(filteredMassAccel);
  
}

/*! @brief Ping the ultrasonic sensor to collect distance data
    @returns Distance in cm */
double measureDistance() {
  // Pull TRIG_PIN low to start transmission with ultrasonic sensor
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 microseconds
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  // Conversion from travel time to distance using speed of sound
  double distance = duration * 0.0344 / 2;
  return distance;
}

/*! @brief Updates current arm height based on the angular position of the motor arm
    */
void calculateArmHeight() {
  // Disable interrupts while reading current encoder position to preserve its value
  noInterrupts();
  int currentPos = encoderPos;
  interrupts();
  // If the position has changed, then update the current position
  if (currentPos != prevEncoderPos) {
    /*One revolution = 2PI, 700 Clicks per revolution
      Find equivalent decimal value of current position (in clicks) using currentPos/click per revolution
      2PI * equivalent decimal gives angular position (in radians) */
    double angle = 2 * PI * currentPos / (double)CLICKS_PER_REVOLUTION;
    // The height of the arm is = length of the arm * sin(angular position)
    armHeight = ARM_LENGTH * sin(angle);
    prevEncoderPos = currentPos;
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

/*! @brief Adjusts motor speed with potentiometer knob. 
*/
void motorSpeedControl(){
  // Read value from potentiometer 
  int rawPotValue = analogRead(A2);

 
  // Map potentiometer value to PWM range 
  int PWM_PotValue = map(rawPotValue, 0, 715, 0, 255); // input range (0-715) is based on rawPotValue with 3V3 input to potentiometer
  
  // PWM motor driver input to adjust motor speed
  analogWrite(MOTOR_DRIVER_IN1, PWM_PotValue);
}

/*! @brief Reads Z-axis analog acceleration values from two accelerometers in m/s^2
*/
 void updateAccelValue(){
   const int rodAccel_offset = 279; // Offset and scale factor determined from manual calibration
   const int massAccel_offset = 284;
   const double rodAccel_scale = 0.005988023952;
   const double massAccel_scale = 0.006024096386;
   
   z_Values.z_Rod = analogRead(A0);
   z_Values.z_Rod = ((double)((z_Values.z_Rod - rodAccel_offset)*rodAccel_scale) - 1.0)*9.81;
 
   z_Values.z_Mass = analogRead(A1); 
   z_Values.z_Mass = ((double)((z_Values.z_Mass - massAccel_offset)*massAccel_scale) - 1.0)*9.81;
 }
