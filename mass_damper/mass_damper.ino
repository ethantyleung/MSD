#include <math.h>

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

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
const int MOTOR_DRIVER_IN1 = 11;
const int MOTOR_DRIVER_IN2 = 10;

// Encoder Data
volatile int encoderPos = 0; // Volatile type informs the compiler that this is a changing variable (hence the keyword volatile), so that it does not optimize it
int prevEncoderPos = 0;
double armHeight = 0;
double initDist = 0;

// Initialize Accelerometer Sensor Object
Adafruit_MPU6050 massMPU, rodMPU;
Adafruit_Sensor *massAccelerometer, *rodAccelerometer;

// MPU6050 I2C Addresses
const int MASS_ADDR = 0x68; // AD0 => GND
const int ROD_ADDR = 0x69;  // AD0 => 5V

// MPU6050 Calibration Data
const double MASS_OFFSET = 0.15;
const double ROD_OFFSET = 2.9;
const double MASS_SCALE = 0.09950248756;
const double ROD_SCALE = 0.09900990099;

// Shared time variable
unsigned long currentTime;

// Filtered output variables
double filteredDist = 0;
double filteredRodAccel = 0;
double filteredMassAccel = 0;

// Alpha values for filtering (lower alpha => more filtering and data lag)
double ultrasonicAlpha = 0.10; 
double accelAlpha = 0.10;

// Error-Catching Variables
double prevRodAccel = 0;
double prevMassAccel = 0;
int rodRepeats = 0;
int massRepeats = 0;

void setup() {
  // Serial communication
  Serial.begin(115200);

  // Setup for accelerometers
  if(!massMPU.begin(MASS_ADDR,&Wire,0) || !rodMPU.begin(ROD_ADDR,&Wire,1)) { // begin() returns false if MPU6050 is not found
    Serial.println("Failed to find MPU6050 chip");
    Serial.println("ABORTING...");
    while (1) {
      delay(10);
    }
  }

  massMPU.setAccelerometerRange(MPU6050_RANGE_4_G); // Set the accelerometer range to +- 4G (so 4*9.81m/s^2 is the max acceleration it can detect)
  rodMPU.setAccelerometerRange(MPU6050_RANGE_4_G); 
  massAccelerometer = massMPU.getAccelerometerSensor();
  rodAccelerometer = rodMPU.getAccelerometerSensor();

  // Setup an I2C timeout in microseconds
  Wire.setTimeout(3000);

  // Setup for ultrasonic sensor
  pinMode(TRIG_PIN, OUTPUT); // Sets the trigPin as an Output
  pinMode(ECHO_PIN, INPUT);  // Sets the echoPin as an Input

  // Setup for rotary encoder
  pinMode(ENCODER_A, INPUT);
  pinMode(ENCODER_B, INPUT);
  digitalWrite(ENCODER_A, HIGH);
  digitalWrite(ENCODER_B, HIGH);
  // Interrupt setup to call updateEncoder() on a rising edge from ENCODER_A pin
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), updateEncoder, FALLING); 

  //setup for motor driver
  pinMode(MOTOR_DRIVER_IN1, OUTPUT);
  pinMode(MOTOR_DRIVER_IN2, OUTPUT);

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

  // Get accelerometer data from the mpu6050 using getEvent call
  sensors_event_t massAccelData, rodAccelData;
  massAccelerometer->getEvent(&massAccelData);
  delay(10);
  rodAccelerometer->getEvent(&rodAccelData);

  // Parse the accelerometer data
  double massAccel = parseAccelData(massAccelData);
  double rodAccel = parseAccelData(rodAccelData);

  /* Error catching for frozen i2c bus
   * Arduino has a known issue with multiple peripherals on a shared I2C line which causes one device's data to hang, or "freeze".
   * A simple way to catch this is to increment a counter each time the rod acceleration does not change.
   * After the data hasn't changed for 3 cycles, we restart the MPU6050 to get the sensor unstuck.*/
  if(prevRodAccel == rodAccel) {
    rodRepeats++;
    // If the value has not changed in the last 3 polls, reset the frozen MPU6050
    if(rodRepeats >= 3) {
      rodMPU.begin(ROD_ADDR,&Wire,1);
      rodRepeats = 0;
    }
  } else {
    // Update number of repeats and previous rod acceleration value
    rodRepeats = 0;
    prevRodAccel = rodAccel;
  }
  // Same error catching for mass MPU6050
  if(prevMassAccel == massAccel) {
    massRepeats++;
    if(massRepeats >= 3) {
      massMPU.begin(MASS_ADDR,&Wire,0);
      massRepeats = 0;
    }
  } else {
    massRepeats = 0;
    prevMassAccel = massAccel;
  }

  // Apply accelerometer filters
  filteredMassAccel = filteredMassAccel*(1.0 - accelAlpha) + accelAlpha*(massAccel);
  filteredRodAccel = filteredRodAccel*(1.0 - accelAlpha) + accelAlpha*(rodAccel);

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


/*! @brief Converts raw accelerometer data to acceleration
    @returns Acceleration in m/s^2 */
double parseAccelData(sensors_event_t sensorData) {
  // Get raw Z-acceleration data from sensor event
  double accelData = sensorData.acceleration.z;
  // Calibrate and convert to G's (Mass Sensor ID = 0, Rod Sensor ID = 1)
  if(sensorData.sensor_id) {
    accelData = (accelData - ROD_OFFSET) * ROD_SCALE;
  } else {
    accelData = (accelData - MASS_OFFSET) * MASS_SCALE;
  }
  // Offset to account for acceleration due to gravity
  accelData--;
  return accelData * 9.81; // Scale by 9.81 m/s^2
}

void motorSpeedControl(){
  // Read value from potentiometer 
  int rawPotValue = analogRead(A0);
 
  // Map potentiometer value to PWM range 
  int PWM_PotValue = map(rawPotValue, 0, 715, 0, 255); // input range (0-715) is based on rawPotValue with 3V3 input to potentiometer

  // Motor direction depends on whether IN1 or IN2 on the motor driver is pulled low.  
  digitalWrite(MOTOR_DRIVER_IN2, LOW); 
  
  // PWM motor driver input to adjust motor speed
  analogWrite(MOTOR_DRIVER_IN1, PWM_PotValue);
}